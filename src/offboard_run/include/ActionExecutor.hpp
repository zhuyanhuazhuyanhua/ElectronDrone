#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <queue>

#include "DroneAction.hpp"
#include "Eigen/Dense"
#include "Eigen/src/Geometry/Quaternion.h"
#include "ros/console.h"
#include "ros/publisher.h"
#include "std_msgs/String.h"
#include "tf2/LinearMath/Matrix3x3.h"

class Vec3dPID {
 public:
  Vec3dPID(double p, double i, double d)
      : p_gain_(p),
        i_gain_(i),
        d_gain_(d),
        integral_(0.0, 0.0, 0.0),
        last_error_(0.0, 0.0, 0.0) {}

  Eigen::Vector3d update(const Eigen::Vector3d &error, double dt) {
    integral_ += error * dt;
    Eigen::Vector3d derivative = (error - last_error_) / dt;
    last_error_ = error;
    return p_gain_ * error + i_gain_ * integral_ + d_gain_ * derivative;
  }

  Eigen::Vector3d update(const Eigen::Vector3d &error, ros::Time time_stamp) {
    // 重载函数，使用默认dt
    if (last_time_.isZero()) {
      last_time_ = time_stamp;
      return Eigen::Vector3d::Zero();
    }
    double dt = (time_stamp - last_time_).toSec();
    last_time_ = time_stamp;
    if (dt <= 0.0) {
      ROS_WARN("Vec3dPID: dt is zero or negative, returning zero output.");
      return Eigen::Vector3d::Zero();
    }
    return update(error, dt);  // 默认50Hz
  }

 private:
  double p_gain_;
  double i_gain_;
  double d_gain_;
  ros::Time last_time_;
  Eigen::Vector3d integral_;
  Eigen::Vector3d last_error_;
};

class ActionExecutor {
 public:
  ActionExecutor(ros::NodeHandle &nh, tf2_ros::Buffer &tf_buffer)
      : nh_(nh), tf_buffer_(tf_buffer) {
    // 初始化发布器
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
    Step_pub_ = nh_.advertise<std_msgs::String>("/step", 10);

    // 初始化订阅器
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, &ActionExecutor::stateCb, this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10, &ActionExecutor::poseCb, this);
    camera_aim_sub_ = nh_.subscribe<geometry_msgs::Point>(
        "/camera_aiming_center", 10, &ActionExecutor::cameraAimCb, this);

    // 初始化服务客户端
    arming_client_ =
        nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    land_client_ =
        nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    // 初始化定时器
    // control_timer_ = nh_.createTimer(
    //     ros::Duration(0.02), &ActionExecutor::controlLoop, this); // 50Hz
    ROS_INFO("ActionExecutor initialized, ready to execute actions.");
  }

  // 添加动作到队列
  void addAction(std::shared_ptr<DroneAction> action) {
    action_queue_.push(action);
    ROS_INFO("Added action to queue. Queue size: %zu", action_queue_.size());
  }

  // 清空动作队列
  void clearActions() {
    while (!action_queue_.empty()) {
      action_queue_.pop();
    }
    current_action_.reset();
  }

  // 紧急停止
  void emergencyStop() {
    clearActions();
    // 发送当前位置作为目标，实现悬停
    if (current_pose_received_) {
      sendPositionSetpoint(current_pose_);
    }
  }

  // 发送位置设定点
  void sendPositionSetpoint(const geometry_msgs::PoseStamped &pose) {
    mavros_msgs::PositionTarget pos_target;
    pos_target.header = pose.header;
    pos_target.header.stamp = ros::Time::now();
    pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                           mavros_msgs::PositionTarget::IGNORE_VY |
                           mavros_msgs::PositionTarget::IGNORE_VZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY |
                           mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    pos_target.position = pose.pose.position;

    // 从四元数提取yaw
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pos_target.yaw = yaw;

    setpoint_pub_.publish(pos_target);
  }

  Eigen::Vector3d BodyVelocity2ENU(const Eigen::Vector3d &body_vec) {
    // 将机体坐标系下的速度转换为NED坐标系
    Eigen::Vector3d enu_vec;
    Eigen::Quaterniond q_current = Eigen::Quaterniond(
        current_pose_.pose.orientation.w, current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y, current_pose_.pose.orientation.z);
    enu_vec = q_current * body_vec;  // 转到ENU
    // ROS_INFO("ENU vector: [%f, %f, %f]", enu_vec.x(), enu_vec.y(),
    // enu_vec.z()); Eigen::Vector3d ned_vec(enu_vec.y(), enu_vec.x(),
    // -enu_vec.z());
    return enu_vec;
  }

  // 主控制循环
  void controlLoop() {
    std::cout << "Executing control loop, action ID: " << action_id_
              << std::endl;
    // 检查是否连接并解锁
    if (!current_state_.connected || !current_state_.armed) {
      return;
      ROS_WARN("Drone not connected or not armed, skipping control loop.");
    }
    ROS_INFO("Executing control loop, action ID: %d", action_id_);

    std_msgs::String step_msg;
    step_msg.data = std::to_string(action_id_);
    Step_pub_.publish(step_msg);

    // 如果当前没有执行的动作，从队列中取出下一个
    if (!current_action_ && !action_queue_.empty()) {
      current_action_ = action_queue_.front();
      action_queue_.pop();
      action_id_++;
      current_action_->setStatus(ActionStatus::EXECUTING);
      current_action_->setStartTime(ros::Time::now());
      ROS_INFO("Starting new action: %d",
               static_cast<int>(current_action_->getType()));
    }

    // 执行当前动作
    if (current_action_) {
      executeAction(current_action_);
    } else {
      // 没有动作时，保持当前位置
      if (current_pose_received_) {
        sendPositionSetpoint(current_pose_);
      }
    }
  }

  // 执行单个动作
  void executeAction(std::shared_ptr<DroneAction> action) {
    switch (action->getType()) {
      case ActionType::MOVE_TO_POSITION:
        executeMoveToPosition(action);
        break;
      case ActionType::HOVER:
        executeHover(action);
        break;
      case ActionType::CAMERA_AIM:
        executeCameraAim(action);
        break;
      case ActionType::LAND:
        executeLand(action);
        break;
      case ActionType::TAKEOFF:
        executeTakeoff(action);
        break;
    }
  }

  void sendDummyPose() {
    if (current_pose_received_) {
      sendPositionSetpoint(current_pose_);
    } else {
      geometry_msgs::PoseStamped dummy_pose;
      dummy_pose.header.frame_id = "world_enu";
      dummy_pose.header.stamp = ros::Time::now();
      dummy_pose.pose.position.x = 0.0;
      dummy_pose.pose.position.y = 0.0;
      dummy_pose.pose.position.z = 0.0;
      dummy_pose.pose.orientation.w = 1.0;  // 无旋转
      sendPositionSetpoint(dummy_pose);
    }
  }

 private:
  // 执行移动到位置
  void executeMoveToPosition(std::shared_ptr<DroneAction> action) {
    geometry_msgs::PoseStamped target = action->getTargetPose();

    // 如果需要坐标转换
    if (action->useBodyFrame() == DroneAction::Frame::WORLD_BODY) {
      try {
        target = tf_buffer_.transform(target, "world_enu", ros::Duration(0.1));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        return;
      }
    } else if (action->useBodyFrame() == DroneAction::Frame::BODY) {
      Eigen::Vector3d X = BodyVelocity2ENU(
          Eigen::Vector3d(target.pose.position.x, target.pose.position.y,
                          target.pose.position.z));
      target = last_finish_pose_;
      target.pose.position.x += X.x();
      target.pose.position.y += X.y();
      target.pose.position.z += X.z();
    }
    // 检查是否到达目标位置
    SpatialPoint current(current_pose_);
    SpatialPoint target_point(target);

    if (current.distance(target_point) < action->getPositionTolerance()) {
      aim_close_count_++;
      if (aim_close_count_ > 20) {
        action->setStatus(ActionStatus::COMPLETED);
        current_action_.reset();
        last_finish_pose_ = current_pose_;
        ROS_INFO("Reached target position");
        aim_close_count_ = 0;
      }
    } else {
      aim_close_count_ = 0;
    }
    // 发送位置指令
    sendPositionSetpoint(target);
  }

  // 执行悬停
  void executeHover(std::shared_ptr<DroneAction> action) {
    // 保持上个动作结束位置
    sendPositionSetpoint(last_finish_pose_);

    // 检查悬停时间
    if ((ros::Time::now() - action->getStartTime()).toSec() >
        action->getHoverTime()) {
      action->setStatus(ActionStatus::COMPLETED);
      current_action_.reset();
      // last_finish_pose_ = current_pose_; // 不更改上次结束位置
      ROS_INFO("Hover completed");
    }
  }

  // 执行相机瞄准
  void executeCameraAim(std::shared_ptr<DroneAction> action) {
    // 检查相机数据是否有效
    if (ros::Time::now() - last_camera_aim_time_ > ros::Duration(0.5)) {
      // 相机数据超时，执行普通位置控制
      // TODO 添加超时多次退出
      ROS_WARN("Camera aim data timeout, executing position control!");
      executeMoveToPosition(action);
      action->setStatus(ActionStatus::COMPLETED);
      current_action_.reset();
      aim_close_count_ = 0;
      return;
    }

    // 检查是否已经对准
    if (std::abs(camera_aim_diff_.x) < action->getCameraAimTolerance() &&
        std::abs(camera_aim_diff_.y) < action->getCameraAimTolerance()) {
      aim_close_count_++;
      if (aim_close_count_ > 20) {
        action->setStatus(ActionStatus::COMPLETED);
        current_action_.reset();
        last_finish_pose_ = current_pose_;
        ROS_INFO("Camera aim completed");
        aim_close_count_ = 0;
      }
    } else {
      aim_close_count_ = 0;
    }

    Eigen::Vector3d camera_aim_vector{camera_aim_diff_.x, camera_aim_diff_.y,
                                      camera_aim_diff_.z};

    ROS_INFO("Camera aim vector: [%f, %f, %f]", camera_aim_vector.x(),
             camera_aim_vector.y(), camera_aim_vector.z());

    // 发送速度控制指令进行对准
    static constexpr double P_gain = 0.005;
    static constexpr double MAX_STEP = 0.05;  // 最大单步移动距离（米）

    // 计算机体坐标系下的位置增量
    // Eigen::Vector3d body_position_delta = camera_aim_vector * P_gain;
    Eigen::Vector3d body_position_delta = pid_cam_aim_.update(
        camera_aim_vector, ros::Time::now());  // 使用PID控制

    // 限制最大移动步长
    for (int i = 0; i < 3; i++) {
      if (body_position_delta[i] > MAX_STEP) {
        body_position_delta[i] = MAX_STEP;
      } else if (body_position_delta[i] < -MAX_STEP) {
        body_position_delta[i] = -MAX_STEP;
      }
    }

    Eigen::Vector3d enu_position_delta = BodyVelocity2ENU(body_position_delta);

    // 计算目标位置
    geometry_msgs::PoseStamped target_pose = current_pose_;
    target_pose.pose.position.x += enu_position_delta.x();
    target_pose.pose.position.y += enu_position_delta.y();
    target_pose.pose.position.z += enu_position_delta.z();

    switch (action->getHoldAxis()) {
      case HoldAxis::X:
        target_pose.pose.position.x = action->getTargetPose().pose.position.x;
        break;
      case HoldAxis::Y:
        target_pose.pose.position.y = action->getTargetPose().pose.position.y;
        break;
      case HoldAxis::Z:
        target_pose.pose.position.z = action->getTargetPose().pose.position.z;
        break;
      default:
        break;
    }
    ROS_INFO("PID OUTOUT: [%.3f, %.3f, %.3f]", body_position_delta.x(),
             body_position_delta.y(), body_position_delta.z());
    ROS_INFO("Current position: [%.2f, %.2f, %.2f]",
             current_pose_.pose.position.x, current_pose_.pose.position.y,
             current_pose_.pose.position.z);
    ROS_INFO("Target position: [%.2f, %.2f, %.2f]", target_pose.pose.position.x,
             target_pose.pose.position.y, target_pose.pose.position.z);
    ROS_INFO("Position delta (ENU): [%.3f, %.3f, %.3f]", enu_position_delta.x(),
             enu_position_delta.y(), enu_position_delta.z());

    // 发送位置指令
    sendPositionSetpoint(target_pose);
  }

  // 执行降落
  void executeLand(std::shared_ptr<DroneAction> action) {
    // 切换到降落模式
    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "AUTO.LAND";

    if (set_mode_client_.call(land_mode) && land_mode.response.mode_sent) {
      action->setStatus(ActionStatus::COMPLETED);
      current_action_.reset();
      last_finish_pose_ = current_pose_;
      ROS_INFO("Landing initiated");
    }
  }

  // 执行起飞
  void executeTakeoff(std::shared_ptr<DroneAction> action) {
    geometry_msgs::PoseStamped takeoff_pose;
    if (action->isStartPoseInitialized()) {
      takeoff_pose = action->getStartPose();
    } else {
      action->setStartPose(current_pose_);
      takeoff_pose = current_pose_;
    }
    takeoff_pose.pose.position.z = action->getTargetAltitude();
    // takeoff_pose.pose.orientation.x = 0.0;
    // takeoff_pose.pose.orientation.y = 0.0;
    // takeoff_pose.pose.orientation.z = 0.0;
    // takeoff_pose.pose.orientation.w = 1.0; // 无旋转

    ROS_INFO("Sending takeoff command : [%.2f, %.2f, %.2f]",
             takeoff_pose.pose.position.x, takeoff_pose.pose.position.y,
             takeoff_pose.pose.position.z);
    SpatialPoint current(current_pose_);
    SpatialPoint target(takeoff_pose);

    if (std::abs(current.z - target.z) < action->getPositionTolerance()) {
      action->setStatus(ActionStatus::COMPLETED);
      current_action_.reset();
      last_finish_pose_ = current_pose_;
      ROS_INFO("Takeoff completed");
    } else {
      sendPositionSetpoint(takeoff_pose);
    }
  }

  // 回调函数
  void stateCb(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
  }

  void poseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose_ = *msg;
    current_pose_received_ = true;
  }

  void cameraAimCb(const geometry_msgs::Point::ConstPtr &msg) {
    camera_aim_diff_ = *msg;
    last_camera_aim_time_ = ros::Time::now();
  }

 private:
  ros::NodeHandle &nh_;
  tf2_ros::Buffer &tf_buffer_;

  Vec3dPID pid_cam_aim_{0.001, 0.0, 0.0001};

  // 发布器和订阅器
  ros::Publisher setpoint_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber camera_aim_sub_;
  ros::Publisher Step_pub_;
  // 服务客户端
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient land_client_;

  // 定时器
  ros::Timer control_timer_;

  // 状态信息
  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::Point camera_aim_diff_;
  ros::Time last_camera_aim_time_;
  bool current_pose_received_ = false;
  int aim_close_count_ = 0;

  //降落相关
  struct LandingState {
    bool initialized = false;
    geometry_msgs::PoseStamped start_pose;
    ros::Time start_time;
    double ground_height = 0.0;  // 地面高度
    int stable_count = 0;        // 稳定计数器
  } landing_state_;

  // 动作队列
  std::queue<std::shared_ptr<DroneAction>> action_queue_;
  std::shared_ptr<DroneAction> current_action_;
  geometry_msgs::PoseStamped
      last_finish_pose_;  // 完成上一动作时的pose，用于hover
  int action_id_ = 0;
};