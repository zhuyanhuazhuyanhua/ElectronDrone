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
    if (action->useBodyFrame()) {
      try {
        target = tf_buffer_.transform(target, "world_enu", ros::Duration(0.1));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        return;
      }
    }

    // 检查是否到达目标位置
    SpatialPoint current(current_pose_);
    SpatialPoint target_point(target);

    if (current.distance(target_point) < action->getPositionTolerance()) {
      aim_close_count_++;
      if (aim_close_count_ > 20) {
        action->setStatus(ActionStatus::COMPLETED);
        current_action_.reset();
        finish_pose_ = current_pose_;
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
    sendPositionSetpoint(finish_pose_);

    // 检查悬停时间
    if ((ros::Time::now() - action->getStartTime()).toSec() >
        action->getHoverTime()) {
      action->setStatus(ActionStatus::COMPLETED);
      current_action_.reset();
      finish_pose_ = current_pose_;
      ROS_INFO("Hover completed");
    }
  }

  // 执行相机瞄准
  void executeCameraAim(std::shared_ptr<DroneAction> action) {
    // 检查相机数据是否有效
    if (ros::Time::now() - last_camera_aim_time_ > ros::Duration(0.5)) {
      // 相机数据超时，执行普通位置控制
      ROS_WARN("Camera aim data timeout, executing position control!");
      executeMoveToPosition(action);
      return;
    }

    // 检查是否已经对准
    if (std::abs(camera_aim_diff_.x) < action->getCameraAimTolerance() &&
        std::abs(camera_aim_diff_.y) < action->getCameraAimTolerance()) {
      aim_close_count_++;
      if (aim_close_count_ > 20) {
        action->setStatus(ActionStatus::COMPLETED);
        current_action_.reset();
        finish_pose_ = current_pose_;
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
    mavros_msgs::PositionTarget vel_cmd;
    static const double P_gain = 0.001;
    Eigen::Vector3d ned_velocity = BodyVelocity2ENU(camera_aim_vector * P_gain);
    vel_cmd.header.stamp = ros::Time::now();
    vel_cmd.header.frame_id = "world_enu";
    vel_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    vel_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW;

    // P控制器
    vel_cmd.velocity.x = ned_velocity.x();
    vel_cmd.velocity.y = ned_velocity.y();
    vel_cmd.velocity.z = ned_velocity.z();
    ROS_INFO("Camera aim velocity: [%f, %f, %f]", vel_cmd.velocity.x,
             vel_cmd.velocity.y, vel_cmd.velocity.z);

    setpoint_pub_.publish(vel_cmd);
  }

  // 执行降落
  void executeLand(std::shared_ptr<DroneAction> action) {
    // 切换到降落模式
    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "AUTO.LAND";

    if (set_mode_client_.call(land_mode) && land_mode.response.mode_sent) {
      action->setStatus(ActionStatus::COMPLETED);
      current_action_.reset();
      finish_pose_ = current_pose_;
      ROS_INFO("Landing initiated");
    }
  }

  // 执行起飞
  void executeTakeoff(std::shared_ptr<DroneAction> action) {
    geometry_msgs::PoseStamped takeoff_pose = current_pose_;
    takeoff_pose.pose.position.z = action->getTargetAltitude();

    SpatialPoint current(current_pose_);
    SpatialPoint target(takeoff_pose);

    if (std::abs(current.z - target.z) < action->getPositionTolerance()) {
      action->setStatus(ActionStatus::COMPLETED);
      current_action_.reset();
      finish_pose_ = current_pose_;
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

  // 动作队列
  std::queue<std::shared_ptr<DroneAction>> action_queue_;
  std::shared_ptr<DroneAction> current_action_;
  geometry_msgs::PoseStamped finish_pose_;  // 完成上一动作时的pose，用于hover
  int action_id_ = 0;
};