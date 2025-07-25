#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>  // 新头文件
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <cmath>

#include "offboard_run/SlidingWindowAverage.h"
#include "ros/console.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "ros/time.h"

class mavros_ctrl {
 public:
  mavros_ctrl(ros::NodeHandle &nh)
      : tf_buffer_(), tf_listener_(tf_buffer_, nh), static_tf_broadcaster_() {
    // 初始化发布器和订阅器
    state_sub_ = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10,
        std::bind(&mavros_ctrl::state_cb, this, std::placeholders::_1));
    px4_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10,
        std::bind(&mavros_ctrl::px4_pose_cb, this, std::placeholders::_1));

    // 修改为使用setpoint_raw/local
    raw_pos_pub_ = nh.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);

    arming_client_ =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    land_client_ =
        nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    target_pos_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/target_position", 10,
        std::bind(&mavros_ctrl::target_pos_cb, this, std::placeholders::_1));
    current_world_body_pos_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/current_world_body_pos", 10);

    // 接受摄像头的校正
    cam_aim_sub_ = nh.subscribe<geometry_msgs::Point>(
        "/camera_aiming_center", 10,
        std::bind(&mavros_ctrl::cam_aim_cb, this, std::placeholders::_1));

    nh.getParam("use_camera_aim", use_camera_aim_);
    ROS_INFO("Use camera aiming: %s", use_camera_aim_ ? "true" : "false");

    // 初始化默认位置目标
    initializePositionTarget(4, false, Eigen::Vector3d(0, 0, 1.2),
                             Eigen::Vector3d(0, 0, 0),
                             Eigen::Vector3d(0.0, 0.0, 6.0));
  }

  // 初始化默认位置目标
  /**
   * 初始化位置目标，支持不同的控制组合
   *
   * @param control_mode 控制模式:
   *     0: 仅位置控制 (x, y, z)
   *     1: 仅速度控制 (vx, vy, vz)
   *     2: 仅加速度控制 (afx, afy, afz)
   *     3: 位置+速度控制
   *     4: 位置+速度+加速度控制
   * @param use_body_frame 是否使用机体坐标系 (true: BODY_NED, false: LOCAL_NED)
   * @param position 位置设定点 (x, y, z)
   * @param velocity 速度设定点 (vx, vy, vz)
   * @param acceleration 加速度设定点 (afx, afy, afz)
   * @param yaw 偏航角设定值 (弧度)
   * @param yaw_rate 偏航角速率 (弧度/秒)
   */
  void initializePositionTarget(
      int control_mode = 0, bool use_body_frame = false,
      const Eigen::Vector3d &position = Eigen::Vector3d(0, 0, 1.2),
      const Eigen::Vector3d &velocity = Eigen::Vector3d(0, 0, 0),
      const Eigen::Vector3d &acceleration = Eigen::Vector3d(0, 0, 0),
      double yaw = 0.0, double yaw_rate = 0.0) {
    mavros_msgs::PositionTarget pos_target;

    // 设置头信息和坐标系
    pos_target.header.frame_id = "world_body";
    pos_target.header.stamp = ros::Time::now();

    // 设置坐标系
    if (use_body_frame) {
      pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    } else {
      pos_target.coordinate_frame =
          mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    }

    // 默认忽略所有字段
    uint16_t type_mask = 0;

    // 根据控制模式设置type_mask和对应的值
    switch (control_mode) {
      case 0:  // 仅位置控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      case 1:  // 仅速度控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      case 2:  // 仅加速度控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      case 3:  // 位置+速度控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      case 4:  // 位置+速度+加速度控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      default:
        ROS_WARN("Unknown control mode: %d, using position control",
                 control_mode);
        type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    }

    // 根据yaw_rate是否为0决定是否使用偏航角速率控制
    if (yaw_rate != 0.0) {
      type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
      type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
    }

    pos_target.type_mask = type_mask;

    // 设置位置、速度、加速度和姿态值
    pos_target.position.x = position.x();
    pos_target.position.y = position.y();
    pos_target.position.z = position.z();

    pos_target.velocity.x = velocity.x();
    pos_target.velocity.y = velocity.y();
    pos_target.velocity.z = velocity.z();

    pos_target.acceleration_or_force.x = acceleration.x();
    pos_target.acceleration_or_force.y = acceleration.y();
    pos_target.acceleration_or_force.z = acceleration.z();

    pos_target.yaw = yaw;
    pos_target.yaw_rate = yaw_rate;

    // 存储结果
    default_pos_target_ = pos_target;

    // 输出调试信息
    ROS_INFO("Initialized position target:");
    ROS_INFO("  Control mode: %d", control_mode);
    ROS_INFO("  Coordinate frame: %s",
             use_body_frame ? "BODY_NED" : "LOCAL_NED");
    ROS_INFO("  Position: [%.2f, %.2f, %.2f]", position.x(), position.y(),
             position.z());
    ROS_INFO("  Velocity: [%.2f, %.2f, %.2f]", velocity.x(), velocity.y(),
             velocity.z());
    ROS_INFO("  Acceleration: [%.2f, %.2f, %.2f]", acceleration.x(),
             acceleration.y(), acceleration.z());
    ROS_INFO("  Yaw: %.2f rad, Yaw rate: %.2f rad/s", yaw, yaw_rate);
    ROS_INFO("  Type mask: %d", type_mask);
  }

  /**
   * 从PoseStamped转换为PositionTarget，并支持添加速度和加速度信息
   *
   * @param pose 位置姿态信息
   * @param control_mode 控制模式:
   *     0: 仅位置控制 (x, y, z)
   *     1: 仅速度控制 (vx, vy, vz)
   *     2: 仅加速度控制 (afx, afy, afz)
   *     3: 位置+速度控制
   *     4: 位置+速度+加速度控制
   * @param use_body_frame 是否使用机体坐标系
   * @param velocity 速度设定点 (vx, vy, vz)
   * @param acceleration 加速度设定点 (afx, afy, afz)
   * @param yaw 覆盖姿态中的偏航角 (如果不为NaN)
   * @param yaw_rate 偏航角速率 (弧度/秒)
   * @return mavros_msgs::PositionTarget 设定点消息
   */
  mavros_msgs::PositionTarget poseToPositionTarget(
      const geometry_msgs::PoseStamped &pose, int control_mode = 4,
      bool use_body_frame = false,
      const Eigen::Vector3d &velocity = Eigen::Vector3d(0, 0, 0),
      const Eigen::Vector3d &acceleration = Eigen::Vector3d(0.0, 0.0, 6.0),
      double yaw = NAN, double yaw_rate = 0.0) {
    mavros_msgs::PositionTarget pos_target;

    // 设置头信息
    pos_target.header = pose.header;
    pos_target.header.stamp = ros::Time::now();

    // 设置坐标系
    if (use_body_frame) {
      pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    } else {
      pos_target.coordinate_frame =
          mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    }

    // 设置位置信息
    pos_target.position.x = pose.pose.position.x;
    pos_target.position.y = pose.pose.position.y;
    pos_target.position.z = pose.pose.position.z;

    // 设置速度信息
    pos_target.velocity.x = velocity.x();
    pos_target.velocity.y = velocity.y();
    pos_target.velocity.z = velocity.z();

    // 设置加速度信息
    pos_target.acceleration_or_force.x = acceleration.x();
    pos_target.acceleration_or_force.y = acceleration.y();
    pos_target.acceleration_or_force.z = acceleration.z();

    // 从四元数提取偏航角（如果未提供yaw参数）
    double extracted_yaw = 0.0;
    if (std::isnan(yaw)) {
      tf2::Quaternion q;
      tf2::fromMsg(pose.pose.orientation, q);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, extracted_yaw);
      pos_target.yaw = extracted_yaw;
    } else {
      pos_target.yaw = yaw;
    }

    // 设置偏航角速率
    pos_target.yaw_rate = yaw_rate;

    // 根据控制模式设置type_mask
    uint16_t type_mask = 0;

    switch (control_mode) {
      case 0:  // 仅位置控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      case 1:  // 仅速度控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      case 2:  // 仅加速度控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      case 3:  // 位置+速度控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      case 4:  // 位置+速度+加速度控制
        type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        break;

      default:
        ROS_WARN("Unknown control mode: %d, using position control",
                 control_mode);
        type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    }

    // 根据yaw_rate是否为0决定是否使用偏航角速率控制
    if (yaw_rate != 0.0) {
      type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
      type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
    }

    pos_target.type_mask = type_mask;

    return pos_target;
  }

  // 初始化函数 - 发送一些位置设定点并等待连接
  bool initialize() {
    ros::Rate rate(50.0);  // 增加频率到50Hz

    // 等待连接
    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state_.connected) {
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("FCU connected!");

    // 等待收到位姿数据
    ROS_INFO("Waiting for position data...");
    while (ros::ok() && !received_pose_) {
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Position data received!");

    // 发送一些位置设定点
    ROS_INFO("Sending initial setpoints...");
    for (int i = 50; ros::ok() && i > 0; --i) {
      if (received_pose_) {
        mavros_msgs::PositionTarget pos_target =
            poseToPositionTarget(current_pose_);
        raw_pos_pub_.publish(pos_target);
      } else {
        default_pos_target_.header.stamp = ros::Time::now();
        raw_pos_pub_.publish(default_pos_target_);
      }
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Initial setpoints sent!");

    return true;
  }

  // 主循环函数 - 按照官方例程的流程
  void spin() {
    // 检查是否降落
    if (is_grounded_ == true) {
      ROS_INFO("Vehicle is grounded. Task Finishi!");
      return;
    }

    // 按照官方顺序: 先切换模式，成功后再尝试解锁
    if (current_state_.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request_ > ros::Duration(1.0))) {
      // 尝试切换到OFFBOARD模式
      mavros_msgs::SetMode offb_set_mode;
      offb_set_mode.request.custom_mode = "OFFBOARD";

      if (set_mode_client_.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      } else {
        ROS_ERROR("Failed to set Offboard mode");
      }
      last_request_ = ros::Time::now();
    } else if (!current_state_.armed &&
               (ros::Time::now() - last_request_ > ros::Duration(1.0))) {
      // 只有在OFFBOARD模式时才尝试解锁
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;

      if (arming_client_.call(arm_cmd)) {
        if (arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");

          // 首次解锁成功后初始化坐标变换
          if (!tf_ready_) {
            initializeTransform();
          }
        } else {
          ROS_ERROR("Arming failed: %s",
                    arm_cmd.response.success ? "true" : "false");
        }
      } else {
        ROS_ERROR("Arming service call failed");
      }
      last_request_ = ros::Time::now();
    }

    // 始终发布静态变换（如果已初始化）
    if (tf_ready_) {
      geometry_msgs::PoseStamped current_world_body;
      current_world_body.header.frame_id = "world_body";
      try {
        current_world_body = tf_buffer_.transform(current_pose_, "world_body",
                                                  ros::Duration(0.5));
        current_world_body_pos_pub_.publish(current_world_body);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform local pos failed: %s", ex.what());
      }
    }

    // ==================== SETPOINT LOGIC ====================
    // 只发布一次setpoint，按照优先级处理

    mavros_msgs::PositionTarget setpoint;
    bool setpoint_ready = false;

    // 检查aim数据有效性
    if (ros::Time::now() - last_cam_aim_receive_time_ >
        ros::Duration(CAM_AIMING_TIMEOUT)) {
      is_aiming_ = false;  // 超时后退出AIMING模式
      ROS_WARN("Camera aiming timeout, exiting AIMING mode.");
    }

    // 优先级1: 如果有有效的目标位置，使用它
    if (current_state_.armed && tf_ready_ && target_is_transformed_) {
      if (!is_aiming_) {
        // 使用转换后的目标位置
        setpoint = poseToPositionTarget(transformed_target_, 4, false,
                                        Eigen::Vector3d(0, 0, 0),
                                        Eigen::Vector3d(0, 0, 2));

        // 打印详细的目标信息
        ROS_INFO("--------- TARGET COMMAND ---------");
        ROS_INFO("Target Received: [%.3f, %.3f, %.3f]",
                 target_positions_.pose.position.x,
                 target_positions_.pose.position.y,
                 target_positions_.pose.position.z);
        ROS_INFO("Frame: %s (coordinate_frame=%d)",
                 transformed_target_.header.frame_id.c_str(),
                 setpoint.coordinate_frame);
        ROS_INFO("Position: [%.3f, %.3f, %.3f]", setpoint.position.x,
                 setpoint.position.y, setpoint.position.z);
        ROS_INFO("Velocity: [%.3f, %.3f, %.3f]", setpoint.velocity.x,
                 setpoint.velocity.y, setpoint.velocity.z);
        ROS_INFO("Acceleration: [%.3f, %.3f, %.3f]",
                 setpoint.acceleration_or_force.x,
                 setpoint.acceleration_or_force.y,
                 setpoint.acceleration_or_force.z);
        ROS_INFO("Yaw: %.3f, Yaw rate: %.3f", setpoint.yaw, setpoint.yaw_rate);
        ROS_INFO("Type mask: %d", setpoint.type_mask);
        ROS_INFO("Current position : [%.3f, %.3f, %.3f]",
                 current_pose_.pose.position.x, current_pose_.pose.position.y,
                 current_pose_.pose.position.z);
        ROS_INFO("---------------------------------");

        setpoint_ready = true;

        // 处理位姿更新和导航
      } else {
        // 如果处于AIMING模式，使用摄像头校正的聚焦中心
        static constexpr double p = 10;
        setpoint = poseToPositionTarget(
            geometry_msgs::PoseStamped(), 4, true,
            Eigen::Vector3d(aiming_center_.x / p, aiming_center_.y / p, 0),
            Eigen::Vector3d(0, 0, 0), 0.0, 0.0);

        ROS_INFO("--------- AIMING MODE ---------");
        ROS_INFO("Aiming Center: [%.3f, %.3f]", aiming_center_.x,
                 aiming_center_.y);
        ROS_INFO("Set Velocity: [%.3f, %.3f, %.3f]", setpoint.velocity.x,
                 setpoint.velocity.y, setpoint.velocity.z);
        ROS_INFO("--------------------------------");
        setpoint_ready = true;
      }
    }
    // 优先级2: 如果没有有效目标，但有当前位置，使用当前位置
    else if (received_pose_) {
      setpoint = poseToPositionTarget(current_pose_);

      ROS_INFO("--------- HOLDING POSITION ---------");
      ROS_INFO("Position: [%.3f, %.3f, %.3f]", setpoint.position.x,
               setpoint.position.y, setpoint.position.z);
      ROS_INFO("------------------------------------");

      setpoint_ready = true;
    }
    // 优先级3: 如果以上都不可用，使用默认位置目标
    else {
      setpoint = default_pos_target_;

      ROS_INFO("--------- DEFAULT POSITION ---------");
      ROS_INFO("Position: [%.3f, %.3f, %.3f]", setpoint.position.x,
               setpoint.position.y, setpoint.position.z);
      ROS_INFO("------------------------------------");

      setpoint_ready = true;
    }

    if (tf_ready_) {
      // 更新位姿和导航
      pubCurrentWorldPose();
    }

    // 发布准备好的setpoint（只发布一次）
    if (setpoint_ready) {
      setpoint.header.stamp = ros::Time::now();
      raw_pos_pub_.publish(setpoint);
    }
  }

 private:
  // 初始化坐标变换
  void initializeTransform() {
    // 记录初始位置，用于后续计算
    geometry_msgs::PoseStamped start_local_body =
        slid_window_avg.computeAveragePose();
    initial_pose_ = start_local_body;

    // 创建 world_enu -> world_body 的静态变换
    world_enu_to_world_body_.header.stamp = ros::Time::now();
    world_enu_to_world_body_.header.frame_id = "world_enu";
    world_enu_to_world_body_.child_frame_id = "world_body";

    // 设置变换参数
    world_enu_to_world_body_.transform.translation.x =
        start_local_body.pose.position.x;
    world_enu_to_world_body_.transform.translation.y =
        start_local_body.pose.position.y;
    world_enu_to_world_body_.transform.translation.z =
        start_local_body.pose.position.z;
    world_enu_to_world_body_.transform.rotation =
        start_local_body.pose.orientation;
    static_tf_broadcaster_.sendTransform(world_enu_to_world_body_);

    ROS_INFO("Initialized body to world_enu transform");
    tf_ready_ = true;
  }

  // pub 当前位姿到 world_body 坐标系
  void pubCurrentWorldPose() {
    if (tf_ready_) {
      geometry_msgs::PoseStamped current_world_body;
      try {
        current_world_body = tf_buffer_.transform(current_pose_, "world_body",
                                                  ros::Duration(0.5));
        current_world_body_pos_pub_.publish(current_world_body);
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform local pos failed: %s", ex.what());
      }
    }
  }

  // 处理目标位置回调
  void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    target_positions_ = *msg;

    target_positions_.header.frame_id = "world_body";  // 设置坐标系

    // 判断 z 坐标是否为 -1.0
    if (target_positions_.pose.position.z > -1.1 &&
        target_positions_.pose.position.z < -0.9) {
      // 尝试上锁

      // 设置飞行模式为自动降落模式
      mavros_msgs::SetMode set_mode;
      set_mode.request.custom_mode = "AUTO.LAND";  // 设置为降落模式
      if (set_mode_client_.call(set_mode)) {
        ROS_INFO("成功设置自动降落模式");
      } else {
        ROS_ERROR("设置自动降落模式失败");
      }

      // 发送降落指令
      mavros_msgs::CommandTOL land_cmd;
      land_cmd.request.altitude = 0;  // 降落到地面
      if (land_client_.call(land_cmd)) {
        ROS_INFO("开始降落...");
      } else {
        ROS_ERROR("降落命令失败");
      }

      // 等待无人机降落完成
      ros::Duration(3.0).sleep();  // 等待一些时间，确保无人机已降落

      // 停桨（解除电机控制）
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = false;  // 停用电机
      if (arming_client_.call(arm_cmd)) {
        ROS_INFO("电机已停");
      } else {
        ROS_ERROR("停桨命令失败");
      }
    } else if (use_camera_aim_ && target_positions_.pose.position.z > -2.1 &&
               target_positions_.pose.position.z < -1.9 &&
               ros::Time::now() - last_cam_aim_receive_time_ <
                   ros::Duration(CAM_AIMING_TIMEOUT)) {
      // 进入AIMING模式
      is_aiming_ = true;
    }

    if (!tf_ready_) {
      ROS_WARN("TF not ready, cannot transform target position");
      target_is_transformed_ = false;
      return;
    }

    // 进行坐标转换
    try {
      transformed_target_ = tf_buffer_.transform(target_positions_, "world_enu",
                                                 ros::Duration(0.5));
      target_is_transformed_ = true;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Transform failed: %s", ex.what());
      target_is_transformed_ = false;
    }
  }

  // 处理状态回调
  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
    // 更新心跳时间
    last_heartbeat_ = ros::Time::now();
  }

  // 处理位姿回调
  void px4_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose_ = *msg;
    current_pose_.header.frame_id = "world_enu";
    received_pose_ = true;
    // 更新滑动窗口平均值
    slid_window_avg.addPose(current_pose_);
  }

  void cam_aim_cb(const geometry_msgs::Point::ConstPtr &msg) {
    aiming_center_ = *msg;
    last_cam_aim_receive_time_ = ros::Time::now();
    ROS_INFO("Received camera aiming center: [%.3f, %.3f]", aiming_center_.x,
             aiming_center_.y);
    static constexpr double close_thresh = 10;
    if (aiming_center_.x < close_thresh && aiming_center_.y < close_thresh) {
      aim_close_count_++;
      if (aim_close_count_ > 5) {
        is_aiming_ = false;  // 如果连续5次接近中心，则退出AIMING模式
        ROS_INFO("Exiting AIMING mode due to close center.");
      }
    } else {
      aim_close_count_ = 0;  // 重置计数器
    }
  }

  // 成员变量
  ros::Publisher raw_pos_pub_;  // 改为发布PositionTarget
  ros::Publisher current_world_body_pos_pub_;
  ros::Subscriber target_pos_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber px4_pose_sub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient land_client_;
  ros::ServiceClient set_mode_client_;
  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped transformed_target_;
  geometry_msgs::PoseStamped initial_pose_;
  mavros_msgs::PositionTarget default_pos_target_;  // 默认位置目标
  geometry_msgs::TransformStamped world_enu_to_world_body_;
  geometry_msgs::PoseStamped target_positions_;  // 目标位置
  int aim_close_count_ = 0;

  // 摄像头校正相关
  geometry_msgs::Point aiming_center_;  // 聚焦中心位置
  ros::Subscriber cam_aim_sub_;
  ros::Time last_cam_aim_receive_time_;  // 最后一次接收摄像头校正的时间
  bool use_camera_aim_ = false;          // 是否使用摄像头校正
  static constexpr double CAM_AIMING_TIMEOUT = 0.5;  // 摄像头校正超时时间
  bool is_aiming_ = false;  // 是否处于摄像头校正状态

  // 状态标志
  bool received_pose_ = false;
  bool tf_ready_ = false;
  bool target_is_transformed_ = false;
  bool is_grounded_ = false;

  // 时间管理
  ros::Time last_request_ = ros::Time::now();
  ros::Time last_heartbeat_ = ros::Time::now();
  ros::Duration heartbeat_timeout_ = ros::Duration(1.2);

  // TF相关
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  // 滑动窗口平均
  SlidingWindowPoseAverage slid_window_avg = SlidingWindowPoseAverage(10);
};

// === 主函数 ===
int main(int argc, char **argv) {
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;

  // 创建控制器实例
  mavros_ctrl mavros_ctrl(nh);

  // 初始化并发送预设点
  if (!mavros_ctrl.initialize()) {
    ROS_ERROR("Initialization failed! Exiting...");
    return 1;
  }

  // 主循环 - 频率保持在50Hz
  ros::Rate rate(50.0);

  while (ros::ok()) {
    mavros_ctrl.spin();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}