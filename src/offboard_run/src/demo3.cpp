#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "offboard_run/SlidingWindowAverage.h"
#include "ros/console.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/time.h"

class mavros_ctrl {
public:
  mavros_ctrl(ros::NodeHandle &nh)
      : tf_buffer_(), tf_listener_(tf_buffer_, nh), static_tf_broadcaster_() {
    // 初始化发布器和订阅器
    state_sub_ = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, // 增加队列大小
        std::bind(&mavros_ctrl::state_cb, this, std::placeholders::_1));
    px4_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 10, // 增加队列大小
        std::bind(&mavros_ctrl::px4_pose_cb, this, std::placeholders::_1));
    global_px4_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    arming_client_ =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ =
        nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    target_pos_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/target_position", 10, // 增加队列大小
        std::bind(&mavros_ctrl::target_pos_cb, this, std::placeholders::_1));
    current_world_body_pos_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/current_world_body_pos", 10);

    // 初始化默认姿态
    default_pose_.pose.position.x = 0;
    default_pose_.pose.position.y = 0;
    default_pose_.pose.position.z = 1.2;
    default_pose_.pose.orientation.w = 1.0; // 单位四元数

    // 设置加速度
  }

  // 初始化函数 - 发送一些位置设定点并等待连接
  bool initialize() {
    ros::Rate rate(50.0); // 增加频率到50Hz

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
    for (int i = 300; ros::ok() && i > 0; --i) {
      if (received_pose_) {
        global_px4_pos_pub_.publish(current_pose_);
      } else {
        global_px4_pos_pub_.publish(default_pose_);
      }
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Initial setpoints sent!");

    return true;
  }

  // 主循环函数 - 按照官方例程的流程
  void spin() {
    // 确保始终发布位置设定点
    if (received_pose_) {
      geometry_msgs::PoseStamped pub_pose = current_pose_;
      pub_pose.header.stamp = ros::Time::now();
      global_px4_pos_pub_.publish(pub_pose);
    } else {
      global_px4_pos_pub_.publish(default_pose_);
    }

    // 按照官方顺序: 先切换模式，成功后再尝试解锁
    if (current_state_.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
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
               (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
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

    // 只有在解锁成功后，才执行导航和坐标变换
    if (current_state_.armed && tf_ready_ && target_is_transformed_) {
      // 发布坐标变换
      static_tf_broadcaster_.sendTransform(world_enu_to_world_body_);

      // 发布转换后的目标位置
      global_px4_pos_pub_.publish(transformed_target_);

      // 处理位姿更新和导航逻辑
      updatePoseAndNavigation();
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

    ROS_INFO("Initialized body to world_enu transform");
    // tf_ready_ = true;
  }

  // 更新位姿和导航
  void updatePoseAndNavigation() {
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
    geometry_msgs::PoseStamped target_positions = *msg;

    if (target_positions.header.frame_id.empty()) {
      target_positions.header.frame_id = "world_body"; // 设置默认坐标系
    }

    if (!tf_ready_) {
      ROS_WARN("TF not ready, cannot transform target position");
      target_is_transformed_ = false;
      return;
    }

    // 进行坐标转换
    try {
      transformed_target_ = tf_buffer_.transform(target_positions, "world_enu",
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
    received_pose_ = true;
    // 更新滑动窗口平均值
    slid_window_avg.addPose(current_pose_);
  }

  // 成员变量
  ros::Publisher global_px4_pos_pub_;
  ros::Publisher current_world_body_pos_pub_;
  ros::Subscriber target_pos_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber px4_pose_sub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::PoseStamped transformed_target_;
  geometry_msgs::PoseStamped initial_pose_;
  geometry_msgs::PoseStamped default_pose_; // 默认姿态
  geometry_msgs::TransformStamped world_enu_to_world_body_;

  // 状态标志
  bool received_pose_ = false;
  bool tf_ready_ = false;
  bool target_is_transformed_ = false;

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

  // 主循环 - 增加频率到50Hz
  ros::Rate rate(50.0);

  while (ros::ok()) {
    mavros_ctrl.spin();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}