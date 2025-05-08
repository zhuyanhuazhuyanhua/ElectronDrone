#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
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
#include <functional>

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
        "mavros/state", 1,
        std::bind(&mavros_ctrl::state_cb, this, std::placeholders::_1));
    px4_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 1,
        std::bind(&mavros_ctrl::px4_pose_cb, this, std::placeholders::_1));
    global_px4_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 1);
    arming_client_ =
        nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ =
        nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    target_pos_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/target_position", 1,
        std::bind(&mavros_ctrl::target_pos_cb, this, std::placeholders::_1));
    current_world_body_pos_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/current_world_body_pos", 1);
  }
  /**
   * @brief 切换到 OFFBOARD 模式,主循环中务必监控心跳时间,超时后一定要调用本函数
   *
   * @param urgent 是否紧急切换,非紧急情况下会发布一段时间setpoints
   * @return true 切换成功
   * @return false 切换失败
   */
  bool switch_to_offboard(bool urgent = true) {
    if (!current_state_.connected) {
      ROS_WARN("Waiting for connection...");
      return false;
    } else if (!received_pose_) {
      ROS_WARN("Waiting for pose...");
      return false;
    } else if (current_state_.mode == "OFFBOARD") {
      ROS_INFO("Already in OFFBOARD mode");
      return true;
    }
    // 空中情况无时间发送大量点
    if (!urgent) {
      // 发布一段时间目标位置以便进入 OFFBOARD
      ros::Rate rate(20.0);
      for (int i = 0; ros::ok() && i < 100; ++i) {
        global_px4_pos_pub_.publish(current_pose_);
        ros::spinOnce();
        rate.sleep();
      }
    }

    if (ros::Time::now() - last_request_ < request_gap_) {
      ROS_INFO("Send request too frequently, please wait...");
    }
    if (current_state_.mode != "OFFBOARD") {
      mavros_msgs::SetMode offb_set_mode;
      offb_set_mode.request.custom_mode = "OFFBOARD";
      ROS_INFO("Switching to OFFBOARD mode...");
      if (set_mode_client_.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
        return true;
      } else {
        ROS_ERROR("Failed to set Offboard mode");
        return false;
      }
    }
    return true;
  }

  bool arm() {
    if (!current_state_.connected) {
      ROS_WARN("Waiting for connection...");
      return false;
    } else if (!received_pose_) {
      ROS_WARN("Waiting for pose...");
      return false;
    } else if (current_state_.armed) {
      ROS_INFO("Already armed");
      return true;
    }

    if (ros::Time::now() - last_request_ < request_gap_) {
      ROS_INFO("Send request too frequently, please wait...");
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client_.call(arm_cmd)) {
      if (arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
        last_request_ = ros::Time::now();
        return true;
      } else {
        ROS_ERROR("Failed to arm vehicle");
        return false;
      }
    } else {
      ROS_ERROR("Arming service call failed");
      return false;
    }
  }

  void spin() {
    if (!current_state_.connected) {
      ROS_INFO("Waiting for connection...");
      return;
    } else if (current_state_.mode != "OFFBOARD") {
      ROS_INFO("Not IN OFFBOARD!!!");
      // 尝试重置OFFBOARD模式
      if (!switch_to_offboard(true)) {
        // 如果OFFBOARD模式切换失败，返回
        return;
      }
    } else if (!received_pose_) {
      ROS_INFO("Waiting for pose...");
      return;
    }

    // 检查心跳超时
    if (ros::Time::now() - last_heartbeat_ > heartbeat_timeout_) {
      ROS_WARN("Heartbeat timeout. Checking Offboard mode...");
      if (current_state_.mode != "OFFBOARD") {
        ROS_INFO("Trying to re-enter Offboard mode...");
        if (!switch_to_offboard(true)) {
          // 如果OFFBOARD模式切换失败，返回
          return;
        }
      }
    }

    bool armed = current_state_.armed;
    // 检查是否arm,没有arm则尝试arm
    if (!armed) {
      ROS_INFO("Vehicle not armed. Trying to arm...");
      if (!arm()) {
        // 如果解锁失败，返回
        ROS_WARN("Failed to arm vehicle");
        return;
      } else {
        ROS_INFO("Vehicle armed");
        if (!tf_ready_) {
          // 首次解鎖
          // 记录初始位置，用于后续计算
          geometry_msgs::PoseStamped start_local_body =
              slid_window_avg.computeAveragePose();
          initial_pose_ = start_local_body;

          // 创建 world_enu -> world_body 的静态变换
          world_enu_to_world_body_.header.stamp = ros::Time::now();
          world_enu_to_world_body_.header.frame_id = "world_enu";
          world_enu_to_world_body_.child_frame_id = "world_body";

          // 设置变换参数（位置和姿态）
          world_enu_to_world_body_.transform.translation.x =
              start_local_body.pose.position.x;
          world_enu_to_world_body_.transform.translation.y =
              start_local_body.pose.position.y;
          world_enu_to_world_body_.transform.translation.z =
              start_local_body.pose.position.z;
          world_enu_to_world_body_.transform.rotation =
              start_local_body.pose.orientation;

          // 发布静态变换
          static_tf_broadcaster_.sendTransform(world_enu_to_world_body_);

          ROS_INFO("Initialized body to world_enu transform.");
          tf_ready_ = true;
        }
      }
    }

    if (tf_ready_ && target_is_transformed_) {
      static_tf_broadcaster_.sendTransform(world_enu_to_world_body_);
      global_px4_pos_pub_.publish(transformed_target_);
    } else {
      ROS_WARN(
          "TF not ready, cannot publish target position, publishing current "
          "pose");
      geometry_msgs::PoseStamped pub_pose = current_pose_;
      pub_pose.header.stamp = ros::Time::now();
      global_px4_pos_pub_.publish(pub_pose);
    }
  }

  geometry_msgs::PoseStamped get_current_pose() { return current_pose_; }

  // 计算当前位姿的平均值,滑动窗口
  SlidingWindowPoseAverage slid_window_avg = SlidingWindowPoseAverage(10);

 private:
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
  geometry_msgs::TransformStamped world_enu_to_world_body_;
  // 是否收到位姿
  bool received_pose_ = false;
  // 首次解锁时记录机体坐标系到 ENU 坐标系的变换
  bool tf_ready_ = false;
  // 目标变换成功
  bool target_is_transformed_ = false;
  // 上次发送请求时间
  ros::Time last_request_ = ros::Time::now();
  ros::Duration request_gap_ = ros::Duration(0.5);

  // 心跳超时时长设置
  ros::Duration heartbeat_timeout_ = ros::Duration(0.5);
  ros::Time last_heartbeat_ = ros::Time::now();

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // 处理目标位置数据
    geometry_msgs::PoseStamped target_positions = *msg;

    if (target_positions.header.frame_id.empty()) {
      target_positions.header.frame_id = "world_body";  // 设置默认坐标系
    }
    // 进行坐标转换
    try {
      transformed_target_ = tf_buffer_.transform(target_positions, "world_enu",
                                                 ros::Duration(0.5));
      target_is_transformed_ = true;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Transform failed: %s", ex.what());
      target_is_transformed_ = false;
      return;
    }
  }

  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
    // 更新心跳时间
    last_heartbeat_ = ros::Time::now();
  }

  void px4_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // 处理 px4 的位姿数据
    current_pose_ = *msg;

    received_pose_ = true;
    // 更新滑动窗口平均值
    slid_window_avg.addPose(current_pose_);

    if (tf_ready_) {
      geometry_msgs::PoseStamped current_world_body;
      try {
        current_world_body = tf_buffer_.transform(current_pose_, "world_body",
                                                  ros::Duration(0.5));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform local pos failed: %s", ex.what());
        return;
      }
      current_world_body_pos_pub_.publish(current_world_body);
    }
  }
};

// === 主函数 ===
int main(int argc, char **argv) {
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;

  mavros_ctrl mavros_ctrl(nh);

  // 切换到 OFFBOARD 模式
  mavros_ctrl.switch_to_offboard(false);

  // 控制setpoints频率
  ros::Rate rate(20.0);

  while (ros::ok()) {
    mavros_ctrl.spin();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}