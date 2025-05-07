#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <vector>

#include "offboard_run/SlidingWindowAverage.h"
#include "ros/console.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/time.h"

class mavros_ctrl {
 public:
  mavros_ctrl(ros::NodeHandle &nh) : tf_listener_(tf_buffer_, nh) {
    // 初始化发布器和订阅器
    state_sub_ = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 1,
        std::bind(&mavros_ctrl::state_cb, this, std::placeholders::_1));
    px4_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "mavros/local_position/pose", 1,
        std::bind(&mavros_ctrl::px4_pose_cb, this, std::placeholders::_1));
    target_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 1);
    arming_client_ =
        nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ =
        nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // TODO读取参数
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
        target_pos_pub_.publish(current_pose_);
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

    // mavros连接无误
    slid_window_avg.addPose(current_pose_);

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
          geometry_msgs::PoseStamped start_local_body =
              slid_window_avg.computeAveragePose();
          start_local_body.header.frame_id = "body";
          // 开始建立TF变换
          geometry_msgs::TransformStamped body_to_enu_transform;
          body_to_enu_transform.header.frame_id = "enu";
          body_to_enu_transform.child_frame_id = "body";
          body_to_enu_transform.header.stamp = ros::Time::now();

          // 设置原点
          body_to_enu_transform.transform.translation.x =
              start_local_body.pose.position.x;
          body_to_enu_transform.transform.translation.y =
              start_local_body.pose.position.y;
          body_to_enu_transform.transform.translation.z =
              start_local_body.pose.position.z;
          // 设置旋转
          body_to_enu_transform.transform.rotation =
              start_local_body.pose.orientation;

          // 发布变换
          tf_broadcaster_.sendTransform(body_to_enu_transform);
          ROS_INFO("Initialized body to world_enu transform.");
          tf_ready_ = true;
        }
      }
    }

    // TODO发布目标位置
    if (tf_ready_) {
    } else {
      ROS_WARN(
          "TF not ready, cannot publish target position, publishing current "
          "pose");
      geometry_msgs::PoseStamped pub_pose = current_pose_;
      pub_pose.header.stamp = ros::Time::now();
      target_pos_pub_.publish(current_pose_);
    }
  }

  geometry_msgs::PoseStamped get_current_pose() { return current_pose_; }

  // 计算当前位姿的平均值,滑动窗口
  SlidingWindowPoseAverage slid_window_avg = SlidingWindowPoseAverage(10);

 private:
  ros::Publisher target_pos_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber px4_pose_sub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  mavros_msgs::State current_state_;
  geometry_msgs::PoseStamped current_pose_;
  std::vector<geometry_msgs::PoseStamped> target_positions_;

  // 是否收到位姿
  bool received_pose_ = false;
  // 首次解锁时记录机体坐标系到 ENU 坐标系的变换
  bool tf_ready_ = false;

  // 上次发送请求时间
  ros::Time last_request_ = ros::Time::now();
  ros::Duration request_gap_ = ros::Duration(0.5);

  // 心跳超时时长设置
  ros::Duration heartbeat_timeout_ = ros::Duration(0.5);
  ros::Time last_heartbeat_;

  tf2_ros::TransformBroadcaster tf_broadcaster_tf2_ =
      tf2_ros::TransformBroadcaster();
  tf2_ros::Buffer tf_buffer_ = tf2_ros::Buffer();
  tf2_ros::TransformListener tf_listener_;

  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    received_pose_ = true;
    current_state_ = *msg;
    // 更新心跳时间
    last_heartbeat_ = ros::Time::now();
  }

  void px4_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // 处理 px4 的位姿数据
    current_pose_ = *msg;
  }

  bool is_close(const geometry_msgs::PoseStamped &p1,
                const geometry_msgs::PoseStamped &p2, double tol) {
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) < tol;
  }
};

// === 创建目标点序列 ===
std::vector<geometry_msgs::PoseStamped> create_target_positions() {
  std::vector<geometry_msgs::PoseStamped> targets;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "body";
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;

  pose.pose.position.z = 1.2;
  targets.push_back(pose);
  pose.pose.position.z = 0.8;
  targets.push_back(pose);
  pose.pose.position.z = 0.0;
  targets.push_back(pose);

  return targets;
}

double get_avg_yaw(const std::vector<Eigen::Quaterniond> &quats) {
  if (quats.empty()) {
    ROS_WARN("Quaternion vector is empty. Returning 0 yaw.");
    return 0.0;
  }

  double sin_sum = 0.0;
  double cos_sum = 0.0;

  for (const auto &q : quats) {
    Eigen::Vector3d euler =
        q.toRotationMatrix().eulerAngles(0, 1, 2);  // Roll, Pitch, Yaw
    double yaw = euler[2];

    sin_sum += std::sin(yaw);
    cos_sum += std::cos(yaw);
  }

  return std::atan2(sin_sum, cos_sum);  // Proper average of angles
}

// 修改坐标转换函数，使用 TF 进行转换
std::vector<geometry_msgs::PoseStamped> p_body_to_ENU(
    const std::vector<geometry_msgs::PoseStamped> &target_points,
    tf2_ros::Buffer *tf_buffer) {
  std::vector<geometry_msgs::PoseStamped> transformed_points;
  for (const auto &target : target_points) {
    geometry_msgs::PoseStamped target_enu;
    // 检查 source_frame 是否为空
    if (target.header.frame_id.empty()) {
      ROS_WARN("Source frame ID is empty. Skipping transformation.");
      continue;
    }
    // 更新时间戳为当前时间
    geometry_msgs::PoseStamped target_with_current_time = target;
    target_with_current_time.header.stamp = ros::Time::now();
    try {
      // 修改为使用 -> 操作符
      target_enu = tf_buffer->transform(target_with_current_time, "enu",
                                        ros::Duration(1.0));
      // 输出转换后的 ENU 坐标系参数
      ROS_INFO("Transformed target in ENU coordinates: x = %f, y = %f, z = %f",
               target_enu.pose.position.x, target_enu.pose.position.y,
               target_enu.pose.position.z);
      transformed_points.push_back(target_enu);
    } catch (tf2::TransformException &ex) {
      // 输出更详细的错误信息
      ROS_ERROR(
          "Transform failed for target point (x=%.2f, y=%.2f, z=%.2f): %s",
          target.pose.position.x, target.pose.position.y,
          target.pose.position.z, ex.what());
      continue;
    }
  }
  // 输出转换成功的目标点数量
  ROS_INFO("Number of successfully transformed target points: %zu",
           transformed_points.size());
  return transformed_points;
}

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