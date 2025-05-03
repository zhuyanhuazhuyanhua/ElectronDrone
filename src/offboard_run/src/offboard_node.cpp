#include "offboard_run/SlidingWindowAverage.h"
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <vector>

// === 全局变量 ===
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

// === 回调函数 ===
void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }

std::vector<Eigen::Quaterniond> init_q;
bool need_q = true;
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  current_pose.pose = msg->pose.pose;
  if (!need_q) {
    return;
  }
  Eigen::Quaterniond q = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  init_q.push_back(q);
}

// === 判断两个位置是否接近 ===
bool is_close(const geometry_msgs::PoseStamped &p1,
              const geometry_msgs::PoseStamped &p2, double tol) {
  double dx = p1.pose.position.x - p2.pose.position.x;
  double dy = p1.pose.position.y - p2.pose.position.y;
  double dz = p1.pose.position.z - p2.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) < tol;
}

// === 创建目标点序列 ===
std::vector<geometry_msgs::PoseStamped> create_target_positions() {
  std::vector<geometry_msgs::PoseStamped> targets;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;

  pose.pose.position.z = 1;
  targets.push_back(pose);
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.5;
  targets.push_back(pose);

  return targets;
}

Eigen::Quaterniond get_avg_q(const std::vector<Eigen::Quaterniond> &quats) {
  Eigen::Quaterniond avg_q;
  avg_q.setIdentity();
  for (const auto &q : init_q) {
    avg_q.coeffs() += q.coeffs();
  }
  avg_q.normalize();

  Eigen::Quaterniond q_body_to_enu = avg_q.conjugate();
  return q_body_to_enu;
}

std::vector<geometry_msgs::PoseStamped>
p_body_to_ENU(const std::vector<geometry_msgs::PoseStamped> &target_points,
              const Eigen::Quaterniond &q) {
  std::vector<geometry_msgs::PoseStamped> transformed_points;
  for (auto target : target_points) {
    Eigen::Vector3d p_body(target.pose.position.x, target.pose.position.y,
                           target.pose.position.z);
    Eigen::Vector3d p_enu = q * p_body;
    target.pose.position.x = p_enu.x();
    target.pose.position.y = p_enu.y();
    target.pose.position.z = p_enu.z();
    transformed_points.push_back(target);
  }
  return transformed_points;
}

// === 主函数 ===
int main(int argc, char **argv) {
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);
  ros::Subscriber odom_sub =
      nh.subscribe("mavros/local_position/odom", 10, odom_cb);

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  ros::Rate rate(20.0);

  // 等待连接
  while (ros::ok() && !current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  // 初始化目标路径
  std::vector<geometry_msgs::PoseStamped> target_positions =
      create_target_positions();

  ros::Rate rate_high(100);
  while (ros::ok() && init_q.size() < 10) {
    ros::spinOnce();
    rate_high.sleep();
  }
  // 构造一个绕 Z 轴旋转的单位四元数
  need_q = false;
  // Eigen::Quaterniond q_enu_to_body(
  // Eigen::AngleAxisd(get_avg_yaw(init_q), Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q_from_body_to_enu = get_avg_q(init_q);

  // Eigen::Quaterniond q_from_body_to_enu = q_enu_to_body.conjugate(); //
  // 反转方向

  std::vector<geometry_msgs::PoseStamped> transformed_target_positions =
      p_body_to_ENU(target_positions, q_from_body_to_enu);

  for (const auto &point : transformed_target_positions) {
    ROS_INFO("Transformed target position: x: %f, y: %f, z: %f",
             point.pose.position.x, point.pose.position.y,
             point.pose.position.z);
  }
  // 发送一段时间目标位置以便进入 OFFBOARD
  for (int i = 0; ros::ok() && i < 100; ++i) {
    local_pos_pub.publish(target_positions.front());
    ros::spinOnce();
    rate.sleep();
  }

  // 请求模式与解锁
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();
  size_t target_index = 0;

  while (ros::ok()) {
    // 设置模式
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard mode enabled");
      }
      last_request = ros::Time::now();
    }

    // 解锁
    if (!current_state.armed &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
      }
      last_request = ros::Time::now();
    }

    // 控制无人机移动到目标
    if (target_index < transformed_target_positions.size()) {
      const auto &target = transformed_target_positions[target_index];
      if (is_close(current_pose, target, 0.2)) {
        ++target_index;
        ROS_INFO("Reached target %lu", target_index);
      } else {
        // 发布目标位置
        local_pos_pub.publish(target);
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
