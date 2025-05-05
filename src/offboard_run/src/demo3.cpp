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
// 新增头文件
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// === 全局变量 ===
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
// 新增全局变量
bool is_armed = false;
geometry_msgs::TransformStamped body_to_enu_transform;

// 初始化 body_to_enu_transform
void initialize_body_to_enu_transform()
{
  body_to_enu_transform.header.frame_id = "enu";
  body_to_enu_transform.child_frame_id = "body";
  // 初始化四元数为单位四元数
  body_to_enu_transform.transform.rotation.w = 1.0;
}

// 前置声明
tf2_ros::TransformBroadcaster *tf_broadcaster;
tf2_ros::Buffer *tf_buffer;
tf2_ros::TransformListener *tf_listener;

// === 回调函数 ===
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
  if (!is_armed && msg->armed)
  {
    // 首次解锁时记录机体坐标系到 ENU 坐标系的变换
    is_armed = true;
    body_to_enu_transform.header.stamp = ros::Time::now();
    body_to_enu_transform.header.frame_id = "enu";
    body_to_enu_transform.child_frame_id = "body";
    body_to_enu_transform.transform.translation.x = current_pose.pose.position.x;
    body_to_enu_transform.transform.translation.y = current_pose.pose.position.y;
    body_to_enu_transform.transform.translation.z = current_pose.pose.position.z;
    body_to_enu_transform.transform.rotation = current_pose.pose.orientation;
  }
}

std::vector<Eigen::Quaterniond> init_q;
bool need_q = true;
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  current_pose.pose = msg->pose.pose;
  if (!need_q)
  {
    return;
  }
  Eigen::Quaterniond q = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  init_q.push_back(q);
}

// === 判断两个位置是否接近 ===
bool is_close(const geometry_msgs::PoseStamped &p1,
              const geometry_msgs::PoseStamped &p2, double tol)
{
  double dx = p1.pose.position.x - p2.pose.position.x;
  double dy = p1.pose.position.y - p2.pose.position.y;
  double dz = p1.pose.position.z - p2.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) < tol;
}

// === 创建目标点序列 ===
std::vector<geometry_msgs::PoseStamped> create_target_positions()
{
  std::vector<geometry_msgs::PoseStamped> targets;
  geometry_msgs::PoseStamped pose;
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

double get_avg_yaw(const std::vector<Eigen::Quaterniond> &quats)
{
  if (quats.empty())
  {
    ROS_WARN("Quaternion vector is empty. Returning 0 yaw.");
    return 0.0;
  }

  double sin_sum = 0.0;
  double cos_sum = 0.0;

  for (const auto &q : quats)
  {
    Eigen::Vector3d euler =
        q.toRotationMatrix().eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
    double yaw = euler[2];

    sin_sum += std::sin(yaw);
    cos_sum += std::cos(yaw);
  }

  return std::atan2(sin_sum, cos_sum); // Proper average of angles
}

// 修改坐标转换函数，使用 TF 进行转换
std::vector<geometry_msgs::PoseStamped>
p_body_to_ENU(const std::vector<geometry_msgs::PoseStamped> &target_points)
{
  std::vector<geometry_msgs::PoseStamped> transformed_points;
  for (const auto &target : target_points)
  {
    geometry_msgs::PoseStamped target_enu;
    try
    {
      // 修改为使用 -> 操作符
      target_enu = tf_buffer->transform(target, "enu", ros::Duration(1.0));
      // 输出转换后的 ENU 坐标系参数
      ROS_INFO("Transformed target in ENU coordinates: x = %f, y = %f, z = %f",
               target_enu.pose.position.x,
               target_enu.pose.position.y,
               target_enu.pose.position.z);
      transformed_points.push_back(target_enu);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Transform failed: %s", ex.what());
      continue;
    }
  }
  return transformed_points;
}

// === 主函数 ===
int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;

  // 初始化 body_to_enu_transform
  initialize_body_to_enu_transform();

  // 在 ros::init() 之后创建需要 ROS 环境的对象
  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);
  tf_broadcaster = new tf2_ros::TransformBroadcaster();

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
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // 初始化目标路径
  std::vector<geometry_msgs::PoseStamped> target_positions =
      create_target_positions();

  ros::Rate rate_high(100);
  while (ros::ok() && init_q.size() < 10)
  {
    ros::spinOnce();
    rate_high.sleep();
  }
  // 构造一个绕 Z 轴旋转的单位四元数（如 0 度）
  need_q = false;
  Eigen::Quaterniond q_from_body_to_enu(
      Eigen::AngleAxisd(get_avg_yaw(init_q), Eigen::Vector3d::UnitZ()));

  std::vector<geometry_msgs::PoseStamped> transformed_target_positions;

  // 发送一段时间目标位置以便进入 OFFBOARD
  for (int i = 0; ros::ok() && i < 100; ++i)
  {
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

  while (ros::ok())
  {
    tf2::Quaternion q;
    // 在进入循环后输出所需信息
    tf2::fromMsg(body_to_enu_transform.transform.rotation, q);
    ROS_INFO("body_to_enu_transform.header.frame_id: %s", body_to_enu_transform.header.frame_id.c_str());
    ROS_INFO("body_to_enu_transform.child_frame_id: %s", body_to_enu_transform.child_frame_id.c_str());
    ROS_INFO("Quaternion q: x = %f, y = %f, z = %f, w = %f", q.x(), q.y(), q.z(), q.w());
    ROS_INFO("Quaternion q length: %f", q.length());
    ROS_INFO("transformed_target_positions.size(): %zu", transformed_target_positions.size());

    // 发布 TF 变换
    if (is_armed)
    {
      body_to_enu_transform.header.stamp = ros::Time::now();
      // 检查 frame_id 和 child_frame_id
      if (body_to_enu_transform.header.frame_id.empty() || body_to_enu_transform.child_frame_id.empty())
      {
        ROS_WARN("frame_id 或 child_frame_id 未设置，跳过变换发布。");
      }
      else
      {
        // 检查四元数有效性
        tf2::Quaternion q;
        tf2::fromMsg(body_to_enu_transform.transform.rotation, q);
        if (q.length() < 1e-6)
        {
          ROS_WARN("四元数无效，跳过变换发布。");
        }
        else
        {
          q.normalize();
          body_to_enu_transform.transform.rotation = tf2::toMsg(q);
          tf_broadcaster->sendTransform(body_to_enu_transform);
        }
      }
    }

    // 设置模式
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard mode enabled");
      }
      last_request = ros::Time::now();
    }

    // 解锁
    if (!current_state.armed &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (arming_client.call(arm_cmd) && arm_cmd.response.success)
      {
        ROS_INFO("Vehicle armed");
        // 解锁后进行坐标转换
        transformed_target_positions = p_body_to_ENU(target_positions);
        // 输出转换后的目标点数量
        ROS_INFO("Number of transformed target positions: %zu", transformed_target_positions.size());
      }
      last_request = ros::Time::now();
    }

    // 控制无人机移动到目标
    if (target_index < transformed_target_positions.size())
    {
      const auto &target = transformed_target_positions[target_index];
      if (is_close(current_pose, target, 0.2))
      {
        ++target_index;
        ROS_INFO("Reached target %lu", target_index);
      }
      else
      {
        // 发布目标位置
        local_pos_pub.publish(target);
      }
    }
    else
    {
      local_pos_pub.publish(current_pose);
      ROS_INFO("Current pose published %lu", target_index);
      ROS_INFO("Current pose: x = %.2f, y = %.2f, z = %.2f",
               current_pose.pose.position.x,
               current_pose.pose.position.y,
               current_pose.pose.position.z);
    }

    ros::spinOnce();
    rate.sleep();
  }

  // 释放动态分配的内存
  delete tf_broadcaster;
  delete tf_listener;
  delete tf_buffer;

  return 0;
}