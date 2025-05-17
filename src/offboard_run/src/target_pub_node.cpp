#include "ros/console.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

// 全局变量，用于存储接收到的消息
geometry_msgs::PoseStamped current_target;
geometry_msgs::PoseStamped current_world_body;
bool current_pos_received = false;

// 判断两个位置是否接近
bool isClose(const geometry_msgs::PoseStamped &pose1,
             const geometry_msgs::PoseStamped &pose2, double threshold) {
  double distance =
      std::sqrt(std::pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                std::pow(pose1.pose.position.y - pose2.pose.position.y, 2) +
                std::pow(pose1.pose.position.z - pose2.pose.position.z, 2));
  return distance < threshold;
}

bool getTargetPoints(ros::NodeHandle &nh,
                     std::vector<geometry_msgs::PoseStamped> &targetpoints) {
  std::vector<float> target_list;

  if (nh.getParam("targetpoints", target_list)) {
    if (target_list.size() % 3 !=
        0) { // 每个目标点有3个元素 (x, y, z, ox, oy, oz, ow) 四元数弃置
      ROS_ERROR("Invalid targetpoints size. Each target point should have 3 "
                "elements (x, y, z).");
      return false;
    }

    for (size_t i = 0; i < target_list.size(); i += 3) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world_body";
      pose.pose.position.x = target_list[i];
      pose.pose.position.y = target_list[i + 1];
      pose.pose.position.z = target_list[i + 2];
      // 废除目标点的位姿功能
      // pose.orientation.x = target_list[i+3];
      // pose.orientation.y = target_list[i+4];
      // pose.orientation.z = target_list[i+5];
      // pose.orientation.w = target_list[i+6];
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      targetpoints.push_back(pose);
    }
    // if(targetpoints.size() != POINTS){
    //     ROS_ERROR("Received %d targetpoints, expected %d.",
    //     (int)targetpoints.size(), POINTS); return false;
    // }else {
    ROS_INFO("Received %zu points from parameter server. SUCCESS.",
             targetpoints.size());
    return true;
    // }
  } else {
    ROS_ERROR("Failed to load targetpoints from parameter server.");
    return false;
  }
}

// 当前位置回调函数
void currentPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("Received current position: [%.2f, %.2f, %.2f]",
           msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  current_world_body = *msg;
  current_pos_received = true;
}

int main(int argc, char **argv) {
  // 初始化 ROS 节点
  ros::init(argc, argv, "target_pub_node");
  // 创建节点句柄
  ros::NodeHandle nh;

  // 创建发布器，发布 geometry_msgs::PoseStamped 类型消息到 /target_position
  // 话题
  ros::Publisher target_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/target_position", 10);

  ros::Subscriber current_pos_sub =
      nh.subscribe("/current_world_body_pos", 10, currentPosCallback);

  // 设置发布频率为 1 Hz
  ros::Rate rate(1.0);

  // 定义目标值动态数组
  std::vector<geometry_msgs::PoseStamped> targetpoints;

  // 从参数服务器获取目标点
  if (!getTargetPoints(nh, targetpoints)) {
    ROS_ERROR("Failed to initialize target points. Exiting.");
    return -1;
  } else {
    for (size_t i = 0; i < targetpoints.size(); ++i) {
      ROS_INFO("Target point %zu: [%.2f, %.2f, %.2f]", i,
               targetpoints[i].pose.position.x, targetpoints[i].pose.position.y,
               targetpoints[i].pose.position.z);
    }
  }

  size_t current_target_index = 0; // 当前目标点的索引

  while (ros::ok()) {
    geometry_msgs::PoseStamped target_pose;

    if (current_target_index < targetpoints.size()) {
      target_pose = targetpoints[current_target_index];
      target_pose.header.stamp = ros::Time::now();
    } else {
      // 动态数组遍历完时，维持原先飞行状态，这里假设维持最后一个目标位置
      ROS_WARN(
          "Target points vector is empty, maintaining last target position.");
      if (!targetpoints.empty()) {
        target_pose = targetpoints.back();
      }
    }

    // 发布消息
    current_target = target_pose;
    target_pub.publish(target_pose);

    // 打印发布信息
    ROS_INFO("Published target position: [%.2f, %.2f, %.2f]",
             target_pose.pose.position.x, target_pose.pose.position.y,
             target_pose.pose.position.z);

    // 判断是否接近
    if (current_pos_received &&
        isClose(current_target, current_world_body, 0.3)) {
      ROS_INFO("Current position is close to the target position.");
      ROS_INFO("Current position: [%.2f, %.2f, %.2f]",
               current_world_body.pose.position.x,
               current_world_body.pose.position.y,
               current_world_body.pose.position.z);
      if (current_target_index < targetpoints.size()) {
        current_target_index++; // 移动到下一个目标点
        ROS_INFO("Moving to the next target.");
      }
    } else {
      ROS_INFO("Current position is not close to the target position.");
      ROS_INFO("Current position: [%.2f, %.2f, %.2f]",
               current_world_body.pose.position.x,
               current_world_body.pose.position.y,
               current_world_body.pose.position.z);
    }

    // 按照设定频率休眠
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}