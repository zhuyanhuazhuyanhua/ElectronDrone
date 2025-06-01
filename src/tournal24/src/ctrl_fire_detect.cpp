#include "ros/console.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

// 全局变量，用于存储接收到的消息
geometry_msgs::PoseStamped current_target;
geometry_msgs::PoseStamped current_world_body;
bool current_pos_received = false;

// 悬停相关变量
bool is_hanging = false;     // 是否正在悬停
ros::Time hang_start_time;   // 悬停开始时间
bool qr_scan_called = false; // 本次悬停是否已调用二维码扫描

// 判断两个位置是否接近
bool isClose(const geometry_msgs::PoseStamped &pose1,
             const geometry_msgs::PoseStamped &pose2, double threshold)
{
  double distance =
      std::sqrt(std::pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                std::pow(pose1.pose.position.y - pose2.pose.position.y, 2) +
                std::pow(pose1.pose.position.z - pose2.pose.position.z, 2));
  return distance < threshold;
}

bool getTargetPoints(ros::NodeHandle &nh,
                     std::vector<geometry_msgs::PoseStamped> &targetpoints)
{
  std::vector<float> target_list;

  if (nh.getParam("targetpoints", target_list))
  {
    if (target_list.size() % 3 !=
        0)
    { // 每个目标点有3个元素 (x, y, z, ox, oy, oz, ow) 四元数弃置
      ROS_ERROR("Invalid targetpoints size. Each target point should have 3 "
                "elements (x, y, z).");
      return false;
    }

    for (size_t i = 0; i < target_list.size(); i += 3)
    {
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
  }
  else
  {
    ROS_ERROR("Failed to load targetpoints from parameter server.");
    return false;
  }
}

// 当前位置回调函数
void currentPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ROS_INFO("Received current position: [%.2f, %.2f, %.2f]",
           msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  current_world_body = *msg;
  current_pos_received = true;
}

int main(int argc, char **argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "target_pub_node");
  // 创建节点句柄
  ros::NodeHandle nh;

  // 修改服务客户端名称，指向红色检测服务
  ros::ServiceClient red_client = nh.serviceClient<std_srvs::Trigger>(
      "/analyse_red"); // 红色检测服务客户端
  ros::Publisher qr_pub =
      nh.advertise<std_msgs::String>("/red_detection_content", 10); // 发布红色检测信息

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
  if (!getTargetPoints(nh, targetpoints))
  {
    ROS_ERROR("Failed to initialize target points. Exiting.");
    return -1;
  }
  else
  {
    for (size_t i = 0; i < targetpoints.size(); ++i)
    {
      ROS_INFO("Target point %zu: [%.2f, %.2f, %.2f]", i,
               targetpoints[i].pose.position.x, targetpoints[i].pose.position.y,
               targetpoints[i].pose.position.z);
    }
  }

  bool hang = false; // 是否悬停
  if (nh.getParam("hang", hang))
  {
    ROS_INFO("Hang parameter: %s", hang ? "true" : "false");
  }
  else
  {
    ROS_WARN("Hang parameter not found, defaulting to false.");
  }

  std::vector<double> hang_time_list; // 悬停时间数组
  if (nh.getParam("hang_time", hang_time_list))
  {
    ROS_INFO("Received %zu hang time values", hang_time_list.size());
    for (size_t i = 0; i < hang_time_list.size(); ++i)
    {
      ROS_INFO("Hang time for point %zu: %.2f seconds", i, hang_time_list[i]);
    }
  }
  else
  {
    ROS_WARN("Hang time parameter not found, using default values.");
    // 如果没有悬停时间参数，为每个目标点设置默认悬停时间
    hang_time_list.resize(targetpoints.size(), 2.0); // 默认2秒
  }

  // 确保悬停时间数组和目标点数组大小一致
  if (hang_time_list.size() != targetpoints.size())
  {
    ROS_WARN("Hang time array size (%zu) doesn't match target points size "
             "(%zu). Adjusting...",
             hang_time_list.size(), targetpoints.size());
    hang_time_list.resize(targetpoints.size(), 2.0); // 用2秒填充缺失的值
  }

  size_t current_target_index = 0; // 当前目标点的索引

  while (ros::ok())
  {
    geometry_msgs::PoseStamped target_pose;

    if (current_target_index < targetpoints.size())
    {
      target_pose = targetpoints[current_target_index];
      target_pose.header.stamp = ros::Time::now();
    }
    else
    {
      // 动态数组遍历完时，维持原先飞行状态，这里假设维持最后一个目标位置
      ROS_WARN(
          "Target points vector is empty, maintaining last target position.");
      if (!targetpoints.empty())
      {
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

    // 判断是否接近目标点
    if (current_pos_received &&
        isClose(current_target, current_world_body, 0.3))
    {

      if (hang && !is_hanging && current_target_index < targetpoints.size())
      {
        // 开始悬停
        is_hanging = true;
        qr_scan_called = false; // 重置扫描标志
        hang_start_time = ros::Time::now();
        ROS_INFO("Started hanging at target point %zu for %.2f seconds",
                 current_target_index, hang_time_list[current_target_index]);
      }

      if (is_hanging)
      {
        // 在悬停开始时调用红色检测服务（只调用一次）
        if (!qr_scan_called)
        {
          ROS_INFO("Calling red detection service...");
          std_srvs::Trigger srv;

          if (red_client.call(srv))
          {
            if (srv.response.success)
            {
              ROS_INFO("Red detection successful: %s",
                       srv.response.message.c_str());

              // 发布红色检测内容
              std_msgs::String qr_msg;
              qr_msg.data = srv.response.message;
              qr_pub.publish(qr_msg);

              ROS_INFO("Published red detection content: %s", qr_msg.data.c_str());
            }
            else
            {
              ROS_WARN("Red detection failed: %s",
                       srv.response.message.c_str());

              // 即使失败也发布空消息或错误信息
              std_msgs::String qr_msg;
              qr_msg.data = "RED_DETECTION_FAILED: " + srv.response.message;
              qr_pub.publish(qr_msg);
            }
          }
          else
          {
            ROS_ERROR("Failed to call red detection service");

            // 服务调用失败时发布错误信息
            std_msgs::String qr_msg;
            qr_msg.data = "RED_SERVICE_UNAVAILABLE";
            qr_pub.publish(qr_msg);
          }

          qr_scan_called = true; // 标记已调用
        }

        // 检查悬停时间是否结束
        double elapsed_time = (ros::Time::now() - hang_start_time).toSec();
        double required_hang_time =
            (current_target_index < hang_time_list.size())
                ? hang_time_list[current_target_index]
                : 2.0;

        ROS_INFO("Hanging... elapsed: %.2f/%.2f seconds", elapsed_time,
                 required_hang_time);

        if (elapsed_time >= required_hang_time)
        {
          // 悬停时间结束，移动到下一个目标点
          is_hanging = false;
          qr_scan_called = false; // 重置扫描标志，为下次悬停做准备
          if (current_target_index < targetpoints.size())
          {
            current_target_index++; // 移动到下一个目标点
            ROS_INFO("Hang complete. Moving to the next target.");
          }
        }
      }
      else if (!hang)
      {
        // 如果不需要悬停，直接移动到下一个目标点
        ROS_INFO("Current position is close to the target position.");
        ROS_INFO("Current position: [%.2f, %.2f, %.2f]",
                 current_world_body.pose.position.x,
                 current_world_body.pose.position.y,
                 current_world_body.pose.position.z);
        if (current_target_index < targetpoints.size())
        {
          current_target_index++; // 移动到下一个目标点
          ROS_INFO("Moving to the next target.");
        }
      }
    }
    else
    {
      // 如果离开了目标点，停止悬停状态
      if (is_hanging)
      {
        is_hanging = false;
        qr_scan_called = false; // 重置扫描标志
        ROS_INFO("Left target position, stopping hang.");
      }

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