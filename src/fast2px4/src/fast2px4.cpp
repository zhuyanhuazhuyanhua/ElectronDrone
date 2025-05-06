#include <Eigen/Eigen>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>

Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_mav;
Eigen::Quaterniond q_px4_odom;
geometry_msgs::PoseStamped vision;

class SlidingWindowAverage {
public:
  SlidingWindowAverage(int windowSize)
      : windowSize(windowSize), windowSum(0.0) {}

  double addData(double newData) {
    if (!dataQueue.empty() && fabs(newData - dataQueue.back()) > 0.01) {
      dataQueue = std::queue<double>();
      windowSum = 0.0;
      dataQueue.push(newData);
      windowSum += newData;
    } else {
      dataQueue.push(newData);
      windowSum += newData;
    }

    // 如果队列大小超过窗口大小，弹出队列头部元素并更新窗口和队列和
    if (dataQueue.size() > windowSize) {
      windowSum -= dataQueue.front();
      dataQueue.pop();
    }
    windowAvg = windowSum / dataQueue.size();
    // 返回当前窗口内的平均值
    return windowAvg;
  }

  int get_size() { return dataQueue.size(); }

  double get_avg() { return windowAvg; }

private:
  int windowSize;
  double windowSum;
  double windowAvg;
  std::queue<double> dataQueue;
};

int windowSize = 8;
SlidingWindowAverage swa = SlidingWindowAverage(windowSize);

double fromQuaternion2yaw(Eigen::Quaterniond q) {
  double yaw =
      atan2(2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  return yaw;
}

void vins_callback(const nav_msgs::Odometry::ConstPtr &msg) {

  p_lidar_body =
      Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  vision.header.stamp = msg->header.stamp;
  q_mav = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  q_px4_odom = Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  swa.addData(fromQuaternion2yaw(q_px4_odom));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins_to_mavros");
  ros::NodeHandle nh("~");

  ros::Subscriber slam_sub =
      nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, vins_callback);
  ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/mavros/local_position/odom", 5, px4_odom_callback);

  ros::Publisher vision_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ros::Time last_request = ros::Time::now();
  float init_yaw = 0.0;
  bool init_flag = 0;
  Eigen::Quaterniond init_q;
  while (ros::ok()) {
    if (swa.get_size() == windowSize && !init_flag) {
      init_yaw = swa.get_avg();
      init_flag = 1;
      init_q = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ()) // des.yaw
               * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      // delete swa;
    }

    if (init_flag) {
      p_enu = init_q * p_lidar_body;

      vision.pose.position.x = p_enu[0];
      vision.pose.position.y = p_enu[1];
      vision.pose.position.z = p_enu[2];

      vision.pose.orientation.x = q_mav.x();
      vision.pose.orientation.y = q_mav.y();
      vision.pose.orientation.z = q_mav.z();
      vision.pose.orientation.w = q_mav.w();

      vision_pub.publish(vision);

      ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: "
               "%.18f\norientation of lidar:\n   x: %.18f\n   y: %.18f\n   z: "
               "%.18f\n   w: %.18f",
               p_enu[0], p_enu[1], p_enu[2], q_mav.x(), q_mav.y(), q_mav.z(),
               q_mav.w());
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
