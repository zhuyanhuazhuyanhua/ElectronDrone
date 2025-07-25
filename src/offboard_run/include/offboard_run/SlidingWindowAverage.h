#pragma once
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <queue>

class SlidingWindowPoseAverage {
 public:
  SlidingWindowPoseAverage(int windowSize) : windowSize(windowSize) {}

  void clear() {
    std::queue<geometry_msgs::PoseStamped> empty;
    std::swap(poseQueue, empty);
    sumPositionX = 0.0;
    sumPositionY = 0.0;
    sumPositionZ = 0.0;
    sumQuaternion = Eigen::Quaterniond::Identity();
    quaternionCount = 0;
  }

  void addPose(const geometry_msgs::PoseStamped& newPose) {
    if (!poseQueue.empty() &&
        exceedsResetThreshold(newPose, poseQueue.back())) {
      reset(newPose);
    } else {
      poseQueue.push(newPose);
      sumPositionX += newPose.pose.position.x;
      sumPositionY += newPose.pose.position.y;
      sumPositionZ += newPose.pose.position.z;
      addQuaternion(toEigenQuaternion(newPose.pose.orientation));
    }

    if (poseQueue.size() > windowSize) {
      const auto& oldPose = poseQueue.front();
      sumPositionX -= oldPose.pose.position.x;
      sumPositionY -= oldPose.pose.position.y;
      sumPositionZ -= oldPose.pose.position.z;
      removeQuaternion(toEigenQuaternion(oldPose.pose.orientation));
      poseQueue.pop();
    }
  }

  geometry_msgs::PoseStamped computeAveragePose() const {
    geometry_msgs::PoseStamped avgPose;
    int n = poseQueue.size();
    avgPose.pose.position.x = sumPositionX / n;
    avgPose.pose.position.y = sumPositionY / n;
    avgPose.pose.position.z = sumPositionZ / n;

    avgPose.pose.orientation = toROSQuaternion(sumQuaternion);
    return avgPose;
  }

  int getSize() const { return poseQueue.size(); }

 private:
  void reset(const geometry_msgs::PoseStamped& newPose) {
    std::queue<geometry_msgs::PoseStamped> empty;
    std::swap(poseQueue, empty);
    sumPositionX = newPose.pose.position.x;
    sumPositionY = newPose.pose.position.y;
    sumPositionZ = newPose.pose.position.z;
    sumQuaternion = toEigenQuaternion(newPose.pose.orientation);
    poseQueue.push(newPose);
  }

  void addQuaternion(const Eigen::Quaterniond& q) {
    if (quaternionCount == 0) {
      sumQuaternion = q;
    } else {
      sumQuaternion =
          Eigen::Quaterniond(sumQuaternion.w() * quaternionCount + q.w(),
                             sumQuaternion.x() * quaternionCount + q.x(),
                             sumQuaternion.y() * quaternionCount + q.y(),
                             sumQuaternion.z() * quaternionCount + q.z())
              .normalized();
    }
    quaternionCount++;
  }

  void removeQuaternion(const Eigen::Quaterniond& q) {
    if (quaternionCount <= 1) {
      sumQuaternion = Eigen::Quaterniond::Identity();
      quaternionCount = 0;
    } else {
      sumQuaternion =
          Eigen::Quaterniond(sumQuaternion.w() * quaternionCount - q.w(),
                             sumQuaternion.x() * quaternionCount - q.x(),
                             sumQuaternion.y() * quaternionCount - q.y(),
                             sumQuaternion.z() * quaternionCount - q.z())
              .normalized();
      quaternionCount--;
    }
  }

  bool exceedsResetThreshold(const geometry_msgs::PoseStamped& a,
                             const geometry_msgs::PoseStamped& b) const {
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    double dz = a.pose.position.z - b.pose.position.z;
    return (dx * dx + dy * dy + dz * dz) > resetThresholdSquared;
  }

  Eigen::Quaterniond toEigenQuaternion(
      const geometry_msgs::Quaternion& q) const {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  geometry_msgs::Quaternion toROSQuaternion(const Eigen::Quaterniond& q) const {
    geometry_msgs::Quaternion ros_q;
    ros_q.w = q.w();
    ros_q.x = q.x();
    ros_q.y = q.y();
    ros_q.z = q.z();
    return ros_q;
  }

  int windowSize;
  double sumPositionX = 0.0;
  double sumPositionY = 0.0;
  double sumPositionZ = 0.0;
  Eigen::Quaterniond sumQuaternion = Eigen::Quaterniond::Identity();
  int quaternionCount = 0;
  std::queue<geometry_msgs::PoseStamped> poseQueue;
  const double resetThresholdSquared = 0.01 * 0.01;
};