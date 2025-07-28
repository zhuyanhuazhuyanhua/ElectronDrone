#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>  // 添加这个以使用 ros::Time

#include <cmath>
#include <memory>

struct SpatialPoint {
  double x;
  double y;
  double z;

  SpatialPoint(double x = 0.0, double y = 0.0, double z = 0.0)
      : x(x), y(y), z(z) {}

  SpatialPoint(const geometry_msgs::PoseStamped &pose)
      : x(pose.pose.position.x),
        y(pose.pose.position.y),
        z(pose.pose.position.z) {}

  SpatialPoint(const geometry_msgs::Point &point)
      : x(point.x), y(point.y), z(point.z) {}

  double distance(const SpatialPoint &other) const {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2) +
                     std::pow(z - other.z, 2));
  }

  double distanceXY(const SpatialPoint &other) const {
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
  }
};

// 动作类型枚举
enum class ActionType {
  MOVE_TO_POSITION = 0,  // 移动到指定位置
  HOVER,                 // 悬停
  CAMERA_AIM,            // 相机瞄准
  LAND,                  // 降落
  TAKEOFF                // 起飞
};

// 动作状态枚举
enum class ActionStatus {
  PENDING,    // 等待执行
  EXECUTING,  // 正在执行
  COMPLETED,  // 已完成
  FAILED,     // 失败
  ABORTED     // 中止
};

enum class HoldAxis {
  X = 0,  // 保持X轴不变
  Y,      // 保持Y轴不变
  Z       // 保持Z轴不变
};

class DroneAction {
 private:
  // 将构造函数声明为私有，但让工厂方法可以访问
  struct PrivateTag {};  // 用于控制访问的标签

 public:
  // 允许 make_shared 访问的构造函数
  explicit DroneAction(PrivateTag) {}

  enum class Frame {
    BODY = 0,
    WORLD_BODY,
    WORLD_ENU,
  };

  // 移动到位置的构造函数
  static std::shared_ptr<DroneAction> createMoveToAction(
      const geometry_msgs::PoseStamped &target_pose,
      Frame use_body_frame = Frame::WORLD_BODY,
      double position_tolerance = 0.1) {
    auto action = std::make_shared<DroneAction>(PrivateTag{});
    action->type_ = ActionType::MOVE_TO_POSITION;
    action->target_pose_ = target_pose;
    action->use_body_frame_ = use_body_frame;
    action->position_tolerance_ = position_tolerance;
    return action;
  }

  // 悬停动作
  static std::shared_ptr<DroneAction> createHoverAction(double hover_time_s) {
    auto action = std::make_shared<DroneAction>(PrivateTag{});
    action->type_ = ActionType::HOVER;
    action->hover_time_s_ = hover_time_s;
    return action;
  }

  // 相机瞄准动作
  static std::shared_ptr<DroneAction> createCameraAimAction(
      const geometry_msgs::PoseStamped &target_pose,
      double camera_aim_tolerance = 10.0,  // 像素
      double position_tolerance = 0.5,
      HoldAxis axis = HoldAxis::Z) {  // 米
    auto action = std::make_shared<DroneAction>(PrivateTag{});
    action->type_ = ActionType::CAMERA_AIM;
    action->target_pose_ = target_pose;
    action->camera_aim_tolerance_ = camera_aim_tolerance;
    action->position_tolerance_ = position_tolerance;
    return action;
  }

  // 降落动作
  static std::shared_ptr<DroneAction> createLandAction() {
    auto action = std::make_shared<DroneAction>(PrivateTag{});
    action->type_ = ActionType::LAND;
    return action;
  }

  // 起飞动作
  static std::shared_ptr<DroneAction> createTakeoffAction(
      double target_altitude, double tolerance = 0.1) {
    auto action = std::make_shared<DroneAction>(PrivateTag{});
    action->type_ = ActionType::TAKEOFF;
    action->target_altitude_ = target_altitude;
    action->position_tolerance_ = tolerance;
    return action;
  }

  // Getters
  ActionType getType() const { return type_; }
  ActionStatus getStatus() const { return status_; }
  geometry_msgs::PoseStamped getTargetPose() const { return target_pose_; }
  Frame useBodyFrame() const { return use_body_frame_; }
  double getHoverTime() const { return hover_time_s_; }
  double getPositionTolerance() const { return position_tolerance_; }
  double getCameraAimTolerance() const { return camera_aim_tolerance_; }
  double getTargetAltitude() const { return target_altitude_; }
  ros::Time getStartTime() const { return start_time_; }
  HoldAxis getHoldAxis() const { return axis_; }
  bool isStartPoseInitialized() const { return init_start_pose_; }
  geometry_msgs::PoseStamped getStartPose() const {
    return init_start_pose_ ? start_pose_ : geometry_msgs::PoseStamped();
  }

  // Setters
  void setStatus(ActionStatus status) { status_ = status; }
  void setStartTime(ros::Time time) { start_time_ = time; }
  void setStartPose(const geometry_msgs::PoseStamped &pose) {
    start_pose_ = pose;
    init_start_pose_ = true;
  }

  // 检查是否已完成
  bool isCompleted() const { return status_ == ActionStatus::COMPLETED; }

  // 检查是否正在执行
  bool isExecuting() const { return status_ == ActionStatus::EXECUTING; }

  // 检查是否失败
  bool isFailed() const { return status_ == ActionStatus::FAILED; }

  // 检查是否中止
  bool isAborted() const { return status_ == ActionStatus::ABORTED; }

 private:
  ActionType type_;
  ActionStatus status_ = ActionStatus::PENDING;

  geometry_msgs::PoseStamped target_pose_;
  Frame use_body_frame_ = Frame::WORLD_BODY;
  double hover_time_s_ = 0.0;
  double position_tolerance_ = 0.1;     // 位置容差（米）
  double camera_aim_tolerance_ = 10.0;  // 相机瞄准容差（像素）
  double target_altitude_ = 1.2;        // 目标高度
  HoldAxis axis_ = HoldAxis::Z;         // 保持的轴

  geometry_msgs::PoseStamped start_pose_;
  bool init_start_pose_ = false;  // 是否已初始化起始位置

  ros::Time start_time_;
};