#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "ActionExecutor.hpp"

class MissionController {
 public:
  MissionController(ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_) {
    // 系统参数
    nh.param("system/use_camera_aim", use_camera_aim_, false);
    nh.param("system/auto_start_mission", auto_start_mission_, true);
    nh.param("mission/takeoff_altitude", takeoff_altitude_, 1.2);

    ROS_INFO("Configuration:");
    ROS_INFO("  Use camera aim: %s", use_camera_aim_ ? "true" : "false");
    ROS_INFO("  Auto start mission: %s",
             auto_start_mission_ ? "true" : "false");
    ROS_INFO("  Takeoff altitude: %.2f", takeoff_altitude_);

    // 订阅器
    tf_status_sub_ = nh_.subscribe<std_msgs::Bool>(
        "/mavros_tf_status", 10, &MissionController::tfStatusCb, this);

    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, &MissionController::stateCb, this);

    // 任务控制订阅器
    start_mission_sub_ = nh_.subscribe<std_msgs::Empty>(
        "/start_mission", 1, &MissionController::startMissionCb, this);

    stop_mission_sub_ = nh_.subscribe<std_msgs::Empty>(
        "/stop_mission", 1, &MissionController::stopMissionCb, this);

    // 服务客户端
    arming_client_ =
        nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 创建动作执行器
    action_executor_ = std::make_unique<ActionExecutor>(nh_, tf_buffer_);

    // 加载任务配置
    if (!loadMissionFromParam()) {
      ROS_ERROR("Failed to load mission configuration!");
    }

    // 初始化定时器
    init_timer_ = nh_.createTimer(ros::Duration(1.0),
                                  &MissionController::initCallback, this);

    ROS_INFO("Mission controller started!");
  }

 private:
  bool loadMissionFromParam() {
    XmlRpc::XmlRpcValue mission_config;
    if (!nh_.getParam("mission/actions", mission_config)) {
      ROS_ERROR("No mission actions found in parameter server!");
      return false;
    }

    if (mission_config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR("Mission actions must be an array!");
      return false;
    }

    ROS_INFO("Loading mission with %d actions...", mission_config.size());

    // 清空之前的任务
    mission_actions_.clear();

    // 解析每个动作
    for (int i = 0; i < mission_config.size(); ++i) {
      if (mission_config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("Action %d is not a struct!", i);
        continue;
      }

      std::string type = mission_config[i]["type"];
      std::shared_ptr<DroneAction> action;

      if (type == "takeoff") {
        double altitude = mission_config[i]["altitude"];
        action = DroneAction::createTakeoffAction(altitude);
        ROS_INFO("  Action %d: Takeoff to %.2f m", i, altitude);

      } else if (type == "land") {
        action = DroneAction::createLandAction();
        ROS_INFO("  Action %d: Land", i);

      } else if (type == "move") {
        XmlRpc::XmlRpcValue pos = mission_config[i]["position"];
        std::string frame = mission_config[i]["frame"];
        double tolerance = 0.1;
        if (mission_config[i].hasMember("tolerance")) {
          tolerance = mission_config[i]["tolerance"];
        }

        geometry_msgs::PoseStamped target;
        target.header.frame_id = frame;
        target.pose.position.x = pos[0];
        target.pose.position.y = pos[1];
        target.pose.position.z = pos[2];
        target.pose.orientation.w = 1.0;

        bool use_body_frame = (frame == "world_body");
        action =
            DroneAction::createMoveToAction(target, use_body_frame, tolerance);
        ROS_INFO("  Action %d: Move to [%.2f, %.2f, %.2f] in %s frame", i,
                 (double)pos[0], (double)pos[1], (double)pos[2], frame.c_str());

      } else if (type == "hover") {
        double duration = mission_config[i]["duration"];
        action = DroneAction::createHoverAction(duration);
        ROS_INFO("  Action %d: Hover for %.2f seconds", i, duration);

      } else if (type == "camera_aim") {
        XmlRpc::XmlRpcValue pos = mission_config[i]["position"];
        std::string frame = mission_config[i]["frame"];
        double tolerance = 10.0;
        if (mission_config[i].hasMember("tolerance")) {
          tolerance = mission_config[i]["tolerance"];
        }

        geometry_msgs::PoseStamped target;
        target.header.frame_id = frame;
        target.pose.position.x = pos[0];
        target.pose.position.y = pos[1];
        target.pose.position.z = pos[2];
        target.pose.orientation.w = 1.0;

        action = DroneAction::createCameraAimAction(target, tolerance);
        ROS_INFO("  Action %d: Camera aim at [%.2f, %.2f, %.2f]", i,
                 (double)pos[0], (double)pos[1], (double)pos[2]);

      } else {
        ROS_WARN("Unknown action type: %s", type.c_str());
        continue;
      }

      if (action) {
        mission_actions_.push_back(action);
      }
    }

    ROS_INFO("Mission loaded with %zu valid actions", mission_actions_.size());
    return !mission_actions_.empty();
  }

  void initCallback(const ros::TimerEvent& event) {
    if (!initialized_ && tf_ready_ && current_state_.connected) {
      // 发送初始设定点
      if (!initial_setpoints_sent_) {
        ROS_INFO("Sending initial setpoints...");
        geometry_msgs::PoseStamped hover_pose;
        hover_pose.header.frame_id = "world_enu";
        hover_pose.header.stamp = ros::Time::now();
        hover_pose.pose.position.x = 0;
        hover_pose.pose.position.y = 0;
        hover_pose.pose.position.z = takeoff_altitude_;
        hover_pose.pose.orientation.w = 1.0;

        ros::Rate rate(50);
        for (int i = 0; i < 50 && ros::ok(); i++) {
          action_executor_->sendPositionSetpoint(hover_pose);
          ros::spinOnce();
          rate.sleep();
        }
        initial_setpoints_sent_ = true;
        ROS_INFO("Initial setpoints sent!");
      }

      // 切换到OFFBOARD模式
      if (current_state_.mode != "OFFBOARD") {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if (set_mode_client_.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent) {
          ROS_INFO("Offboard mode enabled");
        } else {
          ROS_WARN("Failed to set Offboard mode");
        }
      }
      // 解锁
      else if (!current_state_.armed) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (arming_client_.call(arm_cmd)) {
          if (arm_cmd.response.success) {
            ROS_INFO("Vehicle armed successfully");
            initialized_ = true;
            init_timer_.stop();

            // 如果配置为自动开始任务
            if (auto_start_mission_) {
              startMission();
            } else {
              ROS_INFO("Waiting for mission start command...");
            }
          } else {
            ROS_WARN("Failed to arm vehicle");
          }
        } else {
          ROS_ERROR("Arming service call failed");
        }
      }
    }
  }

  void startMission() {
    if (!initialized_) {
      ROS_WARN("Cannot start mission: system not initialized");
      return;
    }

    if (mission_actions_.empty()) {
      ROS_WARN("Cannot start mission: no actions loaded");
      return;
    }

    ROS_INFO("Starting mission with %zu actions", mission_actions_.size());

    // 清空当前队列并添加所有任务动作
    action_executor_->clearActions();
    for (const auto& action : mission_actions_) {
      action_executor_->addAction(action);
    }

    mission_running_ = true;
  }

  void stopMission() {
    ROS_INFO("Stopping mission");
    action_executor_->emergencyStop();
    mission_running_ = false;
  }

  // 回调函数
  void tfStatusCb(const std_msgs::Bool::ConstPtr& msg) {
    tf_ready_ = msg->data;
    if (tf_ready_ && !tf_ready_logged_) {
      ROS_INFO("TF is ready!");
      tf_ready_logged_ = true;
    }
  }

  void stateCb(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
  }

  void startMissionCb(const std_msgs::Empty::ConstPtr& msg) {
    ROS_INFO("Received start mission command");
    startMission();
  }

  void stopMissionCb(const std_msgs::Empty::ConstPtr& msg) {
    ROS_INFO("Received stop mission command");
    stopMission();
  }

 private:
  ros::NodeHandle& nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 订阅器
  ros::Subscriber tf_status_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber start_mission_sub_;
  ros::Subscriber stop_mission_sub_;

  // 服务客户端
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  // 定时器
  ros::Timer init_timer_;

  // 动作执行器
  std::unique_ptr<ActionExecutor> action_executor_;

  // 任务动作序列
  std::vector<std::shared_ptr<DroneAction>> mission_actions_;

  // 状态
  bool tf_ready_ = false;
  bool tf_ready_logged_ = false;
  bool initialized_ = false;
  bool initial_setpoints_sent_ = false;
  bool use_camera_aim_ = false;
  bool auto_start_mission_ = true;
  bool mission_running_ = false;
  double takeoff_altitude_ = 1.2;
  mavros_msgs::State current_state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_controller_node");
  ros::NodeHandle nh;

  try {
    MissionController controller(nh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Exception: %s", e.what());
    return -1;
  }

  return 0;
}