// #define ENABLE_PX4_REBOOT

#include <XmlRpcValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#ifdef ENABLE_PX4_REBOOT // 【修改1】条件编译：仅在启用时包含CommandLong头文件
#include <mavros_msgs/CommandLong.h>
#endif
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
#include "ros/console.h"
#include "tf2/LinearMath/Quaternion.h"

class MissionController {
public:
  MissionController(ros::NodeHandle &nh) : nh_(nh), tf_listener_(tf_buffer_) {
    // 系统参数
    nh.param("system/use_camera_aim", use_camera_aim_, false);
    nh.param("system/auto_start_mission", auto_start_mission_, true);
    nh.param("mission/takeoff_altitude", takeoff_altitude_, 1.2);

    ROS_INFO("Configuration:");
    ROS_INFO("  Use camera aim: %s", use_camera_aim_ ? "true" : "false");
    ROS_INFO("  Auto start mission: %s",
             auto_start_mission_ ? "true" : "false");
    ROS_INFO("  Takeoff altitude: %.2f", takeoff_altitude_);
#ifdef ENABLE_PX4_REBOOT // 【修改2】条件编译：仅在启用时输出重启相关配置
    ROS_INFO("  PX4 Auto Reboot: ENABLED");
    ROS_INFO("  Max arm retries: %d", MAX_ARM_RETRIES);
    ROS_INFO("  Arm retry interval: %.1f seconds", ARM_RETRY_INTERVAL);
    ROS_INFO("  Reboot wait time: %.1f seconds", REBOOT_WAIT_TIME);
#endif

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
#ifdef ENABLE_PX4_REBOOT // 【修改3】条件编译：仅在启用时创建command客户端
    command_client_ =
        nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
#endif

    // 创建动作执行器
    action_executor_ = std::make_unique<ActionExecutor>(nh_, tf_buffer_);

    // 加载任务配置
    if (!loadMissionFromParam()) {
      ROS_ERROR("Failed to load mission configuration!");
    }

    // 初始化定时器
    init_timer_ = nh_.createTimer(ros::Duration(1.5),
                                  &MissionController::initCallback, this);

    ROS_INFO("Mission controller started!");
  }

  void controlLoop() {
    if (!action_executor_) {
      ROS_WARN("ActionExecutor not initialized, skipping control loop.");
      // send a dummy position setpoint to avoid blocking
      action_executor_->sendDummyPose();
      return;
    }

    if (!mission_running_) {
      ROS_WARN("Mission not running, skipping control loop.");
      ROS_INFO("Current state: initialized: %s, tf_ready: %s, connected: %s",
               initialized_ ? "true" : "false", tf_ready_ ? "true" : "false",
               current_state_.connected ? "true" : "false");
      action_executor_->sendDummyPose();
      return;
    }
    action_executor_->controlLoop();
  }

private:
#ifdef ENABLE_PX4_REBOOT // 【修改4】条件编译：预设重启相关常量
  static constexpr int MAX_ARM_RETRIES = 5;         // 最大重试次数
  static constexpr double ARM_RETRY_INTERVAL = 2.0; // 重试间隔（秒）
  static constexpr double REBOOT_WAIT_TIME = 10.0; // 重启后等待时间（秒）
#endif

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
        action = DroneAction::createTakeoffAction(altitude, 0.15);
        ROS_INFO("  Action %d: Takeoff to %.2f m", i, altitude);

      } else if (type == "land") {
        action = DroneAction::createLandAction();
        ROS_INFO("  Action %d: Land", i);

      } else if (type == "move") {
        XmlRpc::XmlRpcValue pos = mission_config[i]["position"];
        double yaw = mission_config[i]["yaw"];
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
        tf2::Quaternion q = tf2::Quaternion();
        q.setRPY(0, 0, yaw);
        target.pose.orientation = tf2::toMsg(q);

        bool use_body_frame = (frame == "world_body");
        action =
            DroneAction::createMoveToAction(target, use_body_frame, tolerance);
        ROS_INFO(
            "  Action %d: Move to [%.2f, %.2f, %.2f] in %s frame, yaw: %.2f "
            "degree",
            i, (double)pos[0], (double)pos[1], (double)pos[2], frame.c_str(),
            yaw * 180.0 / M_PI);

      } else if (type == "hover") {
        double duration = mission_config[i]["duration"];
        action = DroneAction::createHoverAction(duration);
        ROS_INFO("  Action %d: Hover for %.2f seconds", i, duration);

      } else if (type == "camera_aim") {
        XmlRpc::XmlRpcValue pos = mission_config[i]["position"];
        std::string frame = mission_config[i]["frame"];
        std::string axis_param = mission_config[i]["axis"];
        HoldAxis axis;
        if (axis_param == "x" || axis_param == "X") {
          axis = HoldAxis::X;
        } else if (axis_param == "y" || axis_param == "Y") {
          axis = HoldAxis::Y;
        } else if (axis_param == "z" || axis_param == "Z") {
          axis = HoldAxis::Z;
        } else {
          ROS_ERROR("Unknown camera aim axis: %s", axis_param.c_str());
          continue;
        }
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

        action =
            DroneAction::createCameraAimAction(target, tolerance, 0.5, axis);
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

#ifdef ENABLE_PX4_REBOOT // 【修改5】条件编译：仅在启用时包含PX4重启函数
  // PX4重启函数
  bool rebootPX4() {
    ROS_WARN("Attempting to reboot PX4...");

    mavros_msgs::CommandLong reboot_cmd;
    reboot_cmd.request.broadcast = false;
    reboot_cmd.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_cmd.request.confirmation = 0;
    reboot_cmd.request.param1 = 1; // 1: reboot autopilot
    reboot_cmd.request.param2 = 0; // 0: do nothing for onboard computer
    reboot_cmd.request.param3 = 0; // reserved
    reboot_cmd.request.param4 = 0; // reserved
    reboot_cmd.request.param5 = 0; // reserved
    reboot_cmd.request.param6 = 0; // reserved
    reboot_cmd.request.param7 = 0; // reserved

    if (command_client_.call(reboot_cmd)) {
      if (reboot_cmd.response.success) {
        ROS_INFO("PX4 reboot command sent successfully");

        // 重置状态变量
        initialized_ = false;
        initial_setpoints_sent_ = false;
        arm_retry_count_ = 0;
        waiting_after_reboot_ = true;
        reboot_start_time_ = ros::Time::now();

        ROS_INFO("Waiting %.1f seconds for PX4 to reboot...", REBOOT_WAIT_TIME);
        return true;
      } else {
        ROS_ERROR("PX4 reboot command failed: result=%d",
                  reboot_cmd.response.result);
        return false;
      }
    } else {
      ROS_ERROR("Failed to call PX4 reboot service");
      return false;
    }
  }
#endif

  void initCallback(const ros::TimerEvent &event) {
    ROS_INFO(
        "Initializing mission controller to control drone..., auto_start: %s",
        auto_start_mission_ ? "true" : "false");
    ROS_INFO("Now status is initialized: %s, tf_ready: %s, connected: %s",
             initialized_ ? "true" : "false", tf_ready_ ? "true" : "false",
             current_state_.connected ? "true" : "false");

#ifdef ENABLE_PX4_REBOOT // 【修改6】条件编译：仅在启用时包含重启后等待逻辑
    // 如果刚重启完成，等待一段时间
    if (waiting_after_reboot_) {
      double elapsed = (ros::Time::now() - reboot_start_time_).toSec();
      if (elapsed < REBOOT_WAIT_TIME) {
        ROS_INFO("Waiting for PX4 reboot... (%.1f/%.1f seconds)", elapsed,
                 REBOOT_WAIT_TIME);
        return;
      } else {
        ROS_INFO("PX4 reboot wait period completed");
        waiting_after_reboot_ = false;
      }
    }
#endif

    if (!initialized_ && current_state_.connected) {
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
          ROS_ERROR("Failed to set Offboard mode");
        }
      }
      // 解锁
      else if (!current_state_.armed) {
#ifdef ENABLE_PX4_REBOOT // 【修改7】条件编译：仅在启用时使用重试逻辑，否则使用原始逻辑
        // 带重试机制的解锁逻辑
        // 检查是否需要等待重试间隔
        if (arm_retry_count_ > 0) {
          double elapsed = (ros::Time::now() - last_arm_attempt_).toSec();
          if (elapsed < ARM_RETRY_INTERVAL) {
            return; // 还在等待重试间隔
          }
        }

        // 检查是否已达到最大重试次数
        if (arm_retry_count_ >= MAX_ARM_RETRIES) {
          ROS_ERROR("Failed to arm after %d attempts. Attempting PX4 reboot...",
                    MAX_ARM_RETRIES);

          if (rebootPX4()) {
            // 重启成功，重置重试计数
            arm_retry_count_ = 0;
          } else {
            ROS_ERROR("PX4 reboot failed. Stopping initialization.");
            init_timer_.stop();
          }
          return;
        }

        // 尝试解锁
        arm_retry_count_++;
        last_arm_attempt_ = ros::Time::now();

        ROS_INFO("Attempting to arm vehicle (attempt %d/%d)...",
                 arm_retry_count_, MAX_ARM_RETRIES);

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (arming_client_.call(arm_cmd)) {
          if (arm_cmd.response.success) {
            ROS_INFO("Vehicle armed successfully on attempt %d",
                     arm_retry_count_);
            initialized_ = true;
            arm_retry_count_ = 0; // 重置重试计数
            init_timer_.stop();

            // 如果配置为自动开始任务
            if (auto_start_mission_) {
              ROS_INFO("Mission started automatically");
              if (!startMission()) {
                start_mission_timer_ =
                    nh_.createTimer(ros::Duration(0.1),
                                    &MissionController::startMissionCB, this);
              }
            } else {
              ROS_INFO("Waiting for mission start command...");
            }
          } else {
            ROS_WARN("Arm command rejected (attempt %d/%d): %s",
                     arm_retry_count_, MAX_ARM_RETRIES,
                     arm_cmd.response.result == 4 ? "EKF2 not ready"
                                                  : "Unknown error");

            if (arm_retry_count_ < MAX_ARM_RETRIES) {
              ROS_INFO("Will retry arming in %.1f seconds...",
                       ARM_RETRY_INTERVAL);
            }
          }
        } else {
          ROS_ERROR("Arming service call failed (attempt %d/%d)",
                    arm_retry_count_, MAX_ARM_RETRIES);

          if (arm_retry_count_ < MAX_ARM_RETRIES) {
            ROS_INFO("Will retry arming in %.1f seconds...",
                     ARM_RETRY_INTERVAL);
          }
        }
#else
        // 原始解锁逻辑（保持不变）
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (arming_client_.call(arm_cmd)) {
          if (arm_cmd.response.success) {
            ROS_INFO("Vehicle armed successfully");
            initialized_ = true;
            init_timer_.stop();

            // 如果配置为自动开始任务
            if (auto_start_mission_) {
              ROS_INFO("Mission started automatically");
              if (!startMission()) {
                start_mission_timer_ =
                    nh_.createTimer(ros::Duration(0.1),
                                    &MissionController::startMissionCB, this);
              }
            } else {
              ROS_INFO("Waiting for mission start command...");
            }
          } else {
            ROS_ERROR("Failed to arm vehicle");
          }
        } else {
          ROS_ERROR("Arming service call failed");
        }
#endif
      }
    }
  }

  void startMissionCB(const ros::TimerEvent &event) {
    if (startMission()) {
      start_mission_timer_.stop();
      ROS_INFO("Mission started successfully");
    } else {
      ROS_ERROR("Failed to start mission");
    }
  }

  bool startMission() {
    ROS_INFO("Starting mission...");
    if (!initialized_) {
      ROS_WARN("Cannot start mission: system not initialized");
      return false;
    }

    if (mission_actions_.empty()) {
      ROS_WARN("Cannot start mission: no actions loaded");
      return false;
    }

    if (!tf_ready_) {
      ROS_WARN("Waiting for TF to be ready before starting mission");
      return false;
    }

    // while (ros::ok() && !tf_ready_) {
    //   ros::Duration(0.05).sleep();
    //   ros::spinOnce();
    // }

    ROS_INFO("Starting mission with %zu actions", mission_actions_.size());

    // 清空当前队列并添加所有任务动作
    action_executor_->clearActions();
    for (const auto &action : mission_actions_) {
      action_executor_->addAction(action);
    }

    mission_running_ = true;
    return true;
  }

  void stopMission() {
    ROS_INFO("Stopping mission");
    action_executor_->emergencyStop();
    mission_running_ = false;
  }

  // 回调函数
  void tfStatusCb(const std_msgs::Bool::ConstPtr &msg) {
    tf_ready_ = msg->data;
    if (tf_ready_ && !tf_ready_logged_) {
      ROS_INFO("TF is ready!");
      tf_ready_logged_ = true;
    }
  }

  void stateCb(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
  }

  void startMissionCb(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Received start mission command");
    startMission();
  }

  void stopMissionCb(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Received stop mission command");
    stopMission();
  }

private:
  ros::NodeHandle &nh_;
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
#ifdef ENABLE_PX4_REBOOT // 【修改8】条件编译：仅在启用时声明command客户端
  ros::ServiceClient command_client_;
#endif

  // 定时器
  ros::Timer init_timer_;
  ros::Timer start_mission_timer_;

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

#ifdef ENABLE_PX4_REBOOT // 【修改9】条件编译：仅在启用时声明重试相关成员变量
  // 解锁重试相关变量
  int arm_retry_count_ = 0;           // 当前重试次数
  ros::Time last_arm_attempt_;        // 上次解锁尝试时间
  bool waiting_after_reboot_ = false; // 是否正在等待重启完成
  ros::Time reboot_start_time_;       // 重启开始时间
#endif
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mission_controller_node");
  ros::NodeHandle nh;

  try {
    MissionController controller(nh);
    ros::Rate rate(50); // 50Hz 控制频率
    while (ros::ok()) {
      controller.controlLoop();
      ros::spinOnce();
      rate.sleep();
    }
  } catch (const std::exception &e) {
    ROS_ERROR("Exception: %s", e.what());
    return -1;
  }

  return 0;
}