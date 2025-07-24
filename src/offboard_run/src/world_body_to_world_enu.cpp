#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "mavros_msgs/State.h"
#include "offboard_run/SlidingWindowAverage.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/timer.h"
#include "std_msgs/Bool.h"
class Body2ENU {
 public:
  Body2ENU(ros::NodeHandle &nh)
      : tf_buffer_(), tf_listener_(tf_buffer_, nh), static_tf_broadcaster_() {
    nh.param("pub_world_pose", pub_world_pose_, false);
    ROS_INFO("Publish world pose: %s", pub_world_pose_ ? "true" : "false");
    // 初始化发布器
    tf_status_pub_ = nh.advertise<std_msgs::Bool>("/mavros_tf_status", 10);
    px4_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10,
        std::bind(&Body2ENU::px4_pose_cb, this, std::placeholders::_1));
    mavros_state_sub_ = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10,
        std::bind(&Body2ENU::state_cb, this, std::placeholders::_1));

    if (pub_world_pose_) {
      current_world_body_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
          "/current_world_body_pos", 10);
    }

    tf_cast_timer_ =
        nh.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent &) {
          if (tf_ready_) {
            static_tf_broadcaster_.sendTransform(world_enu_to_world_body_);
            ROS_INFO("Published static transform from world_enu to world_body");
          }
          std_msgs::Bool tf_status_msg;
          tf_status_msg.data = tf_ready_;
          tf_status_pub_.publish(tf_status_msg);
        });
  }

 private:
  ros::Publisher tf_status_pub_;
  ros::Publisher current_world_body_pos_pub_;
  ros::Subscriber px4_pose_sub_;
  ros::Subscriber mavros_state_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  geometry_msgs::TransformStamped world_enu_to_world_body_;
  bool tf_ready_ = false;
  SlidingWindowPoseAverage slid_window_avg = SlidingWindowPoseAverage(10);
  geometry_msgs::PoseStamped current_pose_;
  mavros_msgs::State current_state_;
  bool received_pose_ = false;
  bool pub_world_pose_ = false;

  ros::Timer tf_cast_timer_;

  void px4_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // 更新当前位姿
    current_pose_ = *msg;
    current_pose_.header.frame_id = "world_enu";
    received_pose_ = true;

    if (!tf_ready_) {
      slid_window_avg.addPose(current_pose_);
    } else {
      if (pub_world_pose_) {
        geometry_msgs::PoseStamped current_world_body;
        try {
          current_world_body = tf_buffer_.transform(current_pose_, "world_body",
                                                    ros::Duration(0.5));
          current_world_body_pos_pub_.publish(current_world_body);
        } catch (tf2::TransformException &ex) {
          ROS_WARN("Transform local pos failed: %s", ex.what());
        }
      }
    }
  }

  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;

    if (msg->armed && msg->mode == "OFFBOARD") {
      // 如果处于OFFBOARD模式且已解锁，初始化坐标变换
      if (!tf_ready_) {
        InitializeTransform();
        ROS_INFO("Initialized transform from world_enu to world_body");
        static_tf_broadcaster_.sendTransform(world_enu_to_world_body_);
      }
    }
  }

  void InitializeTransform() {
    // 记录初始位置，用于后续计算
    geometry_msgs::PoseStamped start_local_body =
        slid_window_avg.computeAveragePose();

    // 创建 world_enu -> world_body 的静态变换
    world_enu_to_world_body_.header.stamp = ros::Time::now();
    world_enu_to_world_body_.header.frame_id = "world_enu";
    world_enu_to_world_body_.child_frame_id = "world_body";

    // 设置变换参数
    world_enu_to_world_body_.transform.translation.x =
        start_local_body.pose.position.x;
    world_enu_to_world_body_.transform.translation.y =
        start_local_body.pose.position.y;
    world_enu_to_world_body_.transform.translation.z =
        start_local_body.pose.position.z;
    world_enu_to_world_body_.transform.rotation =
        start_local_body.pose.orientation;
    // static_tf_broadcaster_.sendTransform(world_enu_to_world_body_);

    ROS_INFO("Initialized body to world_enu transform");
    tf_ready_ = true;
  }
};

int main(int argc, char **argv) {
  // 初始化 ROS 节点
  ros::init(argc, argv, "world_tf_node");
  // 创建节点句柄
  ros::NodeHandle nh;
  Body2ENU body_to_enu(nh);
  ros::spin();

  return 0;
}