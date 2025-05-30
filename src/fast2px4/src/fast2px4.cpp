#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <queue>

class SlidingWindowPose {
public:
  SlidingWindowPose(int windowSize) : windowSize_(windowSize) {}

  bool addPose(const geometry_msgs::Pose &newPose) {
    // 检查位置变化是否过大（阈值：0.1m）
    if (!poseQueue_.empty()) {
      const auto &lastPose = poseQueue_.back();
      double pos_diff = sqrt(pow(newPose.position.x - lastPose.position.x, 2) +
                             pow(newPose.position.y - lastPose.position.y, 2) +
                             pow(newPose.position.z - lastPose.position.z, 2));

      // 如果位置变化过大，重置队列
      if (pos_diff > 0.1) {
        clearQueue();
      }
    }

    poseQueue_.push(newPose);

    // 保持窗口大小
    if (poseQueue_.size() > windowSize_) {
      poseQueue_.pop();
    }

    return poseQueue_.size() >= windowSize_;
  }

  int size() const { return poseQueue_.size(); }

  void clear() { clearQueue(); }

  geometry_msgs::Pose getAveragePose() const {
    if (poseQueue_.empty()) {
      return geometry_msgs::Pose();
    }

    // 计算位置平均值
    geometry_msgs::Pose avgPose;
    avgPose.position.x = 0.0;
    avgPose.position.y = 0.0;
    avgPose.position.z = 0.0;

    // 四元数平均 - 累加分量后归一化
    std::queue<geometry_msgs::Pose> tempQueue = poseQueue_;
    tf2::Quaternion refQuat, sumQuat(0, 0, 0, 0);

    bool first = true;
    int count = 0;

    while (!tempQueue.empty()) {
      const auto &pose = tempQueue.front();
      tempQueue.pop();

      // 累加位置
      avgPose.position.x += pose.position.x;
      avgPose.position.y += pose.position.y;
      avgPose.position.z += pose.position.z;

      // 四元数平均处理
      tf2::Quaternion quat;
      tf2::fromMsg(pose.orientation, quat);

      if (first) {
        refQuat = quat;
        sumQuat = quat;
        first = false;
      } else {
        // 确保四元数在同一半球（避免翻转）
        if (refQuat.dot(quat) < 0) {
          quat = tf2::Quaternion(-quat.x(), -quat.y(), -quat.z(), -quat.w());
        }
        // 累加四元数分量
        sumQuat.setX(sumQuat.x() + quat.x());
        sumQuat.setY(sumQuat.y() + quat.y());
        sumQuat.setZ(sumQuat.z() + quat.z());
        sumQuat.setW(sumQuat.w() + quat.w());
      }
      count++;
    }

    // 计算平均位置
    avgPose.position.x /= count;
    avgPose.position.y /= count;
    avgPose.position.z /= count;

    // 归一化平均四元数
    sumQuat.normalize();
    avgPose.orientation = tf2::toMsg(sumQuat);

    return avgPose;
  }

private:
  void clearQueue() { poseQueue_ = std::queue<geometry_msgs::Pose>(); }

  int windowSize_;
  std::queue<geometry_msgs::Pose> poseQueue_;
};

class SlamToMavrosConverter {
public:
  SlamToMavrosConverter(ros::NodeHandle &nh)
      : nh_(nh), slidingWindowMavros_(8), slidingWindowSlam_(8),
        initialized_(false), slamDataReceived_(false),
        mavrosDataReceived_(false), tf_buffer_(ros::Duration(10.0)) {

    // TF监听器
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // 订阅话题
    mavros_odom_sub_ =
        nh_.subscribe("/mavros/local_position/odom", 10,
                      &SlamToMavrosConverter::mavrosOdomCallback, this);
    slam_odom_sub_ = nh_.subscribe(
        "/Odometry", 10, &SlamToMavrosConverter::slamOdomCallback, this);

    // 发布话题
    vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 10);

    // 静态变换广播器
    static_tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>();

    // 创建定时器，定期检查初始化状态
    init_timer_ = nh_.createTimer(
        ros::Duration(1.0), &SlamToMavrosConverter::initTimerCallback, this);

    // 重置服务
    reset_service_ = nh_.advertiseService(
        "reset_initialization", &SlamToMavrosConverter::resetCallback, this);

    ROS_INFO("SLAM to MAVROS converter initialized");
    ROS_INFO("Waiting for MAVROS (/mavros/local_position/odom) and SLAM "
             "(/Odometry) data...");
    ROS_INFO("Use 'rosservice call /fast2px4_node/reset_initialization' to "
             "manually reset");
  }

private:
  void mavrosOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    currentMavrosPose_.header = msg->header;
    currentMavrosPose_.pose = msg->pose.pose;
    mavrosDataReceived_ = true;

    // 添加MAVROS位姿到滑动窗口
    bool mavrosWindowFull = slidingWindowMavros_.addPose(msg->pose.pose);

    // 检查是否可以初始化
    checkInitialization();
    // ROS_INFO("MAVROS window size: %d", slidingWindowMavros_.size());
  }

  void slamOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    currentSlamPose_.header = msg->header;
    currentSlamPose_.pose = msg->pose.pose;
    slamDataReceived_ = true;

    // 添加SLAM位姿到滑动窗口
    bool slamWindowFull = slidingWindowSlam_.addPose(msg->pose.pose);

    // 检查是否可以初始化
    checkInitialization();

    // 如果已初始化，立即转换并发布
    if (initialized_) {
      publishTransformedPose();
    }
  }

  void checkInitialization() {
    // 统一的初始化检查逻辑
    if (!initialized_ && mavrosDataReceived_ && slamDataReceived_ &&
        slidingWindowMavros_.size() >= 8 && slidingWindowSlam_.size() >= 8) {

      ROS_INFO("Initialization conditions met:");
      ROS_INFO("  MAVROS window size: %d", slidingWindowMavros_.size());
      ROS_INFO("  SLAM window size: %d", slidingWindowSlam_.size());

      initializeStaticTransform();
    } else if (!initialized_) {
      // 打印当前状态，帮助调试
      static int debug_count = 0;
      if (++debug_count % 50 == 0) { // 每50次打印一次状态
        ROS_INFO("Waiting for initialization: MAVROS=%s(%d), SLAM=%s(%d)",
                 mavrosDataReceived_ ? "OK" : "NO", slidingWindowMavros_.size(),
                 slamDataReceived_ ? "OK" : "NO", slidingWindowSlam_.size());
      }
    }
  }

  void initTimerCallback(const ros::TimerEvent &event) {
    if (!initialized_) {
      // 定期检查初始化条件
      checkInitialization();
    } else {
      // 初始化完成后停止定时器
      init_timer_.stop();
    }
  }

  bool resetCallback(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &res) {
    ROS_WARN("Resetting initialization manually...");

    initialized_ = false;
    mavrosDataReceived_ = false;
    slamDataReceived_ = false;
    slidingWindowMavros_.clear();
    slidingWindowSlam_.clear();

    // 重新启动定时器
    init_timer_.stop();
    init_timer_ = nh_.createTimer(
        ros::Duration(1.0), &SlamToMavrosConverter::initTimerCallback, this);

    ROS_INFO("Initialization reset complete. Waiting for new data...");
    return true;
  }

  void initializeStaticTransform() {
    if (!mavrosDataReceived_ || !slamDataReceived_) {
      ROS_WARN("Cannot initialize: missing MAVROS or SLAM data");
      return;
    }

    // 获取滑动窗口的平均位姿
    geometry_msgs::Pose avgMavrosPose = slidingWindowMavros_.getAveragePose();
    geometry_msgs::Pose avgSlamPose = slidingWindowSlam_.getAveragePose();

    // 计算两个坐标系之间的变换
    tf2::Vector3 mavros_pos(avgMavrosPose.position.x, avgMavrosPose.position.y,
                            avgMavrosPose.position.z);

    tf2::Vector3 slam_pos(avgSlamPose.position.x, avgSlamPose.position.y,
                          avgSlamPose.position.z);

    tf2::Quaternion mavros_quat, slam_quat;
    tf2::fromMsg(avgMavrosPose.orientation, mavros_quat);
    tf2::fromMsg(avgSlamPose.orientation, slam_quat);

    // 计算相对旋转：q_transform = q_mavros * q_slam^(-1)
    tf2::Quaternion slam_quat_inv = slam_quat.inverse();
    tf2::Quaternion q_transform = mavros_quat * slam_quat_inv;

    // 计算相对平移：考虑旋转后的位置差
    tf2::Vector3 slam_pos_rotated = tf2::quatRotate(q_transform, slam_pos);
    tf2::Vector3 translation_offset = mavros_pos - slam_pos_rotated;

    // 创建静态变换：从slam_origin到world（ENU）
    geometry_msgs::TransformStamped static_transform;
    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = "world";      // MAVROS ENU坐标系
    static_transform.child_frame_id = "slam_origin"; // SLAM坐标系原点

    // 设置变换
    static_transform.transform.translation.x = translation_offset.x();
    static_transform.transform.translation.y = translation_offset.y();
    static_transform.transform.translation.z = translation_offset.z();
    static_transform.transform.rotation = tf2::toMsg(q_transform);

    // 发布静态变换
    static_tf_broadcaster_->sendTransform(static_transform);

    initialized_ = true;

    // 打印调试信息
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_transform).getRPY(roll, pitch, yaw);

    ROS_INFO("==== Static Transform Initialized ====");
    ROS_INFO("MAVROS avg pose: [%.3f, %.3f, %.3f]", avgMavrosPose.position.x,
             avgMavrosPose.position.y, avgMavrosPose.position.z);
    ROS_INFO("SLAM avg pose: [%.3f, %.3f, %.3f]", avgSlamPose.position.x,
             avgSlamPose.position.y, avgSlamPose.position.z);
    ROS_INFO("Transform rotation (RPY): [%.3f, %.3f, %.3f] deg",
             roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
    ROS_INFO("Transform translation: [%.3f, %.3f, %.3f]",
             translation_offset.x(), translation_offset.y(),
             translation_offset.z());
    ROS_INFO("Published static transform: world -> slam_origin");
    ROS_INFO("======================================");
  }

  void publishTransformedPose() {
    if (!slamDataReceived_) {
      return;
    }

    try {
      // 创建SLAM位姿在slam_origin坐标系下的PoseStamped
      geometry_msgs::PoseStamped slam_pose_in_origin;
      slam_pose_in_origin.header.stamp = currentSlamPose_.header.stamp;
      slam_pose_in_origin.header.frame_id = "slam_origin";
      slam_pose_in_origin.pose = currentSlamPose_.pose;

      // 使用tf2将位姿从slam_origin变换到world坐标系
      geometry_msgs::PoseStamped pose_in_world;
      tf_buffer_.transform(slam_pose_in_origin, pose_in_world, "world",
                           ros::Duration(0.1));

      // 构建并发布vision pose
      geometry_msgs::PoseStamped vision_pose;
      vision_pose.header.stamp = currentSlamPose_.header.stamp;
      vision_pose.header.frame_id = "world";
      vision_pose.pose = pose_in_world.pose;

      vision_pose_pub_.publish(vision_pose);

      // 打印调试信息（降低频率）
      static int count = 0;
      if (++count % 50 == 0) { // 每50次打印一次
        ROS_INFO(
            "SLAM->ENU: [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
            currentSlamPose_.pose.position.x, currentSlamPose_.pose.position.y,
            currentSlamPose_.pose.position.z, pose_in_world.pose.position.x,
            pose_in_world.pose.position.y, pose_in_world.pose.position.z);
      }

    } catch (tf2::TransformException &ex) {
      ROS_WARN("Transform failed: %s", ex.what());
    }
  }

  // ROS相关
  ros::NodeHandle &nh_;
  ros::Subscriber mavros_odom_sub_;
  ros::Subscriber slam_odom_sub_;
  ros::Publisher vision_pose_pub_;
  ros::Timer init_timer_;            // 初始化检查定时器
  ros::ServiceServer reset_service_; // 重置服务
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // TF相关
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 数据处理
  SlidingWindowPose slidingWindowMavros_;
  SlidingWindowPose slidingWindowSlam_;
  geometry_msgs::PoseStamped currentSlamPose_;
  geometry_msgs::PoseStamped currentMavrosPose_;

  // 状态标志
  bool initialized_;
  bool slamDataReceived_;
  bool mavrosDataReceived_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_to_mavros_converter");
  ros::NodeHandle nh("~");

  try {
    SlamToMavrosConverter converter(nh);

    ROS_INFO("Starting SLAM to MAVROS pose conversion...");
    ROS_INFO("Node name: %s", ros::this_node::getName().c_str());
    ros::spin();

  } catch (const std::exception &e) {
    ROS_ERROR("Error: %s", e.what());
    return -1;
  }

  return 0;
}