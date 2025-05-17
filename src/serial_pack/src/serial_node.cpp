#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

serial::Serial ser;  // 创建串口对象

const size_t frame_length = 64;  // 帧长度

void serialSendCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Writing to serial port: %s", msg->data.c_str());

  // 将消息写入串口
  size_t bytes_written = ser.write(msg->data);  // 将消息写入串口

  if (bytes_written == msg->data.length()) {
    ROS_INFO("Successfully written to serial port.");
  } else {
    ROS_ERROR("Failed to write all data to serial port.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_node");  // 初始化ROS节点
  ros::NodeHandle nh;

  // 获取参数（串口配置）
  std::string port;
  int baud_rate;
  nh.param<std::string>("port", port, "/dev/ttyUSB0");
  nh.param<int>("baud_rate", baud_rate, 115200);

  ros::Publisher serial_pub =
      nh.advertise<std_msgs::String>("serial_read", 1000);
  ros::Subscriber serial_sub = nh.subscribe<std_msgs::String>(
      "/serial_write", 10, serialSendCallback);  // 订阅串口写入消息

  try {
    // 配置串口
    ser.setPort(port);
    ser.setBaudrate(baud_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();  // 打开串口
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port " << port);
    return -1;
  }

  if (!ser.isOpen()) {
    ROS_ERROR_STREAM("Failed to open serial port");
    return -1;
  }

  ROS_INFO_STREAM("Serial port " << port << " initialized at " << baud_rate
                                 << " baud");

  ros::Rate loop_rate(100);  // 设置循环频率为100Hz

  while (ros::ok()) {
    // 如果串口缓冲区中至少有一个完整的数据帧
    if (ser.available() >= frame_length) {
      // 创建一个缓冲区来存储一帧完整的数据
      std::vector<uint8_t> frame_data(frame_length * 2);

      // 读取完整的一帧数据
      ser.read(frame_data.data(), frame_length * 2);

      // 将读取的数据转换为字符串以发布
      std_msgs::String msg;
      msg.data = std::string(frame_data.begin(), frame_data.end());

      // 打印并发布数据
      ROS_INFO_STREAM("Read complete frame from serial: " << msg.data);
      //   serial_pub.publish(msg);
    }

    ros::spinOnce();    // 处理回调
    loop_rate.sleep();  // 按照设定的频率休眠
  }

  ser.close();  // 关闭串口
  return 0;
}
