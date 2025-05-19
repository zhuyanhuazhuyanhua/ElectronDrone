#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

serial::Serial ser;  // 创建串口对象

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
  ros::Subscriber serial_sub =
      nh.subscribe<std_msgs::String>("/serial_write", 10, serialSendCallback);

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
    if (ser.available() > 0) {
      // 读取所有可用的数据
      std::string data = ser.read(ser.available());

      // 创建并发布消息
      std_msgs::String msg;
      msg.data = data;

      // 打印并发布数据 （文本)
      // ROS_INFO_STREAM("Read data from serial: " << msg.data);

      std::ostringstream hex_stream;
      hex_stream << std::hex << std::setfill('0');
      // 打印16进制数据
      for (unsigned char c : data) {
        hex_stream << std::setw(2) << static_cast<int>(c) << " ";
      }
      ROS_INFO_STREAM("Read complete frame (hex): " << hex_stream.str());

      // serial_pub.publish(msg);
    }

    ros::spinOnce();    // 处理回调
    loop_rate.sleep();  // 按照设定的频率休眠
  }

  ser.close();  // 关闭串口
  return 0;
}