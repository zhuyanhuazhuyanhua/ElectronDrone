#include <serial/serial.h>

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "serial_pack/setIO.h"

const static uint8_t CMD_SET_SINGLE_IO = 0x01;  // 设置IO命令
class IO {
 public:
  //  TODO输入模式没做
  enum MODE {
    INPUT_FLOAT = 0,        //浮空输入
    INPUT_LOW = 1,          // 下拉输入
    INPUT_HIGH = 2,         // 上拉输入
    OUTPUT_OPEN_DRAIN = 3,  // 开漏输出
    OUTPUT_PUSH_PULL = 4,   // 推挽输出
  };

  IO(const uint8_t &pin, const MODE &mode, const uint8_t &level = 0) {
    mode_ = mode;
    pin_ = pin;
    level_ = level;
    if (mode_ == OUTPUT_OPEN_DRAIN || mode_ == OUTPUT_PUSH_PULL) {
      ROS_INFO("IO initialized in output mode on pin %d with level %d", pin_,
               level_);
    } else {
      ROS_INFO("IO initialized in input mode on pin %d", pin_);
      if (level) {
        ROS_WARN("Level parameter is ignored in input mode");
      }
    }
  }

  std::array<uint8_t, 3> getIOData() const {
    std::array<uint8_t, 3> data = {pin_, static_cast<uint8_t>(mode_), level_};
    return data;
  }
  uint8_t getIOlevel() const { return level_; }
  MODE getIOmode() const { return mode_; }
  void setIOlevel(const uint8_t &level) { level_ = level; }

 private:
  MODE mode_;
  uint8_t pin_;
  uint8_t level_;  // 仅在输出模式下有效
};

std::vector<IO> io_list;

std::shared_ptr<serial::Serial> ser_ptr;

void IO_cb(const serial_pack::setIO::ConstPtr msg) {
  try {
    uint8_t pin = msg->pin;
    IO::MODE mode = io_list.at(pin).getIOmode();
    if (mode != IO::OUTPUT_PUSH_PULL && mode != IO::OUTPUT_OPEN_DRAIN) {
      ROS_ERROR("Not Output mode.");
      return;
    }
    io_list[pin].setIOlevel(msg->level);

    std::vector<uint8_t> packet;
    packet.push_back(CMD_SET_SINGLE_IO);
    std::array<uint8_t, 3> data = io_list[pin].getIOData();

    packet.insert(packet.end(), data.begin(), data.end());

    ser_ptr->write(packet);

    ROS_INFO("Pin %u set to %s", pin, msg->level ? "HIGH" : "LOW");
  } catch (serial::IOException &e) {
    ROS_ERROR("Serial exception: %s", e.what());
    return;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "io_extra_server");
  ros::NodeHandle nh("~");
  ros::Subscriber io_suber =
      nh.subscribe<serial_pack::setIO>("/setIO", 1, IO_cb);

  for (int i = 0; i < 16; ++i) {
    std::string mode_param = "io" + std::to_string(i) + "_mode";
    std::string level_param = "io" + std::to_string(i) + "_level";

    int mode_val = 4;  // 默认模式为 OUTPUT_PUSH_PULL
    int level_val = 0;

    // 尝试从参数服务器读取参数，若失败则警告
    if (!nh.getParam(mode_param, mode_val)) {
      ROS_WARN("Parameter [%s] not found. Using default OUTPUT_PUSH_PULL.",
               mode_param.c_str());
    }
    if (!nh.getParam(level_param, level_val)) {
      ROS_WARN("Parameter [%s] not found. Using default level 0.",
               level_param.c_str());
    }

    if (mode_val < 0 || mode_val > 4) {
      ROS_ERROR("Invalid mode [%d] for pin %d. Skipping this IO.", mode_val, i);
      continue;
    }
    IO::MODE mode = static_cast<IO::MODE>(mode_val);
    IO io(i, mode, level_val ? 1 : 0);  // 将level_val转换为0或1
    ROS_INFO("IO %d initialized with mode %d and level %d", i, mode_val,
             level_val);
    io_list.push_back(io);
  }

  ser_ptr = std::make_shared<serial::Serial>();

  try {
    ser_ptr->setPort(nh.param<std::string>("port", "/dev/ttyUSB0"));
    ser_ptr->setBaudrate(nh.param<int>("baudrate", 115200));
    ser_ptr->open();

    if (!ser_ptr->isOpen()) {
      ROS_ERROR("Failed to open serial port.");
      return 1;
    }
    ROS_INFO("Serial port opened.");

    // 初始化引脚
    std::vector<uint8_t> packet;
    packet.push_back(CMD_SET_SINGLE_IO);

    for (const IO &io : io_list) {
      std::array<uint8_t, 3> data = io.getIOData();

      packet.insert(packet.end(), data.begin(), data.end());
    }
    // 发送
    ser_ptr->write(packet);
    ROS_INFO("IO initialization command sent.");

  } catch (serial::IOException &e) {
    ROS_ERROR("Serial exception: %s", e.what());
    return 1;
  }

  ros::spin();
  return 0;
}