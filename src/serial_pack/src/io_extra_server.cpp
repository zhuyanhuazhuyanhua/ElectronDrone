#include <serial/serial.h>
#include <std_msgs/Int8.h>
#include <sys/types.h>

#include <array>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "serial_pack/setIO.h"

const static uint8_t CMD_INIT_IO = 0x01;       // 设置IO命令
const static uint8_t CMD_SET_IO_SINGLE = 0x02; // 设置IO命令
const static uint8_t CMD_SET_SERVO = 's';      // 设置舵机命令
const static uint8_t CMD_INIT_PWM_GROUP = 0x21;
const static uint8_t CMD_SET_PWM_DUTY = 0x22;          // 设置PWM命令
const static uint8_t CMD_SET_PWM_GROUPE_ENABLE = 0x23; // 设置PWM组使能命令
const static uint8_t CMD_SET_PWM_GLOBAL_ENABLE = 0x24; // 设置全局PWM使能命令

// 辅助函数：将字节数组转换为16进制字符串用于打印
std::string bytesToHexString(const std::vector<uint8_t> &data) {
  std::stringstream ss;
  ss << "0x";
  for (size_t i = 0; i < data.size(); ++i) {
    ss << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<int>(data[i]);
    if (i < data.size() - 1) {
      ss << " 0x";
    }
  }
  return ss.str();
}

class IO {
public:
  enum MODE {
    INPUT_FLOAT = 1,       //浮空输入
    INPUT_LOW = 2,         // 下拉输入
    INPUT_HIGH = 3,        // 上拉输入
    OUTPUT_OPEN_DRAIN = 4, // 开漏输出
    OUTPUT_PUSH_PULL = 5,  // 推挽输出
  };

  IO(std::shared_ptr<serial::Serial> serial_ptr, const uint8_t &pin,
     const MODE &mode, const uint8_t &level = 0) {
    mode_ = mode;
    pin_ = pin;
    level_ = level;
    ser_ptr_ = serial_ptr;

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

  // 初始化IO引脚
  bool initializeIO() {
    if (!ser_ptr_ || !ser_ptr_->isOpen()) {
      ROS_ERROR("Serial port not available for IO pin %d", pin_);
      return false;
    }

    try {
      std::vector<uint8_t> packet;
      packet.push_back(CMD_INIT_IO);

      std::array<uint8_t, 3> data = getIOInitData();
      packet.insert(packet.end(), data.begin(), data.end());

      // 打印发送的16进制数据
      ROS_INFO("Sending IO init data: %s", bytesToHexString(packet).c_str());

      size_t bytes_written = ser_ptr_->write(packet);
      ser_ptr_->flush(); // 强制刷新缓冲区

      ROS_INFO("IO init: wrote %zu bytes", bytes_written);
      ROS_INFO("IO pin %d initialized", pin_);
      return true;
    } catch (serial::IOException &e) {
      ROS_ERROR("Serial exception when initializing IO pin %d: %s", pin_,
                e.what());
      return false;
    }
  }

  // 设置IO输出电平
  bool setLevel(const uint8_t &level) {
    if (mode_ != OUTPUT_PUSH_PULL && mode_ != OUTPUT_OPEN_DRAIN) {
      ROS_ERROR("IO pin %d is not in output mode", pin_);
      return false;
    }

    if (!ser_ptr_ || !ser_ptr_->isOpen()) {
      ROS_ERROR("Serial port not available for IO pin %d", pin_);
      return false;
    }

    level_ = level;

    try {
      std::vector<uint8_t> packet;

      packet.push_back(CMD_SET_IO_SINGLE);
      packet.push_back(static_cast<uint8_t>(pin_));
      packet.push_back(static_cast<uint8_t>(level));

      // 打印发送的16进制数据
      ROS_INFO("Sending IO setLevel data: %s",
               bytesToHexString(packet).c_str());

      size_t bytes_written = ser_ptr_->write(packet);
      ser_ptr_->flush(); // 强制刷新缓冲区

      ROS_INFO("SetLevel: wrote %zu bytes", bytes_written);
      ROS_INFO("Pin %u set to %s", pin_, level ? "HIGH" : "LOW");
      return true;
    } catch (serial::IOException &e) {
      ROS_ERROR("Serial exception when setting IO pin %d: %s", pin_, e.what());
      return false;
    }
  }

  std::array<uint8_t, 3> getIOInitData() const {
    std::array<uint8_t, 3> data = {pin_, static_cast<uint8_t>(mode_), level_};
    return data;
  }

  uint8_t getIOlevel() const { return level_; }
  MODE getIOmode() const { return mode_; }
  uint8_t getPin() const { return pin_; }

private:
  MODE mode_;
  uint8_t pin_;
  uint8_t level_; // 仅在输出模式下有效
  std::shared_ptr<serial::Serial> ser_ptr_;
};

class ServoController {
public:
  ServoController(std::shared_ptr<serial::Serial> serial_ptr = nullptr)
      : ser_ptr_(serial_ptr) {}

  // 设置串口指针
  void setSerialPtr(std::shared_ptr<serial::Serial> serial_ptr) {
    ser_ptr_ = serial_ptr;
  }

  // 设置舵机角度
  bool setAngle(const uint8_t &angle) {
    if (angle > 180) {
      ROS_ERROR("Angle %d is out of range (0-180).", angle);
      return false;
    }

    if (!ser_ptr_ || !ser_ptr_->isOpen()) {
      ROS_ERROR("Serial port not available for servo control");
      return false;
    }

    try {
      std::vector<uint8_t> packet{CMD_SET_SERVO, angle};

      // 打印发送的16进制数据
      ROS_INFO("Sending servo data: %s", bytesToHexString(packet).c_str());

      size_t bytes_written = ser_ptr_->write(packet);
      ser_ptr_->flush(); // 强制刷新缓冲区

      ROS_INFO("Servo: wrote %zu bytes", bytes_written);
      ROS_INFO("Servo set to angle %d", angle);
      return true;
    } catch (serial::IOException &e) {
      ROS_ERROR("Serial exception when controlling servo: %s", e.what());
      return false;
    }
  }

private:
  std::shared_ptr<serial::Serial> ser_ptr_;
};

class PWM {
public:
  // 设置PWM使能状态
  void setCountValue(const u_int16_t &countValue) { count_value_ = countValue; }
  void setPolarityHigh(const bool &polarity) { polarity_ = polarity; }

  u_int16_t getCountValue() const { return count_value_; }
  bool getPolarityHigh() const { return polarity_; }

private:
  bool polarity_ = false;
  u_int16_t count_value_ = 0; // PWM占空比计数
};

class PWMGroups {
public:
  PWMGroups(const u_int16_t &resolution_us, const u_int16_t &cycle_us) {
    if (resolution_us == 0 || resolution_us > 546) {
      ROS_ERROR("Invalid resolution: %d us. Must be between 1 and 546 us.",
                resolution_us);
      return;
    }
    resolution_ = 120 * resolution_us;       // 分辨率，单位us 120是1us
    count_value_ = cycle_us / resolution_us; // PWM计数周期
  }

  void setPWMDuty(const uint8_t &id, const double &duty) {
    if (id > 4 || duty < 0 || duty > 100) {
      ROS_ERROR("Invalid PWM ID or duty cycle: ID=%d, Duty=%.2f", id, duty);
      return;
    }

    u_int32_t count_value =
        static_cast<u_int32_t>((duty / 100.0) * count_value_);

    pwm_groups_[id].setCountValue(count_value);
    ROS_INFO("Set PWM group %d duty cycle to %.2f%% (count value: %d)", id,
             duty, count_value);
  }

  void setPWMEnable(const bool &enable) {
    enable_ = enable;
    ROS_INFO("Set PWM group enable state to %s", enable ? "true" : "false");
  }

  void setPWMPolarity(const uint8_t &id, const bool &polarity) {
    if (id > 4) {
      ROS_ERROR("Invalid PWM ID: %d", id);
      return;
    }

    pwm_groups_[id].setPolarityHigh(polarity);
    ROS_INFO("Set PWM group %d polarity to %s", id, polarity ? "HIGH" : "LOW");
  }

private:
  std::array<PWM, 4> pwm_groups_; // 4路PWM
  u_int16_t resolution_;
  u_int32_t count_value_ = 10000; // PWM计数周期
  bool enable_ = false;           // PWM使能状态

  std::array<uint8_t, 9> getInitCMD(const uint8_t &group_id) const {
    uint8_t Px = 0;
    for (int i = 0; i < 4; ++i) {
      int d = pwm_groups_[i].getPolarityHigh(); // 获取极性
      if (d) {
        Px |= (1 << (i + 4)); // 将极性位设置到Px中
      } else {
        Px |= (1 << i); // 将极性位设置到Px中
      }
    }
    std::array<uint8_t, 9> init_cmd = {
        CMD_INIT_PWM_GROUP,
        group_id,
        static_cast<uint8_t>(resolution_ & 0xff),
        static_cast<uint8_t>((resolution_ >> 8) & 0xff),
        static_cast<uint8_t>(count_value_ & 0xff),
        static_cast<uint8_t>(count_value_ >> 8 & 0xff),
        static_cast<uint8_t>(count_value_ >> 16 & 0xff),
        static_cast<uint8_t>(count_value_ >> 24 & 0xff),
        Px};
    return init_cmd;
  }

  std::array<uint8_t, 18> getDutyCMD(const uint8_t &group_id) const {
    std::array<uint8_t, 18> duty_cmd = {CMD_SET_PWM_DUTY, group_id};
    for (int i = 0; i < 4; ++i) {
      u_int32_t count_value = pwm_groups_[i].getCountValue();
      duty_cmd[2 + i * 4] = static_cast<uint8_t>(count_value & 0xff);
      duty_cmd[3 + i * 4] = static_cast<uint8_t>((count_value >> 8) & 0xff);
      duty_cmd[4 + i * 4] = static_cast<uint8_t>((count_value >> 16) & 0xff);
      duty_cmd[5 + i * 4] = static_cast<uint8_t>((count_value >> 24) & 0xff);
    }
    return duty_cmd;
  }

  std::array<uint8_t, 3> getEnableCMD(const uint8_t &group_id) const {
    uint8_t Pxx = enable_ ? 1 : 0;
    std::array<uint8_t, 3> enable_cmd = {CMD_SET_PWM_GROUPE_ENABLE, group_id,
                                         Pxx};
    return enable_cmd;
  }
};

// 全局变量
std::vector<IO> io_list;
std::shared_ptr<ServoController> servo_controller;
std::shared_ptr<serial::Serial> ser_ptr;

void IO_cb(const serial_pack::setIO::ConstPtr msg) {
  if (msg->pin >= io_list.size()) {
    ROS_ERROR("Invalid pin number: %d", msg->pin);
    return;
  }

  io_list[msg->pin - 1].setLevel(msg->level);
}

void servo_cb(const std_msgs::Int8::ConstPtr msg) {
  if (servo_controller) {
    servo_controller->setAngle(msg->data);
  } else {
    ROS_ERROR("Servo controller not initialized");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "io_extra_server");
  ros::NodeHandle nh("~");

  // 获取串口参数
  std::string port =
      nh.param<std::string>("port", "/dev/ttyACM0"); // 修改默认端口
  int baudrate = nh.param<int>("baudrate", 115200);

  ROS_INFO("Initializing serial port: %s at %d baud", port.c_str(), baudrate);

  try {
    // 使用构造函数直接创建并打开串口
    ser_ptr = std::make_shared<serial::Serial>(
        port, baudrate, serial::Timeout::simpleTimeout(1000));

    if (!ser_ptr->isOpen()) {
      ROS_ERROR("Failed to open serial port: %s", port.c_str());
      return 1;
    }

    ROS_INFO("Serial port opened successfully: %s at %d baud",
             ser_ptr->getPort().c_str(), ser_ptr->getBaudrate());

  } catch (serial::IOException &e) {
    ROS_ERROR("Serial exception: %s", e.what());
    return 1;
  }

  // 初始化舵机控制器
  servo_controller = std::make_shared<ServoController>(ser_ptr);

  // 初始化IO引脚
  for (int i = 1; i <= 16; ++i) {
    std::string mode_param = "io" + std::to_string(i) + "_mode";
    std::string level_param = "io" + std::to_string(i) + "_level";

    int mode_val = 5; // 默认模式为 OUTPUT_PUSH_PULL
    int level_val = 0;

    ROS_INFO("Initializing IO pin %d", i);
    // 尝试从参数服务器读取参数，若失败则警告
    if (!nh.getParam(mode_param, mode_val)) {
      ROS_WARN("Parameter [%s] not found. Using default OUTPUT_PUSH_PULL.",
               mode_param.c_str());
    }
    if (!nh.getParam(level_param, level_val)) {
      ROS_WARN("Parameter [%s] not found. Using default level 0.",
               level_param.c_str());
    }

    if (mode_val < 1 || mode_val > 5) {
      ROS_ERROR("Invalid mode [%d] for pin %d. Skipping this IO.", mode_val, i);
      continue;
    }

    IO::MODE mode = static_cast<IO::MODE>(mode_val);
    IO io(ser_ptr, i, mode, level_val ? 1 : 0);

    // 初始化IO引脚
    if (!io.initializeIO()) {
      ROS_ERROR("Failed to initialize IO pin %d", i);
    }

    io_list.push_back(io);
    ROS_INFO("IO %d initialized with mode %d and level %d", i, mode_val,
             level_val);
  }

  // 订阅话题
  ros::Subscriber io_suber =
      nh.subscribe<serial_pack::setIO>("/setIO", 10, IO_cb);
  ros::Subscriber servo_suber =
      nh.subscribe<std_msgs::Int8>("/set_servo", 1, servo_cb);

  ROS_INFO("IO controller node started successfully");
  ros::spin();
  return 0;
}