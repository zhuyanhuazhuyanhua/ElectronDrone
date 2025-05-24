#include "uwb_solver.hpp"
#include <deque>
#include <geometry_msgs/Point.h>
#include <iomanip>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>
#include <std_msgs/String.h>
#include <vector>

// Define constants from the original code
#define ANCHOR_LIST_COUNT 16
#define TAG_OUTPUT_DIST 0x0001
#define TAG_OUTPUT_RTLS 0x0002

// Structure to hold the tag data (replacing the global Last_cal_data_hds)
struct TagData {
  uint16_t Cal_Flag;
  uint16_t Dist[ANCHOR_LIST_COUNT]; // cm
  int16_t x;
  int16_t y;
  int16_t z;
};

// ---------------- CRC16 (Modbus) ----------------
uint16_t crc16_modbus(const std::vector<uint8_t> &data) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// Tag data parsing function converted to work with the ROS node
Eigen::Vector4d parseTagData(const std::vector<uint8_t> &buffer) {
  TagData tagData;
  uint16_t output_protocal;
  uint16_t Location_FLAG;
  uint16_t read_idx =
      3; // Starting index as specified in the original code (idx=3)

  // Check if we have enough data to parse
  if (buffer.size() < read_idx + 6) {
    ROS_WARN("Buffer too small for parsing tag header");
    return Eigen::Vector4d::Zero();
  }

  output_protocal = buffer[read_idx + 2] << 8 | buffer[read_idx + 3];
  tagData.Cal_Flag = buffer[read_idx + 4] << 8 | buffer[read_idx + 5];

  read_idx += 6;

  // Parse distance information if available
  if (output_protocal & TAG_OUTPUT_DIST) {
    if (buffer.size() < read_idx + 32) {
      ROS_WARN("Buffer too small for parsing tag distances");
      return Eigen::Vector4d::Zero();
    }

    for (int i = 0; i < ANCHOR_LIST_COUNT; i++) {
      if ((tagData.Cal_Flag >> i) & 0x01) {
        tagData.Dist[i] =
            buffer[i * 2 + read_idx] << 8 | buffer[i * 2 + 1 + read_idx];
        ROS_INFO("Tag distance[%d]: %d", i, tagData.Dist[i]);
      } else {
        tagData.Dist[i] = 0;
      }
    }
    read_idx += 32;
  } else {
    // No distance data available
    for (int i = 0; i < ANCHOR_LIST_COUNT; i++) {
      tagData.Dist[i] = 0;
    }
  }

  if (buffer.size() < read_idx + 2) {
    ROS_WARN("Buffer too small for parsing location flag");
    return Eigen::Vector4d::Zero();
  }

  Location_FLAG = buffer[read_idx] << 8 | buffer[read_idx + 1];
  read_idx += 2;

  // Parse position information if available
  if (output_protocal & TAG_OUTPUT_RTLS) {
    if (buffer.size() < read_idx + 6) {
      ROS_WARN("Buffer too small for parsing tag position");
      return Eigen::Vector4d::Zero();
    }

    if (Location_FLAG != 0) {
      tagData.x = buffer[read_idx] << 8 | buffer[read_idx + 1];
      tagData.y = buffer[read_idx + 2] << 8 | buffer[read_idx + 3];
      tagData.z = buffer[read_idx + 4] << 8 | buffer[read_idx + 5];
    } else {
      tagData.x = 0;
      tagData.y = 0;
      tagData.z = 0;
    }
  } else {
    tagData.x = 0;
    tagData.y = 0;
    tagData.z = 0;
  }

  // // Publish distance data
  // std_msgs::UInt16MultiArray dist_msg;
  // dist_msg.data.resize(ANCHOR_LIST_COUNT);
  // for (int i = 0; i < ANCHOR_LIST_COUNT; i++) {
  //   dist_msg.data[i] = tagData.Dist[i];
  // }
  // dist_pub.publish(dist_msg);
  Eigen::Vector4d distances;
  for (int i = 0; i < 4; i++) {
    if (tagData.Dist[i] != 0) {
      distances(i) = tagData.Dist[i] / 100.0; // Convert cm to meters
    }
  }
  std::cout << "Distances" << distances.transpose() << std::endl;

  ROS_INFO("Tag data parsed: Cal_Flag=0x%04X, Pos=(%d,%d,%d)", tagData.Cal_Flag,
           tagData.x, tagData.y, tagData.z);

  return distances;
}

// ---------------- Main ROS Loop ----------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_reader");
  ros::NodeHandle nh;

  std::vector<Eigen::Vector3d> Anchors = {{0.0, 0.0, 1.605},       // Anchor 1
                                          {3.588, 0.0027, 1.318},  // Anchor 2
                                          {3.370, 6.797, 1.930},   // Anchor 3
                                          {-2.616, 5.569, 1.730}}; // Anchor 4
  uwb_solver uwb_solver(Anchors);

  serial::Serial ser;
  try {
    ser.setPort("/dev/ttyUSB1");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port.");
    return -1;
  }

  if (!ser.isOpen()) {
    ROS_ERROR_STREAM("Serial port not opened.");
    return -1;
  }

  ROS_INFO_STREAM("Serial port opened.");

  std::deque<uint8_t> buffer;
  const size_t MIN_FRAME_SIZE = 4;   // Header(2) + Minimum data(0) + CRC(2)
  const size_t MAX_FRAME_SIZE = 256; // Reasonable maximum size

  ros::Rate loop_rate(100); // 控制循环频率
  while (ros::ok()) {
    if (ser.available() > 0) {
      std::string raw = ser.read(ser.available());

      for (unsigned char c : raw) {
        buffer.push_back(c);
      }

      // 处理缓冲区数据
      while (buffer.size() >= MIN_FRAME_SIZE) {
        // 找帧头
        if (buffer[0] != 0x01 || buffer[1] != 0x03) {
          buffer.pop_front(); // 丢弃非帧头
          continue;
        }

        // 没有长度字段，通过尝试不同的帧长度并验证CRC来断帧
        bool frame_found = false;
        for (size_t frame_len = MIN_FRAME_SIZE;
             frame_len <= std::min(MAX_FRAME_SIZE, buffer.size());
             frame_len++) {

          // 检查当前尝试的帧长度是否有足够的数据
          if (buffer.size() < frame_len) {
            break;
          }

          // 计算除CRC外的帧数据的CRC
          std::vector<uint8_t> frame_data(buffer.begin(),
                                          buffer.begin() + frame_len - 2);
          uint16_t crc_calc = crc16_modbus(frame_data);

          // 提取接收到的CRC
          uint16_t crc_recv =
              buffer[frame_len - 2] | (buffer[frame_len - 1] << 8);

          // 如果CRC匹配，认为找到了有效帧
          if (crc_calc == crc_recv) {
            std::vector<uint8_t> frame(buffer.begin(),
                                       buffer.begin() + frame_len);

            // 打印帧数据（十六进制）
            std::ostringstream hex_stream;
            hex_stream << std::hex << std::setfill('0');
            for (uint8_t byte : frame) {
              hex_stream << std::setw(2) << static_cast<int>(byte) << " ";
            }
            ROS_INFO_STREAM("Valid frame (hex): " << hex_stream.str());

            // 解析并发布标签数据
            Eigen::Vector4d distances = parseTagData(frame);
            Eigen::Vector3d result;
            try {
              result = uwb_solver.solve(distances);
              ROS_INFO_STREAM("UWB Solver result: " << result.transpose());
            } catch (const std::exception &e) {
              ROS_ERROR_STREAM("UWB Solver error: " << e.what());
              continue; // Skip this iteration if there's an error
            }
            // 发布原始帧数据
            std_msgs::String msg;
            msg.data = std::string(frame.begin(), frame.end());

            // 移除已处理帧
            buffer.erase(buffer.begin(), buffer.begin() + frame_len);

            frame_found = true;
            break;
          }
        }

        // 如果尝试了所有可能的长度但没找到有效帧，丢弃第一个字节继续搜索
        if (!frame_found) {
          buffer.pop_front();
        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ser.close();
  return 0;
}