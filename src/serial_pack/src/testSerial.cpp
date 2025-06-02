#include <ros/ros.h>
#include <serial/serial.h>
#include <unistd.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "debug_test");

  try {
    ROS_INFO("=== Serial Debug Test ===");

    // Create serial object with parameters
    serial::Serial ser("/dev/ttyACM0", 115200,
                       serial::Timeout::simpleTimeout(1000));

    ROS_INFO("Port: %s", ser.getPort().c_str());
    ROS_INFO("Baudrate: %d", ser.getBaudrate());

    // Open serial port (already created with parameters)
    ROS_INFO("Opening serial port...");
    if (!ser.isOpen()) {
      ser.open();
    }

    ROS_INFO("Serial port is open: %s", ser.isOpen() ? "YES" : "NO");

    if (!ser.isOpen()) {
      ROS_ERROR("Failed to open serial port!");
      return 1;
    }

    // Wait for stabilization
    ROS_INFO("Waiting for port stabilization...");
    usleep(200000); // 200ms

    // Check initial status
    ROS_INFO("Available bytes: %zu", ser.available());

    // Prepare data
    std::vector<uint8_t> data = {0x02, 0x01, 0x01};
    ROS_INFO("Preparing to send %zu bytes: 0x02 0x01 0x00", data.size());

    // Send data
    ROS_INFO("Sending data...");
    size_t sent = ser.write(data);
    ROS_INFO("write() returned: %zu bytes", sent);

    if (sent == 0) {
      ROS_ERROR("Send failed! Returned 0 bytes");
    } else if (sent != data.size()) {
      ROS_WARN("Partial send: expected %zu, actual %zu", data.size(), sent);
    } else {
      ROS_INFO("Send successful!");
    }

    // Flush buffer
    ROS_INFO("Flushing buffer...");
    ser.flush();

    // Check status after flush
    ROS_INFO("Available bytes after flush: %zu", ser.available());
    ROS_INFO("Serial port still open: %s", ser.isOpen() ? "YES" : "NO");

    // Wait for response
    usleep(100000); // 100ms
    if (ser.available() > 0) {
      ROS_INFO("Received %zu bytes response", ser.available());
    } else {
      ROS_INFO("No response");
    }

    // Close serial port
    ser.close();
    ROS_INFO("Serial port closed");

  } catch (serial::IOException &e) {
    ROS_ERROR("Serial IO exception: %s", e.what());
    return 1;
  } catch (std::exception &e) {
    ROS_ERROR("Other exception: %s", e.what());
    return 1;
  }

  ROS_INFO("Test completed");
  return 0;
}