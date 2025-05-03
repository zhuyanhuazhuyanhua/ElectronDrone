
# 🚁 无人机坐标系转换与 Offboard 控制 Demo

本项目实现了从 `world` 坐标系到 `ENU（East-North-Up）` 坐标系的转换，并使用 MAVROS 将目标位置发布给 PX4 飞控系统，实现对无人机的 Offboard 模式控制。

---

## 🧭 技术路线概述

1. **坐标系转换**：
   - 使用 `Eigen` 库进行四元数和旋转矩阵计算。
   - 将 `world` 坐标系下的目标点（如 LiDAR 坐标）通过初始偏航角（yaw）旋转至 `ENU` 坐标系。
   - 利用滑动窗口平均算法平滑初始 yaw 值，提高稳定性。

2. **ROS 节点功能**：
   - 订阅无人机当前状态（`mavros/state`）和位置信息（`mavros/local_position/odom`）。
   - 发布目标位置至 `mavros/setpoint_position/local`，用于 Offboard 控制。
   - 提供解锁（Arm）和切换 Offboard 模式的服务调用。

3. **Offboard 控制流程**：
   - 初始化 ROS 节点并等待连接 MAVROS。
   - 设置多个目标位置，依次发送并判断是否到达。
   - 自动切换 Offboard 模式并解锁无人机。
   - 实现循环控制，直到所有目标位置完成。

---

## 📦 依赖库

- [ROS (Robot Operating System)](https://www.ros.org/)
- [MAVROS](https://github.com/mavlink/mavros)
- [Eigen](https://eigen.tuxfamily.org/)
- [geometry_msgs](http://wiki.ros.org/geometry_msgs)
- [nav_msgs](http://wiki.ros.org/nav_msgs)
- [mavros_msgs](http://wiki.ros.org/mavros_msgs)

---

## 🛠️ 编译与运行

### 1. 创建工作空间（若未创建）

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <your_repo_url>
```

### 2. 安装依赖

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译

```bash
catkin build
source devel/setup.bash
```

### 4. 运行节点

确保 PX4 已启动并连接 MAVROS：

```bash
roslaunch mavros px4.launch
```

在另一个终端运行本节点：

```bash
rosrun offboard_run demo_node
```

---

## 📌 注意事项

- 确保 PX4 在 QGroundControl 或其他地面站中已配置为支持 Offboard 模式。
- Offboard 命令必须以高于 2Hz 的频率发布，否则 PX4 会自动退出 Offboard 模式。
- 若使用 Gazebo 仿真，请确保 `/mavros/local_position/odom` 有正确数据输出。

---

## 📄 示例代码结构

- `SlidingWindowAverage` 类：用于平滑初始 yaw 角。
- `fromQuaternion2yaw()`：从四元数提取 yaw 角。
- `is_close()`：判断当前位置是否接近目标位置。
- 主函数逻辑：
  - 初始化 ROS 及相关订阅/发布者。
  - 设置目标点队列。
  - 判断连接状态、切换模式、解锁无人机。
  - 循环发布目标点并更新状态。

---

## 📎 相关话题

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/mavros/state` | `mavros_msgs/State` | 获取无人机当前状态 |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | 获取本地位置信息 |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 发布目标位置 |

---

## 📚 参考资料

- [MAVROS Wiki](http://wiki.ros.org/mavros)
- [PX4 Offboard Mode Documentation](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [Eigen Quick Reference](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html)

---


