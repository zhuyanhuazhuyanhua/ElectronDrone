

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

# 全局变量，用于存储最新的位置信息
latest_x = 0
latest_y = 0

class BlackDetection:
    def __init__(self):
        rospy.init_node("analyse_black_node")
        # 订阅 /mavros/local_position/odom 话题
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
        # 创建一个发布者，发布到 /camera_aiming_center 话题，消息类型为 Point
        self.aiming_center_pub = rospy.Publisher('/camera_aiming_center', Point, queue_size=10)
        rospy.loginfo("黑色检测节点已启动...")
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头")
            return

        # 可选：设置分辨率、帧率（如摄像头支持）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    def odom_callback(self, msg):
        global latest_x, latest_y
        # 提取 pose.pose.position.x 和 pose.pose.position.y
        latest_x = msg.pose.pose.position.x
        latest_y = msg.pose.pose.position.y

    def run_detection(self):
        print("按 q 退出")
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                print("读取帧失败")
                continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 二值化处理
            _, binary = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)

            # 对二值化图像进行形态学操作，去除噪声
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

            # 查找轮廓
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cX, cY = 320, 240
            if contours:
                # 找到最大的轮廓
                max_contour = max(contours, key=cv2.contourArea)
                # 计算最大轮廓的面积
                contour_area = cv2.contourArea(max_contour)
                if contour_area < 10:
                    print("最大轮廓面积小于 10 像素，舍弃该区域，中心点坐标设为 (320, 240)")
                    cX, cY = 320, 240
                    # 跳过后续处理
                    continue
                
                # 创建掩码
                mask = np.zeros_like(gray)
                cv2.drawContours(mask, [max_contour], -1, 255, -1)
                
                # 提取轮廓内的 RGB 像素
                rgb_pixels = frame[mask == 255]
                if len(rgb_pixels) > 0:
                    # 计算 RGB 通道的平均值
                    b_mean, g_mean, r_mean = np.mean(rgb_pixels, axis=0)
                    gb_avg = (g_mean + b_mean) / 2
                    
                    # 判断是否满足条件
                    if abs(gb_avg - r_mean) <= 10:
                        # 计算最大轮廓的矩
                        M = cv2.moments(max_contour)
                        if M["m00"] != 0:
                            # 计算中心点坐标
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            print(f"最大黑色区域中心点坐标: ({cX}, {cY})")
                        else:
                            print("最大黑色区域中心点无法计算，坐标设为 (0, 0)")
                    else:
                        print("因 RGB 条件不满足，舍弃该区域，中心点坐标设为 (0, 0)")
                        cX, cY = 320, 240
                else:
                    print("未提取到有效 RGB 像素，中心点坐标设为 (0, 0)")
                    cX, cY = 320, 240
            else:
                print("未检测到黑色区域，中心点坐标设为 (0, 0)")

            #在灰度图上绘制最大黑色区域中心点并显示坐标
            cv2.circle(gray, (cX, cY), 5, (255, 0, 0), -1)  # 绘制蓝色实心圆
            cv2.putText(gray, f"({cX}, {cY})", (cX + 10, cY + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)  # 显示坐标

            # 显示处理后的图像
            cv2.imshow("Gray Frame", gray)


            # 创建 Point 消息
            aiming_center = Point()
            aiming_center.x = 240-cY
            aiming_center.y = 320-cX
            aiming_center.z = 0  # 二维图像，z 坐标设为 0

            # 发布消息
            self.aiming_center_pub.publish(aiming_center)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        black_detection = BlackDetection()
        black_detection.run_detection()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
