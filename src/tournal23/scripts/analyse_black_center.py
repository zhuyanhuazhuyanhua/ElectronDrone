# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# import cv2
# import rospy
# import numpy as np
# from std_srvs.srv import Trigger, TriggerResponse
# # 导入 Odometry 消息类型
# from nav_msgs.msg import Odometry

# # 全局变量，用于存储最新的位置信息
# latest_x = 0
# latest_y = 0

# class RedDetectionService:
#     def __init__(self):
#         rospy.init_node("analyse_black_server")
#         rospy.Service("analyse_black", Trigger, self.handle_red_detection)
#         # 订阅 /mavros/local_position/odom 话题
#         rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
#         rospy.loginfo("黑色检测服务已启动...")
#         rospy.spin()

#     def odom_callback(self, msg):
#         global latest_x, latest_y
#         # 提取 pose.pose.position.x 和 pose.pose.position.y
#         latest_x = msg.pose.pose.position.x
#         latest_y = msg.pose.pose.position.y

#     def handle_red_detection(self, req):
#         # 每次服务调用时重新打开摄像头
#         cap = cv2.VideoCapture(2)

#         if not cap.isOpened():
#             rospy.logerr("无法打开摄像头")
#             return TriggerResponse(success=False, message="无法打开摄像头")

#         # 检测摄像头分辨率
#         width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
#         height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
#         rospy.loginfo(f"摄像头分辨率: 宽度 {width} 像素, 高度 {height} 像素")

#         # 计算图像中点
#         mid_x = width / 2
#         mid_y = height / 2
#         rospy.loginfo(f"图像中点坐标: ({mid_x}, {mid_y})")

#         # 读取最新的一帧
#         ret, frame = cap.read()
#         cap.release()

#         if not ret:
#             rospy.logerr("无法读取图像")
#             return TriggerResponse(success=False, message="无法读取图像")
        

#         # 转换到 HSV 颜色空间
#         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#         # 定义黑色的 HSV 范围（黑色在 HSV 空间中不连续，需要两个范围）
#         lower_red1 = (0, 0, 20)
#         upper_red1 = (180, 43, 70)
#         lower_red2 = (0, 0, 130)
#         upper_red2 = (180, 50, 150)

#         # 创建掩码
#         mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
#         mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#         mask = cv2.bitwise_or(mask1, mask2)

#         # 对掩码进行形态学操作，去除噪声
#         kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
#         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#         mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

#         # 初始化 fire_martix 矩阵
#         fire_martix = (mask == 255).astype(int)

#         # 获取黑色像素的坐标
#         red_pixel_coords = np.argwhere(fire_martix == 1)

#         if red_pixel_coords.size > 0:
#             # 计算 row 和 col 的平均值
#             avg_row, avg_col = np.mean(red_pixel_coords, axis=0)
#             rospy.loginfo(f"黑色像素的平均行坐标: {avg_col}, 平均列坐标: {avg_row}")

#             # 计算差向量
#             diff_x = mid_x - avg_col
#             diff_y = mid_y - avg_row
#             rospy.loginfo(f"图像中点和黑色区域中点的差向量: ({diff_x}, {diff_y})")
#         else:
#             avg_row = 0
#             avg_col = 0
#             rospy.loginfo("未检测到黑色像素，平均行坐标和列坐标均为 0")
#             diff_x = mid_x - avg_col
#             diff_y = mid_y - avg_row
#             rospy.loginfo(f"未检测到黑色区域，差向量: ({diff_x}, {diff_y})")

#         # 查找轮廓
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         if not contours:
#             rospy.loginfo("未检测到黑色区域")
#             return TriggerResponse(success=False, message="未检测到黑色区域")

#         # 查找最大的黑色区域
#         max_contour = max(contours, key=cv2.contourArea)
#         red_area = cv2.contourArea(max_contour)

#         # 可以根据需求调整最小黑色区域阈值
#         min_red_area = 100
#         if red_area < min_red_area:
#             rospy.loginfo("检测到的黑色区域过小")
#             return TriggerResponse(success=False, message="检测到的黑色区域过小")

#         rospy.loginfo(f"检测到黑色区域，面积: {red_area}")

#         # 根据 latest_x 和 latest_y 的值返回不同的结果
#         if latest_x < 150 and latest_y < 150:
#             result = "A"
#         elif 150 <= latest_x <= 200 and 150 <= latest_y <= 200:
#             result = "B"
#         else:
#             result = "C"

#         return TriggerResponse(success=True, message=f"检测到黑色区域，面积: {red_area}, 黑色像素平均行坐标: {avg_row}, 平均列坐标: {avg_col}, 差向量: ({diff_x}, {diff_y}), 无人机位置: X={latest_x}, Y={latest_y}, 结果: {result}")

# if __name__ == "__main__":
#     try:
#         red_service = RedDetectionService()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
# 导入 Odometry 消息类型
from nav_msgs.msg import Odometry

# 全局变量，用于存储最新的位置信息
latest_x = 0
latest_y = 0

class RedDetectionService:
    def __init__(self):
        rospy.init_node("analyse_black_server")
        rospy.Service("analyse_black", Trigger, self.handle_red_detection)
        # 订阅 /mavros/local_position/odom 话题
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_callback)
        rospy.loginfo("黑色检测服务已启动...")
        rospy.spin()

    def odom_callback(self, msg):
        global latest_x, latest_y
        # 提取 pose.pose.position.x 和 pose.pose.position.y
        latest_x = msg.pose.pose.position.x
        latest_y = msg.pose.pose.position.y

    def handle_red_detection(self, req):
        # 每次服务调用时重新打开摄像头
        cap = cv2.VideoCapture(2)

        if not cap.isOpened():
            rospy.logerr("无法打开摄像头")
            return TriggerResponse(success=False, message="无法打开摄像头")

        # 检测摄像头分辨率
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        rospy.loginfo(f"摄像头分辨率: 宽度 {width} 像素, 高度 {height} 像素")

        # 计算图像中点
        mid_x = width / 2
        mid_y = height / 2
        rospy.loginfo(f"图像中点坐标: ({mid_x}, {mid_y})")

        # 读取最新的一帧
        ret, frame = cap.read()
        cap.release()

        if not ret:
            rospy.logerr("无法读取图像")
            return TriggerResponse(success=False, message="无法读取图像")



        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 显示灰度图


        # 二值化处理
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)


        # 对二值化图像进行形态学操作，去除噪声
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)




        # 初始化 black_pixel_coords 矩阵
        black_pixel_coords = np.argwhere(binary == 255)

        if black_pixel_coords.size > 0:
            # 计算 row 和 col 的平均值
            avg_row, avg_col = np.mean(black_pixel_coords, axis=0)
            rospy.loginfo(f"黑色像素的平均列坐标: {avg_row}, 平均行坐标: {avg_col}")

            # 计算差向量
            diff_x = mid_x - avg_col
            diff_y = mid_y - avg_row
            rospy.loginfo(f"图像中点和黑色区域中点的差向量: ({diff_x}, {diff_y})")
        else:
            avg_row = 0
            avg_col = 0
            rospy.loginfo("未检测到黑色像素，平均行坐标和列坐标均为 0")
            diff_x = mid_x - avg_col
            diff_y = mid_y - avg_row
            rospy.loginfo(f"未检测到黑色区域，差向量: ({diff_x}, {diff_y})")

        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            rospy.loginfo("未检测到黑色区域")
            return TriggerResponse(success=False, message="未检测到黑色区域")

        # 查找最大的黑色区域
        max_contour = max(contours, key=cv2.contourArea)
        black_area = cv2.contourArea(max_contour)

        # 可以根据需求调整最小黑色区域阈值
        min_black_area = 100
        if black_area < min_black_area:
            rospy.loginfo("检测到的黑色区域过小")
            return TriggerResponse(success=False, message="检测到的黑色区域过小")

        rospy.loginfo(f"检测到黑色区域，面积: {black_area}")

        # 根据 latest_x 和 latest_y 的值返回不同的结果
        if latest_x < 150 and latest_y < 150:
            result = "A"
        elif 150 <= latest_x <= 200 and 150 <= latest_y <= 200:
            result = "B"
        else:
            result = "C"

        return TriggerResponse(success=True, message=f"检测到黑色区域，面积: {black_area}, 黑色像素平均行坐标: {avg_row}, 平均列坐标: {avg_col}, 差向量: ({diff_x}, {diff_y}), 无人机位置: X={latest_x}, Y={latest_y}, 结果: {result}")

if __name__ == "__main__":
    try:
        red_service = RedDetectionService()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()