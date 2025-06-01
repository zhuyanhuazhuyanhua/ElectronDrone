#!/usr/bin/env python3
import cv2
import rospy
from std_srvs.srv import Trigger, TriggerResponse

class RedDetectionService:
    def __init__(self):
        rospy.init_node("analyse_red_server")
        rospy.Service("analyse_red", Trigger, self.handle_red_detection)
        rospy.loginfo("红色检测服务已启动...")
        rospy.spin()

    def handle_red_detection(self, req):
        # 每次服务调用时重新打开摄像头
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            rospy.logerr("无法打开摄像头")
            return TriggerResponse(success=False, message="无法打开摄像头")

        # 读取最新的一帧
        ret, frame = cap.read()
        cap.release()

        if not ret:
            rospy.logerr("无法读取图像")
            return TriggerResponse(success=False, message="无法读取图像")

        # 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定义红色的 HSV 范围（红色在 HSV 空间中不连续，需要两个范围）
        lower_red1 = (0, 100, 100)
        upper_red1 = (10, 255, 255)
        lower_red2 = (160, 100, 100)
        upper_red2 = (180, 255, 255)

        # 创建掩码
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 对掩码进行形态学操作，去除噪声
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            rospy.loginfo("未检测到红色区域")
            return TriggerResponse(success=False, message="未检测到红色区域")

        # 查找最大的红色区域
        max_contour = max(contours, key=cv2.contourArea)
        red_area = cv2.contourArea(max_contour)

        # 可以根据需求调整最小红色区域阈值
        min_red_area = 100
        if red_area < min_red_area:
            rospy.loginfo("检测到的红色区域过小")
            return TriggerResponse(success=False, message="检测到的红色区域过小")

        rospy.loginfo(f"检测到红色区域，面积: {red_area}")
        return TriggerResponse(success=True, message=f"检测到红色区域，面积: {red_area}")

if __name__ == "__main__":
    try:
        red_service = RedDetectionService()
    except rospy.ROSInterruptException:
        pass