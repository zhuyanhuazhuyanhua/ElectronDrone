#!/usr/bin/env python3
import cv2
import pyzbar.pyzbar as pyzbar
import rospy
from std_srvs.srv import Trigger, TriggerResponse

class QRCodeService:
    def __init__(self):
        rospy.init_node("analyse_qr_server")
        rospy.Service("analyse_qr", Trigger, self.handle_qr_decode)
        rospy.loginfo("二维码解析服务已启动...")
        rospy.spin()

    def handle_qr_decode(self, req):
        # 每次服务调用时重新打开摄像头
        cap = cv2.VideoCapture('/dev/cameraforward')

        if not cap.isOpened():
            rospy.logerr("无法打开摄像头")
            return TriggerResponse(success=False, message="无法打开摄像头")

        # 读取最新的一帧
        ret, frame = cap.read()
        cap.release()

        if not ret:
            rospy.logerr("无法读取图像")
            return TriggerResponse(success=False, message="无法读取图像")

        # 保存当前帧到文件
        # cv2.imwrite("output.jpg", frame)

        # 解析二维码
        barcodes = pyzbar.decode(frame)

        if not barcodes:
            rospy.loginfo("未检测到二维码")
            return TriggerResponse(success=False, message="未检测到二维码")

        # 查找距离中心最近的二维码
        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width / 2, frame_height / 2)
        min_distance = float("inf")
        closest_barcode = None

        for barcode in barcodes:
            points = barcode.polygon
            if len(points) == 4:
                pts = [(point.x, point.y) for point in points]
                qr_center = (
                    sum([p[0] for p in pts]) / 4,
                    sum([p[1] for p in pts]) / 4,
                )
                distance = ((qr_center[0] - frame_center[0]) ** 2 + (qr_center[1] - frame_center[1]) ** 2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
                    closest_barcode = barcode

        # 如果找到最靠近中心的二维码
        if closest_barcode:
            data = closest_barcode.data.decode("utf-8")
            rospy.loginfo(f"检测到居中二维码，内容: {data}")
            return TriggerResponse(success=True, message=data)

        # 如果没有找到有效的二维码
        return TriggerResponse(success=False, message="未找到居中的二维码")

if __name__ == "__main__":
    try:
        qr_service = QRCodeService()
    except rospy.ROSInterruptException:
        pass
