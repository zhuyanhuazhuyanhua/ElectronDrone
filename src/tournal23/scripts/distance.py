import cv2
import numpy as np
import imutils

class DistanceCalculator:
    def __init__(self, knownWidth, fx, fy):
        # 初始化已知物体宽度和相机焦距
        self.knownWidth = knownWidth
        self.fx = fx
        self.fy = fy

    def find_marker(self, image):
        # 在图像中寻找目标物体的轮廓
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(gray, 35, 125)
        cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        c = max(cnts, key=cv2.contourArea)
        return cv2.minAreaRect(c)

    def distance_to_camera(self, perWidth, use_fx=True):
        # 计算物体到摄像头的距离
        focal_length = self.fx if use_fx else self.fy
        return (self.knownWidth * focal_length) / perWidth

    def process_frame(self, frame, use_fx=True):
        try:
            marker = self.find_marker(frame)
            distance = self.distance_to_camera(marker[1][0], use_fx)
            box = cv2.boxPoints(marker)
            box = np.intp(box)
            cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
            # 移除单位转换逻辑
            cv2.putText(frame, "%.2fcm" % distance, 
                        (frame.shape[1] - 200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        2.0, (0, 255, 0), 3)
        except ValueError:
            # 处理没有找到轮廓的情况
            pass
        return frame

# 主程序
if __name__ == "__main__":
    KNOWN_WIDTH = 22 # 设置已知物体宽度（厘米）
    FX = 906.157  # 已知的 fx 焦距
    FY = 902.682  # 已知的 fy 焦距

    calculator = DistanceCalculator(KNOWN_WIDTH, FX, FY)

    # 打开 /dev/video0 摄像头
    cap = cv2.VideoCapture('/dev/cameradown')

    if not cap.isOpened():
        print("无法打开摄像头")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取帧")
            break

        processed_frame = calculator.process_frame(frame, use_fx=True)

        cv2.imshow("Distance", processed_frame)

        # 按 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头并关闭所有窗口
    cap.release()
    cv2.destroyAllWindows()
