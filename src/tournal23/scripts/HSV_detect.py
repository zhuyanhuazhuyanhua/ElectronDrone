#!/usr/bin/env python3
import cv2
import numpy as np

def main():
    # 打开摄像头 0；如要指定其他设备，把 0 换成 2 即可
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)   # CAP_V4L2 在 Linux 下更稳定，可省
    if not cap.isOpened():
        print("无法打开摄像头，请检查 /dev/video0 是否存在或是否被占用")
        return

    # 可选：设置分辨率、帧率（如摄像头支持）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    print("按 q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("读取帧失败")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 二值化处理
        _, binary = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)

        # 对二值化图像进行形态学操作，去除噪声
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cX, cY = 320, 240
        if contours:
            # 找到最大的轮廓
            max_contour = max(contours, key=cv2.contourArea)
            
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

        # 在灰度图上绘制最大黑色区域中心点并显示坐标
        cv2.circle(gray, (cX, cY), 5, (255, 0, 0), -1)  # 绘制蓝色实心圆
        cv2.putText(gray, f"({cX}, {cY})", (cX + 10, cY + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)  # 显示坐标

        # 显示处理后的图像
        cv2.imshow("Gray Frame", gray)
        cv2.imshow("BGR", frame)
        cv2.imshow("HSV", hsv)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()