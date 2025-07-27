#!/usr/bin/env python3
import torch
from ultralytics import YOLO
import cv2
import time
import logging

# 配置日志记录
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('inference_log.txt'),
        logging.StreamHandler()
    ]
)

# 检查 CUDA 是否可用
if not torch.cuda.is_available():
    logging.error("CUDA 不可用，将使用 CPU 进行推理")
    device_id = 'cpu'
else:
    device_id = 'cuda:0'  # 使用第一块 GPU
    logging.info(f"使用 GPU: {torch.cuda.get_device_name(0)}")

# 加载模型并指定设备
model = YOLO('/home/bupt/fly_ws/yolov8n.pt')
model.to(device_id)  # 将模型移动到指定设备

# 打印模型设备信息
logging.info(f"模型运行设备: {next(model.model.parameters()).device}")

# 0 表示默认摄像头；如果是 RTSP/文件，把 0 换成 URL 或路径
cap = cv2.VideoCapture(0)

# 设置摄像头分辨率（可选，根据需要调整）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

total_inference_time = 0
frame_count = 0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.error("读取帧失败，退出循环")
            break

        # 记录推理开始时间
        start_time = time.time()
        
        # 在推理时指定设备
        results = model(frame, verbose=False, device=device_id)  
        
        # 记录推理结束时间
        end_time = time.time()

        # 计算本次推理耗时
        inference_time = end_time - start_time
        total_inference_time += inference_time
        frame_count += 1

        # 每10帧记录一次推理时间，避免日志过多
        if frame_count % 10 == 0:
            logging.info(f"帧 {frame_count} 推理耗时: {inference_time:.4f} 秒")

        # 计算检测框中心点平均值
        total_cx = 0
        total_cy = 0
        box_count = 0
        for result in results:
            boxes = result.boxes.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].astype(int)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                total_cx += cx
                total_cy += cy
                box_count += 1

        if box_count > 0:
            avg_cx = total_cx / box_count
            avg_cy = total_cy / box_count
            logging.info(f"帧 {frame_count} 检测框中心点平均值: cx = {avg_cx:.2f}, cy = {avg_cy:.2f}")
        else:
            logging.info(f"帧 {frame_count} 未检测到目标")

        annotated = results[0].plot()  # 画框
        cv2.imshow('YOLOv8 Real-time', annotated)
        
        if cv2.waitKey(1) & 0xFF == 27:  # ESC 退出
            break

except KeyboardInterrupt:
    logging.info("用户中断程序")

finally:
    if frame_count > 0:
        # 计算平均推理时间
        average_inference_time = total_inference_time / frame_count
        fps = 1.0 / average_inference_time if average_inference_time > 0 else 0
        logging.info(f"处理帧数: {frame_count}")
        logging.info(f"平均推理耗时: {average_inference_time:.4f} 秒")
        logging.info(f"平均 FPS: {fps:.2f}")

    cap.release()
    cv2.destroyAllWindows()
