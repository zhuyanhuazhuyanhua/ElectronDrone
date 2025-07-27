#!/usr/bin/env python3
from ultralytics import YOLO
# import torch

model = YOLO("yolov8n.pt")  # Load a pretrained YOLOv8 model
model.to('cuda')


for _ in range(10):
    # Run inference on an image
    results = model("https://ultralytics.com/images/bus.jpg")

    # Print results
    # print(results)

# # Load a YOLO11n PyTorch model
# model = YOLO("/home/bupt/yolo11n.pt")

# # Export the model to TensorRT
# model.export(format="engine")  # creates 'yolo11n.engine'

# # Load the exported TensorRT model
# trt_model = YOLO("yolo11n.engine")

# # Run inference
# results = trt_model("https://ultralytics.com/images/bus.jpg")