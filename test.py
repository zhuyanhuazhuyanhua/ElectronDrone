#!/usr/bin/env python3
import cv2
import pyzbar.pyzbar as pyzbar

image = cv2.imread('output.jpg')

decodedObjects = pyzbar.decode(image)

for obj in decodedObjects:
    print("Data:", obj.data.decode('utf-8'))