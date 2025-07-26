#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mid360点云识别圆环中心点坐标
基于博客: https://blog.csdn.net/2301_80039308/article/details/148280958

主要功能：
1. 从ROS话题订阅点云数据
2. 实现最小二乘法圆环拟合
3. 实现RANSAC圆环拟合算法
"""

import numpy as np
from scipy.optimize import least_squares
import tkinter as tk
from tkinter import ttk, messagebox
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class CircleDetector:
    def __init__(self):
        self.point_cloud_data = None
        self.fitted_center = None
        self.fitted_radius = None
        self.true_center = None
        self.true_radius = None

    def find_circle_center_least_squares(self, pc_array):
        """使用最小二乘法拟合圆环中心"""
        x = pc_array[:, 0]
        y = pc_array[:, 1]
        z = pc_array[:, 2]
        
        center_guess = np.array([np.mean(x), np.mean(y), np.mean(z)])
        
        def residuals_circle(center):
            dist_squared = (x - center[0])**2 + (y - center[1])**2 + (z - center[2])**2
            mean_dist_squared = np.mean(dist_squared)
            return dist_squared - mean_dist_squared
        
        result = least_squares(residuals_circle, center_guess)
        
        center = result.x
        dist = np.sqrt((x - center[0])**2 + (y - center[1])**2 + (z - center[2])**2)
        radius = np.mean(dist)
        
        return center, radius
    
    def distance_to_circle(self, x, y, a, b, r):
        """计算点到圆的距离"""
        return abs(np.sqrt((x - a)**2 + (y - b)**2) - r)
    
    def fit_circle_ransac(self, points):
        """使用RANSAC算法拟合圆"""
        best_a, best_b, best_r = 0, 0, 0
        max_inliers = 0
        num_iterations = 100
        inlier_threshold = 1.0
        
        points_2d = points[:, :2]
        
        for _ in range(num_iterations):
            if len(points_2d) < 3:
                break
                
            sample_indices = np.random.choice(len(points_2d), 3, replace=False)
            x1, y1 = points_2d[sample_indices[0]]
            x2, y2 = points_2d[sample_indices[1]]
            x3, y3 = points_2d[sample_indices[2]]
            
            try:
                A = np.array([[2 * (x2 - x1), 2 * (y2 - y1)],
                             [2 * (x3 - x1), 2 * (y3 - y1)]])
                B = np.array([x2**2 - x1**2 + y2**2 - y1**2,
                             x3**2 - x1**2 + y3**2 - y1**2])
                
                if np.linalg.det(A) == 0:
                    continue
                    
                center = np.linalg.solve(A, B)
                a, b = center
                r = np.sqrt((x1 - a)**2 + (y1 - b)**2)
                
                inliers = []
                for point in points_2d:
                    x, y = point
                    d = self.distance_to_circle(x, y, a, b, r)
                    if d <= inlier_threshold:
                        inliers.append(point)
                
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_a, best_b, best_r = a, b, r
                    
            except np.linalg.LinAlgError:
                continue
        
        best_z = np.mean(points[:, 2])
        return np.array([best_a, best_b, best_z]), best_r

class CircleDetectorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Mid360点云识别圆环中心点坐标")
        self.root.geometry("600x600")
        
        self.detector = CircleDetector()
        self.setup_ui()
        rospy.init_node('circle_detector', anonymous=True)
        rospy.Subscriber('/cloud_registered', PointCloud2, self.point_cloud_callback)

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        title_label = ttk.Label(main_frame, text="Mid360点云识别圆环中心点坐标", 
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        control_frame = ttk.LabelFrame(main_frame, text="控制面板", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Label(control_frame, text="拟合算法:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.algorithm_var = tk.StringVar(value="最小二乘法")
        algorithm_combo = ttk.Combobox(control_frame, textvariable=self.algorithm_var,
                                     values=["最小二乘法", "RANSAC算法"])
        algorithm_combo.grid(row=1, column=0, sticky=tk.W+tk.E, pady=5)
        
        ttk.Button(control_frame, text="拟合圆环中心", 
                  command=self.fit_circle).grid(row=2, column=0, pady=10, sticky=tk.W+tk.E)
        
        result_frame = ttk.LabelFrame(control_frame, text="拟合结果", padding="10")
        result_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=10)
        
        self.result_text = tk.Text(result_frame, height=15, width=40)
        self.result_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        scrollbar = ttk.Scrollbar(result_frame, orient=tk.VERTICAL, command=self.result_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.result_text.configure(yscrollcommand=scrollbar.set)
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)

    def point_cloud_callback(self, msg):
        """处理接收到的点云数据"""
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            self.detector.point_cloud_data = np.array(points)
            self.result_text.delete(1.0, tk.END)
            self.result_text.insert(tk.END, f"接收到点云数据\n")
            self.result_text.insert(tk.END, f"点数: {len(points)}\n\n")
        except Exception as e:
            messagebox.showerror("错误", f"处理点云数据失败: {str(e)}")

    def fit_circle(self):
        """拟合圆环中心"""
        if self.detector.point_cloud_data is None:
            messagebox.showwarning("警告", "请等待点云数据接收！")
            return
        
        algorithm = self.algorithm_var.get()
        
        try:
            if algorithm == "最小二乘法":
                center, radius = self.detector.find_circle_center_least_squares(
                    self.detector.point_cloud_data)
                method_name = "最小二乘法"
            else:
                center, radius = self.detector.fit_circle_ransac(
                    self.detector.point_cloud_data)
                method_name = "RANSAC算法"
            
            self.detector.fitted_center = center
            self.detector.fitted_radius = radius
            
            self.result_text.insert(tk.END, f"=== {method_name}拟合结果 ===\n")
            self.result_text.insert(tk.END, f"拟合圆心: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]\n")
            self.result_text.insert(tk.END, f"拟合半径: {radius:.3f}\n")
            self.result_text.insert(tk.END, f"拟合效果: {'Very Good' if np.linalg.norm(center - self.detector.true_center) < 0.5 else 'Good' if np.linalg.norm(center - self.detector.true_center) < 1.0 else 'Fair'}\n\n")
            
        except Exception as e:
            messagebox.showerror("错误", f"拟合失败: {str(e)}")

def main():
    """主函数"""
    print("Mid360点云识别圆环中心点坐标系统")
    root = tk.Tk()
    app = CircleDetectorGUI(root)
    import threading
    ros_thread = threading.Thread(target=rospy.spin)
    ros_thread.start()
    root.mainloop()
    rospy.signal_shutdown("GUI closed")

if __name__ == "__main__":
    main()