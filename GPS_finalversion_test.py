import cv2
import numpy as np
import torch
import utm
from scipy.spatial.transform import Rotation as R
from math import radians, sin, cos, sqrt, atan2
import time

# ========== 相机内参 ==========
K = np.array([[1739.13,    0, 728],   # fx, 0, cx
              [   0,   1739.13, 544], # 0, fy, cy
              [   0,       0,   1]])  # 0, 0, 1




# ========== 加载 YOLOv5 模型 ==========
yolo_model = torch.hub.load('ultralytics/yolov5', 'custom',
                             path='E:\\pyproject\\yolov5-6.0\\runs\\train\\exp13\\weights\\best.pt')

# ========== 假设已知无人机状态 ==========
# 请根据实际情况设置
drone_lat = 51.4235487       # 纬度
drone_lon = -2.6710046     # 经度
drone_alt = 100.0         # 高度（单位：米）
roll = 0.0                # 横滚角（单位：弧度）
pitch = 0.0               # 俯仰角（单位：弧度）
angle = 180
yaw = -radians(angle)         # 偏航角（角度转换为弧度，例如180度）

# ========== 像素坐标 → GPS ==========
import numpy as np
import utm

# 相机视场在 40m 高度时的实际尺寸
FOV_X = 33.5  # 水平宽度（米）
FOV_Y = 25.0  # 垂直高度（米）

# 图像尺寸
IMG_W = 1456  # 图像宽度（像素）
IMG_H = 1088  # 图像高度（像素）

def pixel_to_gps(u, v, drone_lat, drone_lon, drone_alt, roll, pitch, yaw):
    """
    输入：
        u, v: 像素坐标
        drone_lat, drone_lon: 无人机的 GPS 坐标
        drone_alt: 无人机的高度
        roll, pitch, yaw: 无人机的姿态角（弧度）

    输出：
        tgt_lat, tgt_lon: 目标点的 GPS 坐标
    """

    # 计算像素点相对于图像中心的比例偏移
    x_ratio = (u - IMG_W / 2) / (IMG_W / 2)  # 归一化到 [-1, 1]
    y_ratio = (v - IMG_H / 2) / (IMG_H / 2)  # 归一化到 [-1, 1]

    # 将像素比例映射到实际地面距离
    dx = x_ratio * (FOV_X / 2)  # 水平方向偏移（米）
    dy = y_ratio * (FOV_Y / 2)  # 垂直方向偏移（米） Y轴镜像时无负号

    # 旋转偏移量，使其与无人机朝向对齐
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw)],
                      [np.sin(yaw),  np.cos(yaw)]])
    offset = R_yaw @ np.array([dx, dy])  # 旋转后的 (dx, dy)

    # 将无人机 GPS 坐标转换为 UTM 坐标
    utm_x, utm_y, zone, letter = utm.from_latlon(drone_lat, drone_lon)

    # 计算目标点的 UTM 坐标
    tgt_x = utm_x + offset[0]
    tgt_y = utm_y + offset[1]

    # 转回 GPS 坐标
    tgt_lat, tgt_lon = utm.to_latlon(tgt_x, tgt_y, zone, letter)

    return tgt_lat, tgt_lon


# ========== 绘制检测框 ==========
def draw_detections(frame, detections):
    for det in detections:
        x1, y1, x2, y2, conf, cls = det
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        if int(cls) == 0:
            label, color = "Zebra", (255, 255, 0)
        elif int(cls) == 1:
            label, color = "Rhino", (0, 0, 255)
        elif int(cls) == 2:
            label, color = "Elephant", (0, 255, 0)
        else:
            label, color = f"Class {int(cls)}", (255, 255, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame

# ========== 分析图像 ==========
def analyze_frame(frame):
    results = yolo_model(frame)
    detections = results.xyxy[0].cpu().numpy()

    counts = {'zebra': 0, 'rhino': 0, 'elephant': 0}
    gps_locations = {'zebra': [], 'rhino': [], 'elephant': []}
    object_positions = []  # 用于存放特定目标（例如犀牛）的 GPS 坐标

    for det in detections:
        x1, y1, x2, y2, conf, cls = det
        # 取检测框中心点
        u = (x1 + x2) / 2
        v = (y1 + y2) / 2
        gps_lat, gps_lon = pixel_to_gps(u, v, drone_lat, drone_lon, drone_alt, roll, pitch, yaw)

        if int(cls) == 0:
            counts['zebra'] += 1
            gps_locations['zebra'].append((gps_lat, gps_lon))
        elif int(cls) == 1:
            counts['rhino'] += 1
            gps_locations['rhino'].append((gps_lat, gps_lon))
            object_positions.append((gps_lat, gps_lon))
        elif int(cls) == 2:
            counts['elephant'] += 1
            gps_locations['elephant'].append((gps_lat, gps_lon))

    return counts, gps_locations, object_positions, detections

# ========== 主程序 ========== 
if __name__ == '__main__':
    # 读取指定图片
    image_path = r"E:\pyproject\yolov5-6.0\123.jpg"
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取图像文件: {image_path}")
        exit(1)

    # 保存原始图像（可选）
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(f"raw_image_{timestamp}.jpg", frame.copy())

    # 目标检测与定位
    counts, gps_data, object_positions, detections = analyze_frame(frame)

    # 保存带检测框的图像
    annotated = draw_detections(frame.copy(), detections)
    cv2.imwrite(f"detected_image_{timestamp}.jpg", annotated)

    # 输出检测结果
    print("检测结果：", counts)
    print("斑马：", gps_data['zebra'])
    print("犀牛：", gps_data['rhino'])
    print("大象：", gps_data['elephant'])

    # 左上角像素
    lat1, lon1 = pixel_to_gps(0, 0, drone_lat, drone_lon, drone_alt, roll, pitch, yaw)

    # 右下角像素
    lat2, lon2 = pixel_to_gps(1456, 1088, drone_lat, drone_lon, drone_alt, roll, pitch, yaw)

    print(f"(0, 0) 像素 -> 经纬度:   {lat1:.6f}, {lon1:.6f}")
    print(f"(1456, 1088) 像素 -> 经纬度: {lat2:.6f}, {lon2:.6f}")

