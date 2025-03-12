from pymavlink import mavutil
import time
import math

import cv2
import numpy as np
import torch
import utm
from scipy.spatial.transform import Rotation as R
from math import radians, sin, cos, sqrt, atan2
import time

connection_str = 'tcp:127.0.0.1:14550'

# 6 个航点坐标（lat, lon），假设高度都设为 30 m
waypoints_phase1 = [
    (51.4233649, -2.6715177),  # 起飞 -> 30m
    (51.4228247, -2.6711047),
    (51.4225788, -2.6694953),
    (51.4231993, -2.6687551),
    (51.4239135, -2.6692244),
    (51.4235487, -2.6710046),  # 到达后停留5秒
]


waypoints_phase3 = [
    (51.4236057, -2.6707265),
    (51.4239486, -2.6690984),
    (51.4232328, -2.6686531),
    (51.4225437, -2.6693290),
    (51.4227477, -2.6710725),
    (51.4233900, -2.6715231),
]

takeoff_alt = 30
acceptance_radius = 2

def upload_first_6_waypoints(master):
    """上传前 7 个航点，并设置起飞和停留逻辑。"""
    count = len(waypoints_phase1)  # 7

    # 发送 MISSION_COUNT
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count
    )

    seq_uploaded = 0
    while seq_uploaded < count:
        # 等待 MISSION_REQUEST(_INT)
        msg = master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True)
        if not msg:
            continue
        print(f"request seq={msg.seq}")
        req = msg.to_dict()
        req_seq = req['seq']

        lat, lon = waypoints_phase1[req_seq]
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        current = 1 if req_seq == 0 else 0
        if req_seq == 0:
            command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            param1 = 0
            param2 = acceptance_radius
            param3 = 0
            param4 = 0
            altitude = takeoff_alt
            autocontinue = 1
        else:
            # 普通航点
            command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            altitude = takeoff_alt
            if req_seq in [5]:
                altitude = 40
            param1 = 0
            param2 = acceptance_radius
            param3 = 0
            param4 = 0
            autocontinue = 1

        # 发送 MISSION_ITEM_INT
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            req_seq,
            frame,
            command,
            current,
            autocontinue,
            param1, param2, param3, param4,
            int(lat * 1e7),
            int(lon * 1e7),
            altitude
        )

        seq_uploaded += 1
        print(f"[Phase1] Uploaded waypoint seq={req_seq}")

    # 等待 MISSION_ACK
    ack_msg = master.recv_match(type=['MISSION_ACK'], blocking=True)
    if ack_msg:
        ack_msg = ack_msg.to_dict()
        if ack_msg.get('type') == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("[Phase1] Mission upload successful (ACK)")
        else:
            print("[Phase1] Mission upload failed: ACK type=", ack_msg.get('type'))


def wait_until_mission_finished(master, last_seq):
    print(f"Waiting until waypoint is reached...")
    while True:
        msg = master.recv_match(blocking=True)
        if not msg:
            continue
        mtype = msg.get_type()
        if mtype == "MISSION_ITEM_REACHED":
            reached_seq = msg.seq
            if reached_seq >= last_seq:
               print(f"Reached the last waypoint ({last_seq}).")
               break

def upload_third_6_waypoints(master):
    count = len(waypoints_phase3)
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count
    )
    seq_uploaded = 0
    while seq_uploaded < count:
        msg = master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True)
        if not msg:
            continue
        print(f"request seq={msg.seq}")
        req = msg.to_dict()
        req_seq = req['seq']

        lat, lon = waypoints_phase3[req_seq]
        # 使用相对高度坐标系
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        current = 1 if req_seq == 0 else 0  # 第一个航点标记为 current

        if req_seq == 0:
            command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            param1 = 0
            param2 = acceptance_radius
            param3 = 0
            param4 = 0
            altitude = takeoff_alt
            autocontinue = 1
        else:
            command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            altitude = takeoff_alt
            param1 = 0
            param2 = acceptance_radius
            param3 = 0
            param4 = 0
            autocontinue = 1

        # 发送 MISSION_ITEM_INT
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            req_seq,
            frame,
            command,
            current,
            autocontinue,
            param1, param2, param3, param4,
            int(lat * 1e7),
            int(lon * 1e7),
            altitude
        )

        seq_uploaded += 1
        print(f"[Phase3] Uploaded waypoint seq={req_seq}")

    # 等待 MISSION_ACK
    ack_msg = master.recv_match(type=['MISSION_ACK'], blocking=True)
    if ack_msg:
        ack_msg = ack_msg.to_dict()
        if ack_msg.get('type') == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("[Phase3] Mission upload successful (ACK)")
        else:
            print("[Phase3] Mission upload failed: ACK type=", ack_msg.get('type'))

# ========== 相机内参 ==========
K = np.array([[1739.13,    0, 728],   # fx, 0, cx
              [   0,   1739.13, 544], # 0, fy, cy
              [   0,       0,   1]])  # 0, 0, 1


# ========== 加载 YOLOv5 模型 ==========
yolo_model = torch.hub.load('ultralytics/yolov5', 'custom',
                             path='/home/company3/pyproject/yolov5-6.0/groupproject/best.pt')

# ========== 假设已知无人机状态 ==========（下面标明）
# 请根据实际情况设置
#drone_lat = 37.7749       # 纬度
#drone_lon = -122.4194     # 经度
#drone_alt = 100.0         # 高度（单位：米）
#roll = 0.0                # 横滚角（单位：弧度）
#pitch = 0.0               # 俯仰角（单位：弧度）
#angle = 45
#yaw = radians(angle)         # 偏航角（角度转换为弧度，例如45度）

# ========== 像素坐标 → GPS ==========

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
    dy = -y_ratio * (FOV_Y / 2)  # 垂直方向偏移（米）

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
def analyze_frame(frame, drone_lat, drone_lon, drone_alt, yaw, polygon):
    results = yolo_model(frame)
    detections = results.xyxy[0].cpu().numpy()

    counts = {'zebra': 0, 'rhino': 0, 'elephant': 0}
    gps_locations = {'zebra': [], 'rhino': [], 'elephant': []}
    object_positions = []  # 用于存放特定目标（例如犀牛）的 GPS 坐标

    for det in detections:
        x1, y1, x2, y2, conf, cls = det
        u = (x1 + x2) / 2
        v = (y1 + y2) / 2
        roll = 0
        pitch = 0
        gps_lat, gps_lon = pixel_to_gps(u, v, drone_lat, drone_lon, drone_alt, roll, pitch, yaw)
        
        # 判断是否在指定区域内
        if not point_in_polygon((gps_lat, gps_lon), polygon):
            continue  # 不在区域内则跳过处理

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


#多边形顶点判断
def point_in_polygon(point, polygon):
    """
    判断一个点是否在多边形内
    :param point: (lat, lon)
    :param polygon: [(lat1, lon1), (lat2, lon2), ...] 多边形顶点列表，要求顺序正确（顺时针或逆时针）
    :return: True 如果点在多边形内，否则 False
    """
    lat, lon = point
    # 为了使用标准的射线法，先将 (lat, lon) 转换为 (x, y)，其中 x 为经度，y 为纬度
    x, y = lon, lat
    # 同样转换多边形的点
    poly = [(p[1], p[0]) for p in polygon]
    
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        # 判断当前点与前一个点构成的线段是否跨过射线
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

def main():
    master = mavutil.mavlink_connection(connection_str)
    master.wait_heartbeat()
    print("Heartbeat OK - system %u component %u" % (master.target_system, master.target_component))

    mode = 'GUIDED'
    mode_id = master.mode_mapping().get(mode)

    if mode_id is not None:
        master.set_mode(mode_id)
        print(f"switched to  {mode} ")
    else:
        print("mode switch fail")
    time.sleep(5)

    master.arducopter_arm()

    target_altitude = 30  # 目标起飞高度

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # 确认
        0, 0, 0, 0,  # 其他参数（通常不用）
        0, 0, target_altitude  # 纬度, 经度, 高度
    )


    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # 转换成米
            print(f"altitude: {current_alt}m")
            if current_alt >= target_altitude - 1:  # 允许 1m 误差
                print("reached target altitude")
                break
    # -----------------------------
    # 阶段1：上传前6个航点 + 执行
    master.mav.mission_clear_all_send(master.target_system, master.target_component)

    upload_first_6_waypoints(master)

    mode = 'AUTO'
    mode_id = master.mode_mapping().get(mode)
    if mode_id is not None:
        master.set_mode(mode_id)
        print(f"switched to  {mode} ")
    else:
        print("mode switch fail")

    # 4. 等待直到第6个航点执行完（索引=5）
    wait_until_mission_finished(master, last_seq=5)

    mode = 'GUIDED'
    mode_id = master.mode_mapping().get(mode)
    if mode_id is not None:
        master.set_mode(mode_id)
        print(f"switched to  {mode} ")
    else:
        print("mode switch fail")
    time.sleep(2)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        0,   # 目标朝向 0
        30,   # 旋转速度（度/秒）
        1,    # 顺时针 (1) / 逆时针 (0)
        0,    # 停留时间（不需要）
        0, 0, 0  # 其他参数留空
    )

    while True:
        attitude_msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=5)
        if attitude_msg:
            current_yaw = math.degrees(attitude_msg.yaw)
            if abs(current_yaw - 0) < 1:
                print("Yaw angle aligned to target")
                break
    time.sleep(5)

    print("first stage mission finished")

    # -----------------------------
    # 阶段2：上传一个降落航点 + 执行
    msg_position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    msg_attitude = master.recv_match(type="ATTITUDE", blocking=True, timeout=5)


    lat = msg_position.lat / 1e7
    lon = msg_position.lon / 1e7
    alt = msg_position.relative_alt / 1000  # 转换为米

    yaw = math.degrees(msg_attitude.yaw)
    pitch = math.degrees(msg_attitude.pitch)
    roll = math.degrees(msg_attitude.roll) #Yaw angle (-pi..+pi)
    print(f"Latitude: {lat:.7f}, Longitude: {lon:.7f}, Altitude: {alt:.2f}m")
    print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
    time.sleep(5)



    # ========== 主程序 ==========
    # 读取图像（此处使用摄像头采集，实际使用中可以更换为图片文件）

    drone_lat = lat      # 纬度
    drone_lon = lon      # 经度
    drone_alt = alt      # 高度（单位：米）
    #roll = 0.0           # 横滚角（单位：弧度）
    #pitch = 0.0         # 俯仰角（单位：弧度）
    #angle = 45
    #yaw = radians(angle)         # 偏航角（角度转换为弧度，例如45度）
    
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("摄像头读取失败")
        exit(1)

    # 保存原始图像
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
    #print("大象：", gps_data['elephant'])

    # 定义四个顶点（请确保顶点的顺序能够正确描述多边形区域）
    polygon_points = [
        (51.4236378, -2.6708736),
        (51.4235444, -2.6708193),
        (51.4235508, -2.6712004),
        (51.4234673, -2.6711314)
]
    
    rhino_location_set = (51.4234772, -2.6711175)

    # 提取需要判断的点
    point = gps_data['rhino'][0]  # 格式为 (lat, lon)

    if point_in_polygon(point, polygon_points):
        land_latlon = point
    else:
        land_latlon = rhino_location_set

    #Rhino Location
    lat2, lon2 = land_latlon
    master.mav.set_position_target_global_int_send(
        0,  # 时间戳（0=立即执行）
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # 相对高度坐标系
        int(0b110111111000),  # 仅控制位置
        int(lat2 * 1e7),  # 纬度
        int(lon2 * 1e7),  # 经度
        30,  # 高度
        1, 1, 1,  # 忽略速度控制
        0, 0, 0,  # 忽略加速度控制
        0, 0  # 忽略偏航
    )

    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            if abs(current_lat - lat2) < 0.00002 and abs(current_lon - lon2) < 0.00002:
                print("reach landing point")
                break


    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # Confirmation
        0, 0, 0, 0,  # 不需要额外参数
        0, 0, 0
    )
    print(" `GUIDED` landing")

    while True:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:  # 如果 `ARMED` 标志消失，说明无人机已 DISARM
                print("landed")
                break

    master.mav.mission_clear_all_send(master.target_system, master.target_component)

    mode = 'GUIDED'
    mode_id = master.mode_mapping().get(mode)

    if mode_id is not None:
        master.set_mode(mode_id)
        print(f"switched to  {mode} ")
    else:
        print("mode switch fail")
    time.sleep(5)

    master.arducopter_arm()

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # 确认
        0, 0, 0, 0,  # 其他参数（通常不用）
        0, 0, target_altitude  # 纬度, 经度, 高度
    )


    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # 转换成米
            print(f"altitude: {current_alt}m")
            if current_alt >= target_altitude - 1:  # 允许 1m 误差
                print("reached target altitude")
                break

    upload_third_6_waypoints(master)

    mode = 'AUTO'
    mode_id = master.mode_mapping().get(mode)
    if mode_id is not None:
        master.set_mode(mode_id)
        print(f"switched to  {mode} ")
    else:
        print("mode switch fail")

    wait_until_mission_finished(master, last_seq=5)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # Confirmation
        0, 0, 0, 0,  # 不需要额外参数
        0, 0, 0
    )
    print("`GUIDED` landing")

    while True:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                print("landed")
                break


if __name__ == "__main__":
    main()


