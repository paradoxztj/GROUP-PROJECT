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

import subprocess

connection_str = 'tcp:127.0.0.1:14550'

# 6 waypoint coordinates (lat, lon), assuming the altitude is set to 30 m
waypoints_phase1 = [
    (51.4233649, -2.6715177),  # Takeoff -> 30m
    (51.4228247, -2.6711047),
    (51.4225788, -2.6694953),
    (51.4231993, -2.6687551),
    (51.4239135, -2.6692244),
    (51.4235487, -2.6710046),  # Hover for 5 seconds upon arrival
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
    """Upload the first 7 waypoints and set the takeoff and hover logic."""
    count = len(waypoints_phase1)  # 7

    # Send MISSION_COUNT
    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        count
    )

    seq_uploaded = 0
    while seq_uploaded < count:
        # Wait for MISSION_REQUEST(_INT)
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
            # Normal waypoint
            command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            altitude = takeoff_alt
            if req_seq in [5]:
                altitude = 40
            param1 = 0
            param2 = acceptance_radius
            param3 = 0
            param4 = 0
            autocontinue = 1

        # Send MISSION_ITEM_INT
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

    # Wait for MISSION_ACK
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
        # Use relative altitude coordinate system
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        current = 1 if req_seq == 0 else 0  # Mark the first waypoint as current

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

        # Send MISSION_ITEM_INT
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

    # Wait for MISSION_ACK
    ack_msg = master.recv_match(type=['MISSION_ACK'], blocking=True)
    if ack_msg:
        ack_msg = ack_msg.to_dict()
        if ack_msg.get('type') == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("[Phase3] Mission upload successful (ACK)")
        else:
            print("[Phase3] Mission upload failed: ACK type=", ack_msg.get('type'))


# ========== Camera intrinsic parameters ==========
K = np.array([[761.9, 0, 320],
              [0, 761.9, 240],
              [0, 0, 1]])

# ========== Load YOLOv5 model ==========
yolo_model = torch.hub.load('ultralytics/yolov5', 'custom',
                            path="C:\\Users\\Spectral\\Desktop\\ar\\.venv\\best.pt")

# ========== Assume the drone status is known ==========
# Please set according to the actual situation
# drone_lat = 37.7749       # latitude
# drone_lon = -122.4194     # longitude
# drone_alt = 100.0         # altitude (in meters)
# roll = 0.0                # roll angle (in radians)
# pitch = 0.0               # pitch angle (in radians)
# angle = 45
# yaw = radians(angle)      # yaw angle (converted from degrees to radians, e.g., 45°)

# ========== Pixel coordinates → GPS ==========
# Actual size of camera field of view at 40m altitude
FOV_X = 33.5  # Horizontal width (meters)
FOV_Y = 25.0  # Vertical height (meters)

# Image dimensions
IMG_W = 1456  # Image width (pixels)
IMG_H = 1088  # Image height (pixels)


def pixel_to_gps(u, v, drone_lat, drone_lon, drone_alt, roll, pitch, yaw):
    # Calculate relative offset of pixel from image center
    x_ratio = (u - IMG_W / 2) / (IMG_W / 2)  # Normalize to [-1, 1]
    y_ratio = (v - IMG_H / 2) / (IMG_H / 2)  # Normalize to [-1, 1]

    # Map pixel ratio to actual ground displacement
    dx = x_ratio * (FOV_X / 2)  # Horizontal offset (meters)
    dy = -y_ratio * (FOV_Y / 2)  # Vertical offset (meters)

    # Rotate the offset to align with drone heading
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw)],
                      [np.sin(yaw), np.cos(yaw)]])
    offset = R_yaw @ np.array([dx, dy])  # Rotated (dx, dy)

    # Convert drone GPS coordinates to UTM
    utm_x, utm_y, zone, letter = utm.from_latlon(drone_lat, drone_lon)

    # Calculate UTM coordinates of target point
    tgt_x = utm_x + offset[0]
    tgt_y = utm_y + offset[1]

    # Convert back to GPS coordinates
    tgt_lat, tgt_lon = utm.to_latlon(tgt_x, tgt_y, zone, letter)

    return tgt_lat, tgt_lon


# ========== Draw detection boxes ==========
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


# ========== Analyze image ==========
def analyze_frame(frame, drone_lat, drone_lon, drone_alt, yaw, polygon):
    results = yolo_model(frame)
    detections = results.xyxy[0].cpu().numpy()

    counts = {'zebra': 0, 'rhino': 0, 'elephant': 0}
    gps_locations = {'zebra': [], 'rhino': [], 'elephant': []}
    object_positions = []  # Stores GPS coordinates of specific targets (e.g., rhinos)

    for det in detections:
        x1, y1, x2, y2, conf, cls = det
        u = (x1 + x2) / 2
        v = (y1 + y2) / 2
        roll = 0
        pitch = 0
        gps_lat, gps_lon = pixel_to_gps(u, v, drone_lat, drone_lon, drone_alt, roll, pitch, yaw)

        # Check if the detected point is within the designated area
        if not point_in_polygon((gps_lat, gps_lon), polygon):
            continue  # Skip processing if outside the area

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


# Polygon vertex check
def point_in_polygon(point, polygon):
    """
    Determine whether a point is inside a polygon
    :param point: (lat, lon)
    :param polygon: [(lat1, lon1), (lat2, lon2), ...] List of polygon vertices; the order must be correct (clockwise or anticlockwise)
    :return: True if the point is inside the polygon, otherwise False
    """
    lat, lon = point
    # To use the standard ray-casting method, first convert (lat, lon) to (x, y), where x is longitude and y is latitude
    x, y = lon, lat
    # Similarly, convert the polygon points
    poly = [(p[1], p[0]) for p in polygon]

    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        # Determine whether the line segment formed by the current point and the previous point crosses the ray
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def main():
    master = mavutil.mavlink_connection(connection_str)
    master.wait_heartbeat()
    print("Heartbeat OK - system %u component %u" % (master.target_system, master.target_component))

    master.mav.set_mode_send(
        master.target_system,
        1,
        4
    )
    time.sleep(5)

    master.arducopter_arm()

    target_altitude = 30  # Target takeoff altitude

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0, 0, 0, 0,  # other parameters (usually not used)
        0, 0, target_altitude  # latitude, longitude, altitude
    )

    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # convert to meters
            print(f"altitude: {current_alt}m")
            if current_alt >= target_altitude - 1:  # allow a 1m error margin
                print("reached target altitude")
                break
    # -----------------------------
    # Phase 1: Upload the first 6 waypoints and execute
    master.mav.mission_clear_all_send(master.target_system, master.target_component)

    upload_first_6_waypoints(master)

    master.mav.set_mode_send(
        master.target_system,
        1,
        3
    )

    # 4. Wait until the 6th waypoint is executed (index=5)
    wait_until_mission_finished(master, last_seq=5)

    master.mav.set_mode_send(
        master.target_system,
        1,
        4
    )
    time.sleep(2)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        0,  # target heading 0
        30,  # rotation speed (degrees/second)
        1,  # clockwise (1) / anticlockwise (0)
        0,  # hold time (not needed)
        0, 0, 0  # other parameters left empty
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
    # Phase 2: Upload a landing waypoint and execute
    msg_position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    msg_attitude = master.recv_match(type="ATTITUDE", blocking=True, timeout=5)

    lat = msg_position.lat / 1e7
    lon = msg_position.lon / 1e7
    alt = msg_position.relative_alt / 1000  # convert to meters

    yaw = math.degrees(msg_attitude.yaw)
    pitch = math.degrees(msg_attitude.pitch)
    roll = math.degrees(msg_attitude.roll)  # Yaw angle (-pi..+pi)
    print(f"Latitude: {lat:.7f}, Longitude: {lon:.7f}, Altitude: {alt:.2f}m")
    print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
    time.sleep(5)

    # ========== Main program ==========
    # Read image (using camera capture here; in actual use, you can replace it with an image file)

    drone_lat = lat  # latitude
    drone_lon = lon  # longitude
    drone_alt = alt  # altitude (in meters)
    # roll = 0.0           # roll angle (in radians)
    # pitch = 0.0          # pitch angle (in radians)
    # angle = 45
    # yaw = radians(angle) # yaw angle (convert from degrees to radians, e.g., 45°)

    timestamp = 20170615_123845  # Generate the current timestamp string for file naming
    image_path = r"C:\Users\Spectral\Desktop\ar\.venv\raw_image_20170615_123845.jpg"
    frame = cv2.imread(image_path)
    polygon = [
        (51.4236378, -2.6708736),
        (51.4235444, -2.6708193),
        (51.4234673, -2.6711314),
        (51.4235508, -2.6712004)
    ]

    if frame is not None:
        # Object detection and localization
        counts, gps_data, object_positions, detections = analyze_frame(frame, drone_lat, drone_lon, drone_alt, yaw, polygon)

        # Save image with detection boxes
        annotated = draw_detections(frame.copy(), detections)
        detected_path = f"detected_image_{timestamp}.jpg"
        cv2.imwrite(detected_path, annotated)
        print(f"Detection image saved: {detected_path}")

        # Output detection results
        print("Detection results:", counts)
        print("Zebra:", gps_data['zebra'])
        print("Rhino:", gps_data['rhino'])
        print("Elephant:", gps_data['elephant'])

        result_path = f"result_{timestamp}.txt"
        with open(result_path, 'w') as f:
            f.write(f"result: {counts}\n")
            f.write(f"zebra: {gps_data.get('zebra')}\n")
            f.write(f"rhino: {gps_data.get('rhino')}\n")
            f.write(f"elephant: {gps_data.get('elephant')}\n")
        print(f"Detection result saved: {result_path}")


        rhino_location_set = (51.4234772, -2.6711175)

        point = gps_data['rhino'][0]  # Format: (lat, lon)

        if point_in_polygon(point, polygon):
            land_latlon = point
        else:
            land_latlon = rhino_location_set
    else:
        land_latlon = rhino_location_set
        print("The image is unavailable, skip target detection and result recording.")

    # Rhino Location
    lat2, lon2 = land_latlon
    master.mav.set_position_target_global_int_send(
        0,  # Timestamp (0 = execute immediately)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Relative altitude coordinate system
        int(0b110111111000),  # Control position only
        int(lat2 * 1e7),  # Latitude
        int(lon2 * 1e7),  # Longitude
        30,  # Altitude
        1, 1, 1,  # Ignore speed control
        0, 0, 0,  # Ignore acceleration control
        0, 0  # Ignore yaw
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
        0, 0, 0, 0,  # No additional parameters required
        0, 0, 0
    )
    print(" `GUIDED` landing")

    while True:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:  # If the `ARMED` flag is cleared, the drone is DISARMED
                print("landed")
                break

    time.sleep(5)

    # master.mav.command_long_send(
    #     master.target_system,  # 目标system id
    #     master.target_component,  # 目标component id
    #     mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
    #     0,  # confirmation
    #     8,  # param1 = SERVO通道 (这里是第8路)
    #     2200,  # param2 = PWM微秒值 (例如1500μs)
    #     0, 0, 0, 0, 0  # 剩余参数无用
    # )
    # time.sleep(5)
    master.mav.mission_clear_all_send(master.target_system, master.target_component)

    master.mav.set_mode_send(
        master.target_system,
        1,
        4
    )
    time.sleep(5)

    master.arducopter_arm()

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0, 0, 0, 0,  # other parameters (usually not used)
        0, 0, target_altitude  # latitude, longitude, altitude
    )

    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # convert to meters
            print(f"altitude: {current_alt}m")
            if current_alt >= target_altitude - 1:  # allow a 1m error margin
                print("reached target altitude")
                break

    upload_third_6_waypoints(master)

    master.mav.set_mode_send(
        master.target_system,
        1,
        3
    )

    wait_until_mission_finished(master, last_seq=5)

    master.mav.set_mode_send(
        master.target_system,
        1,
        4
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,  # Confirmation
        0, 0, 0, 0,  # No additional parameters required
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