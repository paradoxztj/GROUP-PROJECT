from pymavlink import mavutil
import time
import math

device_path = '/dev/ttyAMA0'
baud_rate = 921600

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


def main():
    master = mavutil.mavlink_connection(device_path, baud=baud_rate)
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


    # 经纬度（单位转换：MAVLink 发送的是 E7，需除以 1e7）
    lat = msg_position.lat / 1e7
    lon = msg_position.lon / 1e7
    alt = msg_position.relative_alt / 1000  # 转换为米

    yaw = math.degrees(msg_attitude.yaw)
    pitch = math.degrees(msg_attitude.pitch)
    roll = math.degrees(msg_attitude.roll) #Yaw angle (-pi..+pi)
    print(f"Latitude: {lat:.7f}, Longitude: {lon:.7f}, Altitude: {alt:.2f}m")
    print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
    time.sleep(5)

    #Rhino Location
    land_latlon = (51.4234772, -2.6711175)
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

    time.sleep(5)

    master.mav.command_long_send(
        master.target_system,  # 目标system id
        master.target_component,  # 目标component id
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        8,  # param1 = SERVO通道 (这里是第8路)
        2000,  # param2 = PWM微秒值 (例如1500μs)
        0, 0, 0, 0, 0  # 剩余参数无用
    )
    time.sleep(5)

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
