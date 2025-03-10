from pymavlink import mavutil

# 假设已连接到飞控
device_path = 'COM4'
baud_rate = 9600

master = mavutil.mavlink_connection(device_path, baud=baud_rate)
master.wait_heartbeat()



master.arducopter_arm()

master.mav.command_long_send(
    master.target_system,     # 目标system id
    master.target_component,  # 目标component id
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
    0,     # confirmation
    8,     # param1 = SERVO通道 (这里是第8路)
    1000,  # param2 = PWM微秒值 (例如1500μs)
    0,0,0,0,0  # 剩余参数无用
)
