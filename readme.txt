# Base ----------------------------------------
yolo version -> yolo v5 6.0
model -> best.pt
virtual environment -> conda activate yolo_v5
label tool -> labelimg



# PS   ----------------------------------------
in GPS_finalversion_test.py
61  y镜像 dy = y_ratio * (FOV_Y / 2)
    非镜像 dy = -y_ratio * (FOV_Y / 2)
    偏航角yaw需要取负值


