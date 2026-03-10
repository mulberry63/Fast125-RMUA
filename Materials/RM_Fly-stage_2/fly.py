import airsim

client = airsim.MultirotorClient()
client.enableApiControl(True)

# fly_speed = 4.0
# drivetrain = airsim.DrivetrainType.ForwardOnly
# yaw_mode = airsim.YawMode(False, 0)

# connect to the AirSim simulator
# client.enableApiControl(True)   # get control
# client.armDisarm(True)          # unlock
# client.takeoffAsync().join()    # takeoff

# square flight
# client.moveToZAsync(-3, fly_speed).join()               # 上升到3m高度
# client.moveToPositionAsync(9, 0, -2.8, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标
# client.moveToPositionAsync(10, -3, -2.8, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标
# client.moveToPositionAsync(13, -14, -2.8, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标
# client.moveToPositionAsync(15.5, -21.9, -2.6, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标 1
# client.moveToPositionAsync(23.0, -45.2, -2.5, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标 2
# client.moveToPositionAsync(20.65, -64.78, -2.0, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标 3
# client.moveToPositionAsync(9, -81.67, -1.45, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标 4
# client.moveToPositionAsync(-10.35, -93.2, -1.58, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标 5
# client.moveToPositionAsync(-29.9, -97.67, -3.15, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标 6
# client.moveToPositionAsync(-51.8, -101.8, -4.97, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标 7
# client.moveToPositionAsync(-62.57, -102.6, -4.45, fly_speed, drivetrain=drivetrain, yaw_mode=yaw_mode).join()  # 飞到（5,0）点坐标


# names = ["MovingCircle_2", "MovingCircle2_5",
# "MovingCircle3_11",
# "MovingCircle4_2"]

# for name in names:
#     pose = client.simGetObjectPose(name)
#     print(name)
#     tmp_y = pose.position.y_val
#     pose.position.y_val = pose.position.x_val
#     pose.position.x_val = pose.position.y_val
#     print(pose.position)
#     print(pose.orientation)
#     print("\n")

# client.landAsync().join()       # land
# client.armDisarm(False)         # lock
# client.enableApiControl(False)  # release control
