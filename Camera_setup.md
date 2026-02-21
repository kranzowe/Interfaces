Setup for the Realsense Camera: Enabling ONLY RGB @6FPS (saivng computing power)

ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false pointcloud.enable:=false rgb_camera.color_profile:=640x480x6


See the topic list, below is what we want:
\camera\camera\color\image_raw

This displays the raw image data, which we need a package to run all of this. I made camera_test_pkg located in src of ClankerCollective. After you run what is above, do the following commands in the base of ClankerCollective:

source /opt/ros/humble/setup.bash

source ~/ClankerCollective/install/setup.bash

ros2 run camera_test_pkg my_color_sub



