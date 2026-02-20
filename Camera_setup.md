Setup for the Realsense Camera: Enabling ONLY RGB (saivng computing power)


ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_gyro:=false \
  enable_accel:=false \
  enable_pointcloud:=false \
  align_depth.enable:=false

See the topic list

\camera\camera\color\image_raw

This displays the raw image data, which we need a seperate node to run: Camera_Sub_Node.py



