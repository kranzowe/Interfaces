# How to Run Rover Autonomously using Lidar Wide Field Integration Only

- First place Rover at the starting location, and start the Robo rover
```bash
ros2 launch robo_rover rover_launch.py
```
- Spin up the lidar launch file
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```
- Run the Lidar Bug Script (This pwm value gives it a good enough speed, 
  can play with faster speeds, Lower pwm value == Higher Speed)
```bash
ros2 run clanker_hardware lidar_bug.py --ros-args -p ol_speed:=1390.0
```


# How to Run Auntonomous Localization with Lidar Bug Script

Since the speed of the rover is heavily dependent on the battery, 
you may need to do calibrate to be sure the computed odom matches the 
actual distance traveled. To determine the constant scalar value.

### To Calibrate ODOM
You need to do at least 5 different runs of this process to have at least
a somewhat estimate of the scaling.
Always place the rover at the beginning of the Start tape which is on the ground. 
Be sure to let the start of the tap align with the rover baselink,
which is the midpoint between the front and reer wheel. Repeat this for 5 diff runs

- Start the Rover node when the Rover is placed correctly, to ensure odom is restarted from 0.
- Run the timed lidar bug script. (This will move the rover forward for 10sec and stop and print out the odom)
```bash
ros2 run clanker_hardware lidar_bug_timed.py --ros-args -p ol_speed:=1390.0
```
- Use an Iphone or tape to measure the actual distance from the start of the tap to the midpoint of the spinning lidar
of where the car is.
- Repeat the process by restarting rover node 5 times and following the process to determine the right scale to use. 
scale = Actual distance / Odom distance

### Add The Scalar value to Rover Node.
After figuring out the scaler value. Go to Line 576 in Rover_node.py and add it, like this
```
return -self.cmd_vel_scale * float(self.get_velocity_ol_steady_state()) * 1.40
```
Build After

### To Run the Localization Autonomously
- Place Robot at the Start Location and Run Robo Rover
- Run Rviz2
```bash
rviz2
```
- Launch the AMCL Localization Script
```bash
ros2 launch estimation_localization amcl_localization.launch.py
```
- In Rviz2, use the 2D set Pose Button at the top to set the Initial Pose on the Map.
Click on the map, where the rover actually is and drag it slightly to the direction it is facing
- In another terminal, run the Lidar Bug Script
```bash
ros2 run clanker_hardware lidar_bug.py --ros-args -p ol_speed:=1390.0
```
- To get the pose of the rover, subscribe to \amcl_pose
