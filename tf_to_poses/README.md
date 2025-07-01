# tf to poses
## Introduction
This project is used for users who wants to receive the position of the robot, the lidar and the camera on the in geometry_msgs.PoseStamped on wanted global frame.
The message is calculated by the lookuptransform mechanism of the TF.
This branch is for ros2 (humble).

### docker image
docker pull cognimbus/tf_to_poses:humble  

### parameters
#### base_frame
the wanted base_frame (robot's location on the map).
#### laser_frame
the wanted laser_frame (lidar's location on the map).
#### camera_frame
the wanted camera_frame (camera's location on the map).
#### global_frame
the wanted global_frame (by deafult is map).

#### rate
The rate in HZ for publishing the messages

### running the launch
#### ros2 launch tf_to_poses bringup_launch.py


