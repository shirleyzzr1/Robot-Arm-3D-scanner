## FAKE_SENSOR

##### PACKAGE LIST

The repository consists of the following ROS packages

- **realsense_reader**

  Starting the realsense camera and process the pointcloud data to fuse the lidar data

- **adroit_move**

  Control the movement of the adroit arm

##### Hardware Requirement
- **Adroit Arm**
- **Realsense Camera**

##### USAGE

1. make sure you have realsense2_camera package installed 

2. start the camera and start data fusion

```
roslaunch realsense_reader read_camera.launch use_rviz:=false
```

3. start the adroit arm control

```
roslaunch adroit_move adroit_move.launch simulation:=false
```

<arg name = "simulation" set simulation to false if running on the real robot


