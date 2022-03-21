### Adroit Arm Control

This Package contains the interaction with the hdt adroit arm using moveit.

#### Files:

**basic_control.yaml** provides the waypoints and gripper state for robot to 
follow. gripper_strength controls to what extent the robot pincer is closed.

**adroit_move.launch** loads all the hardware and software required and controllers

for hdt adroit arm, and also provides the transformation from camera frame to object.

#### Basic control design:

Cartesian_path is used to for path planning, which has really quick and
accurate execuation, and are more likely to reach the desired goals.For adroit arm, 
the joint limit is been set on each of the joint, so that the arm won't go
into the state of continously rotating. The limit is changed in the urdf file.

#### Camera calibration:

First find the transformation from camera_depth_frame to TAG

Then match the distance between TAG to robot base link

Set a static transform from robot end_link to object frame for better understanding

Then we get the whole complete tf tree

