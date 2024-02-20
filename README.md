# Comannd


# TIAGO Movement c++ project


To build this project, you will need the following:
Robot:Tiago

*********************************************************************************************************
$cd ros/workspaces/workspace

$catkin_make

$source devel/setup.bash
**********************************************Exercise 1**************************************************

Launch a world with a table of items and tiago and YOLO

 $ 
 roslaunch object_detection_world tiago.launch world_suffix:=tutorial2

Lower Tiago's head

  $ rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['head_1_joint', 'head_2_joint']
points:
- positions: [0.0, -0.7]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 2, nsecs: 0}"

**********************************************Exercise 2**************************************************
Check /perception_msgs/Rect

 $ rostopic echo /perception_msgs/Rect

**********************************************Exercise 3**************************************************
Launch Point cloud segmentation

$ 
roslaunch plane_segmentation plane_segmentation_tiago.launch 

Change topic of Point Cloud in RViz to see point cloud with objects only/ with table surface only/ with objects and table surface

**********************************************Exercise 4**************************************************

$ roslaunch object_labeling object_labeling.launch 

Change topic of Point cloud to /labeled_objects

