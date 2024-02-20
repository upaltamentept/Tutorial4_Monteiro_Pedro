#include <grasp/grasp.h>

void Grasp::init()
{
  object_pose_sub_ = nh_.subscribe
                     ("/grasp_object", 1, &Grasp::ObjectPoseCallback, this);
  grasp_pub_ = nh_.advertise<std_msgs::Bool>("/grasp_done", 1);

  place_sub_ = nh_.subscribe("/put_down", 1, &Grasp::PlaceCallback, this);

  grasp_done_.data = true;

  tiago_arm_torso_.setPoseReferenceFrame("base_footprint");
}

/* Gripper Control */
void Grasp::closeGripper()
{
  std::vector<double> close_value = {0.001, 0.001};
  
  tiago_gripper_.setJointValueTarget(close_value);
  tiago_gripper_.plan(gripper_plan_);
  tiago_gripper_.move();
  ROS_INFO_STREAM("Gripper Closed");
}

void Grasp::closeGripper_tiago()
{
  system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");
  ROS_INFO_STREAM("Gripper Closed");
}

void Grasp::openGripper()
{
  std::vector<double> open_value = {0.044, 0.044};
  
  tiago_gripper_.setJointValueTarget(open_value);
  tiago_gripper_.plan(gripper_plan_);
  tiago_gripper_.move();
  ROS_INFO_STREAM("Gripper Opened");
}

/* Functionalities to Grasp */
void Grasp::intoReadyPose()
{
  std::vector<double> arm_ready_value;
  arm_ready_value = {0.15, 1.57, 0, -1.57, 1.57, -1.57, 80, 0};

  tiago_arm_torso_.setJointValueTarget(arm_ready_value);
  tiago_arm_torso_.plan(arm_plan_);
  tiago_arm_torso_.move();
}

void Grasp::preGraspApproach()
{ 
  pre_approach_pose_ = target_obj_pose_;
  pre_approach_pose_.pose.position.x -= 0.3;

  ROS_INFO_STREAM("Planned Position: " << pre_approach_pose_.pose.position);

  tf2::Quaternion quaternion;
  quaternion.setRPY(-1.57, 0.0, 0.0);
  
  pre_approach_pose_.pose.orientation = tf2::toMsg(quaternion);
  tiago_arm_torso_.setPoseTarget(pre_approach_pose_);

  bool succ = (tiago_arm_torso_.plan(arm_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  if (!succ)
  {
    ROS_INFO_STREAM("Planning failed");
  }

  tiago_arm_torso_.move();
  ROS_INFO_STREAM("Pre Grasp Goal Reached");
}

void Grasp::toGraspPose()
{
  grasp_pose_ = pre_approach_pose_.pose;
  grasp_pose_.position.x += 0.22;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_approach_pose_.pose);
  waypoints.push_back(grasp_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = tiago_arm_torso_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  tiago_arm_torso_.execute(trajectory);
}

void Grasp::retreatArm()
{
  retreat_pose_ = grasp_pose_;
  retreat_pose_.position.z += 0.2;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(grasp_pose_);
  waypoints.push_back(retreat_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = tiago_arm_torso_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  tiago_arm_torso_.execute(trajectory);
}

/* Functionalities to Place */
void Grasp::toPlacePose()
{
  ROS_INFO_STREAM("Preparing to Place");
  
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(retreat_pose_);
  waypoints.push_back(grasp_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = tiago_arm_torso_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  tiago_arm_torso_.execute(trajectory);
  
  ROS_INFO_STREAM("Place Pose Reached");
}

/*  */
bool Grasp::grasp(geometry_msgs::PoseStamped &obj_pose)
{
  ROS_INFO_STREAM("Starting Grap Procedure");
  target_obj_pose_ = obj_pose;
  intoReadyPose();
  preGraspApproach();
  openGripper();
  toGraspPose();
  closeGripper();
  retreatArm();
  grasp_command_ = false;
  return true;
}

void Grasp::place()
{
  toPlacePose();
  openGripper();
  place_command_ = false;
}

void Grasp::update()
{
  if (grasp_command_)
  {
    if (grasp(target_obj_pose_))
    {
      ROS_INFO_STREAM("Grasping Done");
      grasp_pub_.publish(grasp_done_);
    }
  }

  if (place_command_)
  {
    place();
  }
}

void Grasp::ObjectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  double x,y,z;
  x = msg->pose.position.x;
  y = msg->pose.position.y;
  z = msg->pose.position.z;
  target_obj_pose_.header.frame_id = "base_footprint";
  target_obj_pose_.pose.position.x = x;
  target_obj_pose_.pose.position.y = y;
  target_obj_pose_.pose.position.z = z;
  target_obj_pose_.pose.orientation.w = 1.0;
  grasp_command_ = true;
}

void Grasp::PlaceCallback(const std_msgs::Bool::ConstPtr msg)
{
  if (msg->data == true)
  {
    place_command_ = true;
  }
}
