#ifndef GRASP_H
#define GRASP_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>

class Grasp
{
  private:
    moveit::planning_interface::MoveGroupInterface tiago_arm_torso_;
    moveit::planning_interface::MoveGroupInterface tiago_gripper_;
  public:
    Grasp(): tiago_arm_torso_("arm_torso"), tiago_gripper_("gripper") {};
    
    void init();
    bool grasp(geometry_msgs::PoseStamped &obj_pose);
    void place(); 
  
  private:
    // For general setting
    void setWorkspaceToTable(); // TBD

    // To perform a grasp
    void intoReadyPose();
    void preGraspApproach();
    void openGripper();
    void toGraspPose();
    void closeGripper();
    void retreatArm();
    void closeGripper_tiago();

    // To place the object
    void toPlacePose();
    void returnHome();
  
  public:
    void update();

  private:
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;

    geometry_msgs::PoseStamped target_obj_pose_;
    geometry_msgs::PoseStamped pre_approach_pose_;
    geometry_msgs::Pose grasp_pose_;
    geometry_msgs::Pose retreat_pose_;
    geometry_msgs::Pose place_pose_;

    ros::NodeHandle nh_;

    ros::Subscriber object_pose_sub_;
    ros::Publisher grasp_pub_;

    ros::Subscriber place_sub_;

    void ObjectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void PlaceCallback(const std_msgs::Bool::ConstPtr msg);
    
    bool grasp_command_ = false;
    bool place_command_ = false;
    std_msgs::Bool grasp_done_;
};

#endif