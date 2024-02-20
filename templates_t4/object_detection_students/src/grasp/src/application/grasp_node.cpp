#include <grasp/grasp.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  /*  
  geometry_msgs::PoseStamped target_obj_pose;
  target_obj_pose.header.frame_id = "base_footprint";
  target_obj_pose.pose.position.x = 0.5;
  target_obj_pose.pose.position.y = -0.5;
  target_obj_pose.pose.position.z = 0.5;
  target_obj_pose.pose.orientation.w = 1;

  Grasp grasp;

  grasp.grasp(target_obj_pose);
  */   

  Grasp grasp;

  grasp.init();

  ros::Rate loop_rate(10);
  loop_rate.sleep();
  while (ros::ok())
  { 
    grasp.update();
    loop_rate.sleep();
  }



  ros::shutdown;
  return 0;
}