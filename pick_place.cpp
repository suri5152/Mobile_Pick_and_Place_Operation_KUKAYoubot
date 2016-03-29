#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning");
  ROS_INFO("Entered planning node");
  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();
    
  moveit::planning_interface::MoveGroup group("arm_1");
  moveit::planning_interface::MoveGroup group_gripper("arm_1_gripper");
 
 moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";

  
  co.id = "part";
  //co.operation = moveit_msgs::CollisionObject::REMOVE;
 
  
  moveit_msgs::AttachedCollisionObject aco;
  aco.object = co;
 
  
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

  co.primitive_poses[0].position.x = 0.6;
  co.primitive_poses[0].position.y = -0.7;
  co.primitive_poses[0].position.z = 0.5;
 

  //home position
  group.setNamedTarget("folded");
  group.move();
 
  
 //pick position
  std::map<std::string, double> pick_joints;
  pick_joints["arm_joint_1"] =  2.90;
  pick_joints["arm_joint_2"] =2.46;
  pick_joints["arm_joint_3"] = -2.28;
  pick_joints["arm_joint_4"] =  3.38;
  pick_joints["arm_joint_5"] = 2.82;
  group.setJointValueTarget(pick_joints);
  group.move();

  //open the gripper
  group_gripper.setNamedTarget("open");
  group_gripper.move();
 
  // attach part to the gripper
 group_gripper.attachObject("part", "gripper_finger_link_l");
  //close the gripper
  group_gripper.setNamedTarget("close");
  group_gripper.move();
    
//place location
  std::map<std::string, double> place_joints;
  place_joints["arm_joint_1"] =  3.04;
  place_joints["arm_joint_2"] =  0;
  place_joints["arm_joint_3"] = -2.46;
  place_joints["arm_joint_4"] =  0.07;
  place_joints["arm_joint_5"] = 2.82;
  group.setJointValueTarget(place_joints);
  group.move();

  //open the gripper
  group_gripper.setNamedTarget("open");
  group_gripper.move();
  
  //dettach the object
 group_gripper.detachObject("part");
  
  //close the gripper
  group_gripper.setNamedTarget("close");
  group_gripper.move();

 //home position
  group.setNamedTarget("folded");
  group.move(); 
  //ros::shutdown(); 
}
