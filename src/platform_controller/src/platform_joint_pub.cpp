#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


using namespace std;


const double degree = M_PI/180;

//Create a joint stat variables
sensor_msgs::JointState joint_state;
sensor_msgs::JointState left_cam_state;
sensor_msgs::JointState right_cam_state;
sensor_msgs::JointState baseline_state;


//define the variables
float left_pan_cam_angle;
float right_pan_cam_angle;
float baseline_value;

//create a call back function to subscribe the value
void left_cam_callback(const std_msgs::Float64 value){
  //store the value in it's variablies
  left_pan_cam_angle = value.data;
}

void right_cam_callback(const std_msgs::Float64 value){
  //store the value in it's variable
  right_pan_cam_angle = value.data;
}

void baseline_callback(const std_msgs::Float64 value){
  baseline_value = value.data;
}




int main(int argc, char **argv)
{
  ros::init(argc , argv, "controller");
  ros::NodeHandle nh;


  //create the subscriber
  ros::Subscriber left_pan_cam_sub = nh.subscribe("left/pan/angle",10,left_cam_callback);
  ros::Subscriber right_pan_cam_sub = nh.subscribe("right/pan/angle",10,right_cam_callback);
  ros::Subscriber basline_sub = nh.subscribe("baseline/value",10,baseline_callback);

  //create the publisher of the joint state
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_state_publisher",1);

  //define the joint state
  joint_state.name.resize(3);
  joint_state.position.resize(3);
  joint_state.name[0] ="baseline_joint";
  joint_state.name[1] ="left_pan_cam_joint";
  joint_state.name[2] ="right_cam_joint";

  double rate = 10.0;
  ros::Rate r(rate);

  while(nh.ok()){
      ros::spinOnce();
      // update the joint
      joint_state.header.stamp = ros::Time::now();
      joint_state.position.push_back(baseline_value);
      joint_state.position.push_back(left_pan_cam_angle * degree);
      joint_state.position.push_back(right_pan_cam_angle * degree);

      //publish the joints
      joint_state_pub.publish(joint_state);

      r.sleep();
    }



}
