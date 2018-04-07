#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>


#define PI 3.14159265359
std::string turtle_name;


double left_val = 5;
double right_val = -5;
double left_angle_val = 0;
double right_angle_val = 0;

void baseline_callback(const std_msgs::Float64 value){

  float val = value.data;


  left_val = -1*(val/1000)/2;
  right_val = (val/1000)/2;

}

#define PI 3.14159265359
// this function used to convert degree to rad
float deg2rad(float degree){
    return degree*PI/180;
}

void left_callback(const std_msgs::Float64 value){

  left_angle_val = deg2rad(180+(90 - (value.data)));

}

void right_callback(const std_msgs::Float64 value){

  right_angle_val = deg2rad(180+ (90 - (value.data)));

}

geometry_msgs::Vector3 object_pos;

void obect_pos_callback(const geometry_msgs::Vector3 value){

  object_pos.x = value.x/1000;  // convert the value to meters
  object_pos.y = value.y/1000;  // convert the value to meters
  object_pos.z = value.z/1000;  // convert the value to meters

}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publish");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("baseline/position", 10, &baseline_callback);
  ros::Subscriber left_cam_angle_sub = nh.subscribe("left/pan/angle", 10, &left_callback);
  ros::Subscriber right_cam_angle_sub = nh.subscribe("right/pan/angle", 10, &right_callback);
  ros::Subscriber object_pos_sub = nh.subscribe("/object_pos", 10, &obect_pos_callback);

  //create the transformation
  static tf::TransformBroadcaster left_cam;
  tf::Transform left_transform;

  static tf::TransformBroadcaster right_br;
  tf::Transform right_transform;

  static tf::TransformBroadcaster object_pos_TB;
  tf::Transform object_pos_trans;

  double rate = 10.0;
  ros::Rate r(rate);
  while(nh.ok()){

      ros::spinOnce();
      tf::Quaternion q_left_pan_cam;
      q_left_pan_cam.setRPY(0, 0, left_angle_val);
      left_transform.setRotation(q_left_pan_cam);

      left_transform.setOrigin( tf::Vector3(0, left_val, 0.035) );
      left_cam.sendTransform(tf::StampedTransform(left_transform, ros::Time::now(), "/rail_link", "/left_pan_link"));


      tf::Quaternion q_right_pan_cam;
      q_right_pan_cam.setRPY(0, 0, right_angle_val);
      right_transform.setRotation(q_right_pan_cam);
      right_transform.setOrigin( tf::Vector3(0, right_val, 0.065) );
      right_br.sendTransform(tf::StampedTransform(right_transform, ros::Time::now(), "/rail_link", "/right_pan_link"));


      object_pos_trans.setOrigin(tf::Vector3(object_pos.y,object_pos.x,0.065));
      object_pos_TB.sendTransform(tf::StampedTransform(object_pos_trans, ros::Time::now(),"/rail_link", "/object_pos"));

      r.sleep();
    }

  ros::spin();
  return 0;
}
