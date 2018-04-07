#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

using namespace std;


int main( int argv,char **argc)
{

  ros::init(argv,argc,"controller");
  ros::NodeHandle nh;

  ros::Publisher left_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/left/pan/move", 50);
  ros::Publisher right_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/right/pan/move",50);
  ros::Publisher baseline_pub = nh.advertise<std_msgs::Float64>("/baseline/move",50);

  //Define virables to store the value of joints
  geometry_msgs::Vector3 lcam_pan, rcam_pan;
  std_msgs::Float64 baseline;

  float rangle,langle, _baseline;
  while(nh.ok())
    {
      ros::spinOnce();
      cout<<"Please enter (l) for left cam, (r) for right cam, (b) for baseline, or (q) to exit:"<<endl;
      char joint_char;
      cin >> joint_char;
      if(joint_char == 'l') {
	  cout << "Please enter the value of the angle (-50 - 50):"<<endl;
	  cin >> langle;
	  lcam_pan.x = langle;
	  lcam_pan.y = 0;
	  lcam_pan.z = 0;
	  // publish data
	  left_cam_pan_pub.publish(lcam_pan);
	}else if(joint_char == 'r') {
	  cout << "Please enter the value of the angle (-50 - 50):"<<endl;
	  cin >> rangle;
	  rcam_pan.x = rangle;
	  rcam_pan.y = 0;
	  rcam_pan.z = 0;
	  // publish data
	  right_cam_pan_pub.publish(rcam_pan);
	}else if(joint_char == 'b') {
	  cout << "Please enter the value of the baseline (50 - 550 mm):"<<endl;
	  cin >> _baseline;
	  baseline.data = _baseline;
	  // publish data
	  baseline_pub.publish(baseline);
	}else{
	  break;
	}
    }
  return 0;
}
