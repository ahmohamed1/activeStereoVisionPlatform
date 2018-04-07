#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

////////////////////////////////////////////////
/// The digram shows the tringul and the difinition used in the calculation
///
///                  P
///                 /B\
///                /   \
///               /     \
///             l/       \r
///             /         \
///            /           \
///           /             \
///          /_______________\
///          R        b      L
///


using namespace std;

double baseline_value,left_pan,right_pan;
// define the angles used in the calulcation
double AL = 0, AR = 0, AO = 0;
double LS = 0, RS = 0;
double DR =0, DL =0;
double LSS = 0, RSS =0;
double XposL = 0, XposR = 0;

#define PI 3.14159265359
// this function used to convert degree to rad
double deg2rad(double degree){
    return (degree * M_PI / 180);
}
void left_pan_callback(const std_msgs::Float64& val){
  left_pan = val.data;
}

void right_pan_callback(const std_msgs::Float64& val){
  right_pan = val.data;
}

void baseline_callback(const std_msgs::Float64& val){
  baseline_value = val.data;
}


geometry_msgs::Vector3 object_pos;

int main( int argv,char **argc)
{

  ros::init(argv,argc,"two_and_half_depth");
  ros::NodeHandle nh;

  ros::Publisher left_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/left/pan/move", 1);
  ros::Publisher right_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/right/pan/move",1);
  ros::Publisher baseline_pub = nh.advertise<std_msgs::Float64>("/baseline/move",1);
  ros::Publisher object_pos_pub = nh.advertise<geometry_msgs::Vector3>("/object_pos",1); ////This publish the coordinate of the object

  // define the angle subscriper
  ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",1,right_pan_callback);
  ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",1,left_pan_callback);
  ros::Subscriber baseline_sub = nh.subscribe("/baseline/position",1,baseline_callback);

  ros::Rate rate(20);   // Hz

  while(nh.ok())
    {
      ros::spinOnce();
      cout <<" ================================================"<<endl;
      double B = baseline_value;
      //Step 1- convert the angles to the coordinate
      AL = 90 - left_pan;
      AR = 90 + right_pan;

      //Step 2- calcualte the B angle and the length of r all values are in mm
        AO = 180 - (AL + AR);
      //cout << "B: "<<B<<endl;
        LS = B * sin(deg2rad(AL)) / sin(deg2rad(AO));
        RS = B * sin(deg2rad(AR)) / sin(deg2rad(AO));

        // cout << "side R: "<<RS<<" L: " << LS <<endl;

      //Step 3 - calculate the targer position in x and z
        DR = RS * sin(deg2rad(AL));
        DL = LS * sin(deg2rad(AR));
        cout << "depth R: "<<DR<<" L: " << DL <<endl;
      // Step 4 - calculate the short side therefore we can calculate the x position
        RSS = RS * cos(deg2rad(AL));
        LSS = LS * cos(deg2rad(AR));

        // cout << "short side R: "<<RSS<<" L: " << LSS << " SUM: "<< RSS+LSS <<endl;

        XposL = (-B/2) + LSS;
        XposR = (B/2) - RSS;

        cout << "Xpos R: "<< XposR <<" L: " << XposL <<endl;

      //assinge the coordinate to the object position and publish the topic
      object_pos.x = XposR;
      object_pos.y = DR;
      object_pos.z = 0;
      object_pos_pub.publish(object_pos);
      rate.sleep();
    }
  return 0;
}
