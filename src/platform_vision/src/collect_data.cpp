#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "opencv2/core/utility.hpp"

#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <std_msgs/String.h>

#include <generate_disparity_map.h>


using namespace cv;
using namespace std;

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

        // calibration repeated time
        int repeat_data_collection = 20;
        // this virable will have the position of the motors
int run_array [30][2] = {{0,-0},
                         {1,-1},
                         {2,-2},
                         {3,-3},
                    		 {4,-4},
                    		 {5,-5},
                    		 {6,-6},
                    		 {-1,1},
                         {-2,2},
                         {-3,3}};

// global virable
Mat left_img, right_img;
Size sizee = Size(2048 , 1080);
//Size sizee = Size(1200,900);
float left_pan = 0,right_pan = 0;
float baseline_value = 0.0;
float baseline_value_old = 0.0;
// These virables are the value form the offline calibration
float angle_z = -0.425903209166667;
float angle_x = 0.537310137283333;
float old_angle_sum = 1.0;

float old_left_pan = 0,old_right_pan = 0;
void left_pan_callback(const std_msgs::Float64& val){
  left_pan = val.data;
}

void right_pan_callback(const std_msgs::Float64& val){

  right_pan = val.data;
}


void baseline_callback(const std_msgs::Float64& val){

  baseline_value = val.data / 1000;
}


ros::Publisher left_cam_pan_pub;
ros::Publisher right_cam_pan_pub;
void move_camera_to_located_position(int x, ros::NodeHandle nh);


int main(int argc,char** argv)
{

  ros::init(argc,argv,"MoveMotors");
  ros::NodeHandle nh;

  //define the publishers
  left_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/left/pan/move", 50);
  right_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/right/pan/move",50);
  // define the angle subscriper
  ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",10,right_pan_callback);
  ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",10,left_pan_callback);
  ros::Subscriber baseline_sub = nh.subscribe("/baseline/position",10,baseline_callback);
  ros::Duration(2).sleep();

  // start the while loop
  while(nh.ok()){

    int new_angle = -1;
    cout <<" Please enter the reading or enter -2 to exit: ";
    cin >> new_angle ;

    if (new_angle == -2){
      break;
    }
    move_camera_to_located_position( new_angle, nh);

  }// End of while loop

  return 1;

}


void move_camera_to_located_position(int x, ros::NodeHandle nh){
  //Define virables to store the value of joints
  geometry_msgs::Vector3 lcam_pan, rcam_pan;
  rcam_pan.y = 0;
  rcam_pan.z = 0;
  lcam_pan.y = 0;
  lcam_pan.z = 0;

  while(nh.ok()){
     ros::spinOnce();
     float left_angle = run_array[x][0];
      float right_angle = run_array[x][1];

      lcam_pan.x = left_angle;
      rcam_pan.x = right_angle;

      // publish data
      right_cam_pan_pub.publish(rcam_pan);
      left_cam_pan_pub.publish(lcam_pan);

      ros::Duration(0.2).sleep(); // sleep for 0.8 a second
      // cout << "Right Difference: " << abs(right_pan - right_angle) <<endl;
      //cout << "left Difference: " << abs(left_pan - left_angle) <<endl;
      if(abs(left_pan - left_angle) <= 0.15){
          if(abs(right_pan - right_angle) <= 0.15){
          // ROS_INFO("Both camera set to the new position !!!");
          //ros::Duration(5).sleep();
          break;
          }
      }else{
         //cout<<left_angle<<"___"<<right_angle <<endl;
      }
    }
}
