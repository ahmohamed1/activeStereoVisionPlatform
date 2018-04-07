#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>

using namespace std;


float angle = 0;
float time_val = 0;
string file_name = "test";
void angle_callback(const std_msgs::Float64& value){
  angle = value.data;
}

void time_callback(const std_msgs::Float64& value){
  time_val = value.data;
}


int main(int argc, char** argv){
  ros::init(argc,argv,"save_data");
  ros::NodeHandle nh;
  nh.getParam("/file_name",file_name);
  ros::Subscriber angle_sub = nh.subscribe("/angle",10,angle_callback);
  ros::Subscriber time_sub = nh.subscribe("time",10,time_callback);

  // open new file
  ofstream file;


  string dir_name = "/home/abdulla/ros_bag_recorder/" + file_name + ".txt";
  cout<< "File save to : " <<dir_name<<endl;
  file.open("/home/abdulla/ros_bag_recorder/test1.txt");

  double rate = 10.0;
  ros::Rate r(rate);

  while(nh.ok())
    {
       ros::spinOnce();
       file << angle << ","<< time_val <<"\n";
       //cout<<"Angle at : "<< angle<<endl;
       r.sleep();
    }
  file.close();
}
