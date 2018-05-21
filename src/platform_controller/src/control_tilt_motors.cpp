#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

#define PI 3.14159265359

using namespace std;

class ControlTiltMotors{

public:
  ControlTiltMotors(){
    FirstStart = true;
    ros::NodeHandle nh;
    cout << "---------------------------------\n" << endl;
    cout << "Start Tilt Motors Controller\n" << endl;
    cout << "---------------------------------\n" << endl;

    //define the subscriber and publisher
    rightMotorSubscribe = nh.subscribe("/right/tilt/move",5,&ControlTiltMotors::rightMotorCallback, this);
    leftMotorSubscribe = nh.subscribe("/left/tilt/move",5,&ControlTiltMotors::leftMotorCallback, this);
    BothMotorsSubscribe = nh.subscribe("/tilt_both_motor/move",5,&ControlTiltMotors::bothMotorCallback, this);
    MotorcurrentAngle = nh.subscribe("/joint_states",5,&ControlTiltMotors::MotorstatesCallback, this);

    rightMotorPublisher = nh.advertise<std_msgs::Float64>("/right_motor_tilt/command",5);
    leftMotorPublisher = nh.advertise<std_msgs::Float64>("/left_motor_tilt/command",5);

    rightMotorCurrentPublisher = nh.advertise<std_msgs::Float64>("/right/tilt/angle",5);
    leftMotorCurrentPublisher = nh.advertise<std_msgs::Float64>("/left/tilt/angle",5);


    maximumAngle = 40;
    minimumAngle = -40;

  }

  void rightMotorCallback(const std_msgs::Float64 &msg){
    std_msgs::Float64 radAngle;
    radAngle.data = degree2radian(msg.data);
    rightMotorPublisher.publish(radAngle);
  }

  void leftMotorCallback(const std_msgs::Float64 &msg){
    std_msgs::Float64 radAngle;
    radAngle.data = degree2radian(msg.data);
    leftMotorPublisher.publish(radAngle);
  }

  void bothMotorCallback(const std_msgs::Float64 &msg){
    std_msgs::Float64 radAngle;
    if (msg.data < maximumAngle || msg.data > minimumAngle){
    radAngle.data = degree2radian(msg.data);
    rightMotorPublisher.publish(radAngle);
    // Move the left to the opisite direction
    radAngle.data = -radAngle.data;
    leftMotorPublisher.publish(radAngle);
  }

  }


  void MotorstatesCallback(const sensor_msgs::JointState &msg){
    std_msgs::Float64 rightAngle, leftAngle;

    rightAngle.data = radian2degree(msg.position[0]);
    leftAngle.data = radian2degree(msg.position[1]);

    rightMotorCurrentPublisher.publish(rightAngle);
    leftMotorCurrentPublisher.publish(leftAngle);

  }


  float degree2radian(float degree){
    return (degree * PI / 180);
  }

  float radian2degree(float radian){
    return (radian * 180 / PI);
  }

  void startController(){
    // 
    // if(FirstStart){
    //   ros::Duration(10).sleep();
    //   std_msgs::Float64 rightAngle;
    //   ROS_INFO("TILT MOTOR MOVE TO ZERO POSITION!!!");
    //   rightAngle.data = radian2degree(0);
    //   rightMotorCurrentPublisher.publish(rightAngle);
    //   leftMotorCurrentPublisher.publish(rightAngle);
    //   FirstStart = false;
    // }
    ros::spin();

  }
private:
  ros::Subscriber rightMotorSubscribe, leftMotorSubscribe;
  ros::Subscriber BothMotorsSubscribe, MotorcurrentAngle;
  ros::Publisher rightMotorPublisher, leftMotorPublisher;
  ros::Publisher rightMotorCurrentPublisher, leftMotorCurrentPublisher;
  bool FirstStart;
  float maximumAngle, minimumAngle;
};

int main(int argc, char **argv){
  ros::init(argc,argv,"ControlTiltMotors");
  ControlTiltMotors controlTiltMotors;
  controlTiltMotors.startController();
  printf("Exit");
  return 0;

}
