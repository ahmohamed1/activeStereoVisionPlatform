#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include<std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"



using namespace cv;
using namespace cv::ximgproc;
using namespace std;
#include "include/motorController.h"
#include "include/helpFunctions.h"
#include "include/FastMatchTemplate.h"
// global virable
Mat left_img, right_img;

float left_pan = 0,right_pan = 0;
float baseline_value = 0.0;
Size imageSize = Size(2048 , 1080);//Size(4096,2160);



int main(int argc,char** argv)
{
    FastTemplateMatch vergFastMatchTemplate;
    ros::init(argc,argv,"VergenceController");
    string windowsNameString = "SlaveCamera";
    ros::NodeHandle nh;
    int windowSize = 250;
    //define the subscriber and publisher
    GetImageClass rightImageSubClass(nh, "right");
    GetImageClass leftImageSubClass(nh, "left");
    cv::Rect windowSizeRectangule = returnRectanguleSizeOfCenterImage(imageSize,windowSize);
    //Define the publisher
    MotorController motorController(nh, "right");
    motorController.moveToZero();
    Mat temp;
    int rectSize = 200; // this int describe the size of the templet multiply by 2
    Point2f center_img = Point(imageSize.width/2, imageSize.height/2);
    Rect ROI = Rect(imageSize.width/2 - (rectSize/2), imageSize.height/2 - (rectSize/2), rectSize, rectSize);


    namedWindow(windowsNameString, WINDOW_NORMAL);
    resizeWindow(windowsNameString,640,480);
    cv::moveWindow(windowsNameString, 1000,600);
    ros::Rate r(20); // 10 hz
    // start the while loop
    while(nh.ok()){
        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey(1);
        right_img = rightImageSubClass.getImage();
        left_img = leftImageSubClass.getImage();
        if(!left_img.empty() && !right_img.empty()){

          temp = left_img(windowSizeRectangule);
          cv::Point2f difference;
          cv::Mat editedImage;
          // cout<<left_img.size() <<endl;
          tie(editedImage, difference) = vergFastMatchTemplate.trackTargetPNCC(right_img, temp, 80);
          imshow(windowsNameString, editedImage);
          // std::cout<< "Difference: " << difference << endl;
          // calculate the integral
          bool panState = motorController.movePanMotor(difference.x); // 0 to move the pan motor
          bool tiltState = motorController.moveTiltMotor(difference.y); // 1 to move the tilt motor
          motorController.checkVergeCorrectely(panState,tiltState);
          //Step 7 - this step use to select the right parameters of the PID
          char ikey = waitKey(1);

          if(ikey == 'q'){
              motorController.moveToZero();
              break;
          }
      }// end of the if statment of the images
      r.sleep();
  }// End of while loop


return 1;


}
