#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
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

cv::Size templateSize2D(0,0);
bool updateTemplateSize2D = false;
void templateSize_callback(const geometry_msgs::Pose2D &tempSize){

  templateSize2D = cv::Size(tempSize.x, tempSize.y);
  updateTemplateSize2D = true;

}

int main(int argc,char** argv)
{

    int windowSize = 200;
    int confidenceSize = 65;
    if (argc > 1){
      istringstream ss(argv[1]);
      if (!(ss >> windowSize)){
        std::cout << "No argument \n";
      }

      istringstream saa(argv[2]);
      if (!(saa >> confidenceSize)){
        std::cout << "No argument \n";
      }else{
        cout<<"confidence tracking is " << confidenceSize <<endl;
      }
  }else{
    std::cout << "No Argument "<< '\n';
  }

    templateSize2D = cv::Size(windowSize,windowSize);
    ros::init(argc,argv,"VergenceController");
    ros::NodeHandle nh;
    ros::Subscriber templateSizeSubscriber = nh.subscribe("/templateSize" ,1, &templateSize_callback);

    FastTemplateMatch vergFastMatchTemplate;
    string windowsNameString = "SlaveCamera";

    //define the subscriber and publisher
    GetImageClass rightImageSubClass(nh, "right");
    GetImageClass leftImageSubClass(nh, "left");
    cv::Rect windowSizeRectangule = returnRectanguleSizeOfCenterImage2D(imageSize, templateSize2D);
    //Define the publisher
    MotorController motorController(nh, "right");
    motorController.moveToZero();

    // motorController.tiltGoto(10.0);

    namedWindow(windowsNameString, WINDOW_NORMAL);
    resizeWindow(windowsNameString,640,480);
    cv::moveWindow(windowsNameString, 1000,600);
    ros::Rate r(15); // 10 hz
    // start the while loop
    while(nh.ok()){
        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey(1);
        right_img = rightImageSubClass.getImage();
        left_img = leftImageSubClass.getImage();
        if(!left_img.empty() && !right_img.empty()){

          if(updateTemplateSize2D == true){
            windowSizeRectangule = returnRectanguleSizeOfCenterImage2D(imageSize, templateSize2D);
            updateTemplateSize2D = false;
          }
          cv::Mat temp = left_img(windowSizeRectangule);
          cv::Point2f difference;
          cv::Mat editedImage;
          // cout<<left_img.size() <<endl;
          cv::Rect _tempRect;
          tie(editedImage, difference, _tempRect) = vergFastMatchTemplate.trackTargetPNCC(right_img, temp, confidenceSize);
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
