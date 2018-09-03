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
//Size sizee = Size(640,420);
Size sizee = Size(2048,1080);

float left_pan = 0,right_pan = 0;
float baseline_value = 0.0;
Size imageSize = Size(2048 , 1080);//Size(4096,2160);
Point2f windowsCenter(imageSize.width/2, imageSize.height/2);
bool updateTracker = false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        windowsCenter.x = x;
        windowsCenter.y = y;
        // cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        updateTracker = true;
    }

}


int main(int argc,char** argv)
{

    FastTemplateMatch gazeFastMatchTemplate;
    ros::init(argc,argv,"GazeController");
    string windowsNameString = "MasterCamera";
    ros::NodeHandle nh;
    int windowSize = 250;
    //define the subscriber and publisher
    GetImageClass leftImageSubClass(nh, "left");
    cv::Rect windowSizeRectangule = returnRectanguleSizeOfCenterImage(imageSize,windowSize);
    //Define the publisher
    MotorController motorController(nh, "left", true);
    motorController.moveToZero();
    Mat temp;

    namedWindow(windowsNameString, WINDOW_NORMAL);
    resizeWindow(windowsNameString,640,480);
    cv::moveWindow(windowsNameString, 1000,20);
    ros::Rate r(15); // 10 hz
    // start the while loop
    //set the callback function for any mouse event
    setMouseCallback(windowsNameString, CallBackFunc, NULL);
    while(nh.ok()){
        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey(1);
        left_img = leftImageSubClass.getImage();
        if(!left_img.empty() ){
          if(updateTracker){
            Rect2d bbox = returnRectanguleTemplate(windowsCenter, windowSize, left_img.size());
            // cout<<"bbox: "<< bbox.size() << endl;
            temp = left_img(bbox);
            updateTracker = false;
          }

          if (!temp.empty()){
            // temp = left_img(windowSizeRectangule);
            cv::Point2f difference;
            cv::Mat editedImage;
            // cout<<left_img.size() <<endl;
            tie(editedImage, difference) = gazeFastMatchTemplate.trackTargetPNCC(left_img, temp, 85);
            imshow(windowsNameString, editedImage);

            // std::cout<< "Difference: " << difference << endl;
            bool panState = motorController.movePanMotor(difference.x); // 0 to move the pan motor
            bool tiltState = motorController.moveTiltMotor(-difference.y); // 1 to move the tilt motor
            motorController.checkVergeCorrectely(panState,tiltState);
            char ikey = waitKey(1);

            if(ikey == 'q'){
                motorController.moveToZero();
                break;
            }
          }else{
            imshow(windowsNameString, left_img);
          }
      }// end of the if statment of the images
      r.sleep();
  }// End of while loop


return 1;


}
