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

int main(int argc,char** argv)
{

    /////////////////////////////////////
    int             matchPercentage = 90;
    bool            findMultipleTargets = false;
    int             numMaxima = 2;
    int             numDownPyrs = 3;
    int             searchExpansion = 15;
    /////////////////////////////////////
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
    Point2f center_img = Point(sizee.width/2, sizee.height/2);
    Rect ROI = Rect(sizee.width/2 - (rectSize/2), sizee.height/2 - (rectSize/2), rectSize, rectSize);
    Mat result;


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

            //Step 1 - Resize the images and and convert them to gray
            vector<Point> foundPointsList;
            vector<double> confidencesList;
            //Step 2 - create the templet
            temp = left_img(windowSizeRectangule);
            //Step 3 - Do the matching process and normalize the result
            // perform the match
            if(!FastMatchTemplate(right_img,
                              temp,
                              &foundPointsList,
                              &confidencesList,
                              matchPercentage,
                              findMultipleTargets,
                              numMaxima,
                              numDownPyrs,
                              searchExpansion))
            {
                printf("\nERROR: Fast match template failed.\n");
                return 3;
            }

            Mat colorImage;

            // if the original is a grayscale image, convert it to color
            if(right_img.channels() == 1)
            {
                cvtColor(right_img, colorImage, CV_GRAY2RGB);
            }
            else
            {
                colorImage = right_img.clone();
            }

            // cout << "Points detected: " << foundPointsList.size() <<endl;
            DrawFoundTargets(&colorImage,
                             temp.size(),
                             foundPointsList,
                             confidencesList);

            // wait for both windows to be closed before releasing images
            colorImage = drawCross(colorImage);
            imshow(windowsNameString, colorImage);
            // Step 6 - Move the motor Using the PID Controller by implementing the P controll
            //Get the points if the templets
            Point2f temp_pos = center_img;
            if(foundPointsList.size() != 0){
              temp_pos = foundPointsList[0];
            }
            // cout << "TargetPose: " << temp_pos <<endl;
            ///calculate the differences between the templete pos and the center of image
            /// first check if there is any object in the image to move
            cv::Point2f difference = center_img - temp_pos;
            // std::cout<< "Difference: " << difference << endl;
            // calculate the integral
            motorController.movePanMotor(difference.x); // 0 to move the pan motor
            motorController.moveTiltMotor(difference.y); // 1 to move the tilt motor

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
