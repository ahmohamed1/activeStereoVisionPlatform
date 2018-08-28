#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sstream>
#include <math.h>       /* exp */

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"

// Local headers
#include "include/featurematchingbasedcuda.h"
#include "include/motorController.h"
#include "include/helpFunctions.h"


using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cv::xfeatures2d;


Size imageSize = Size(2048 , 1080);//Size(4096,2160);
static void help()
{
    cout << "\nThis program demonstrates using SURF_CUDA features detector, descriptor extractor and BruteForceMatcher_CUDA" << endl;
    cout << "\nUsage:\n\tmatcher_simple_gpu --left <image1> --right <image2>" << endl;
}

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// global virable
Mat left_img, right_img;
Size sizee = Size(4096,2160);
// Size sizee = Size(2048 , 1080);

// void left_img_callback(const sensor_msgs::ImageConstPtr& img_msg){
//     left_img =  convertROSMat2OpencvMat(img_msg);
// }

// void right_img_callback(const sensor_msgs::ImageConstPtr& img_msg){
//     right_img =  convertROSMat2OpencvMat(img_msg);
// }


int main(int argc,char** argv)
{

  cv::namedWindow("slave image", cv::WINDOW_NORMAL);
  cv::resizeWindow("slave image", 900, 600);

  int windowSize = 250;
  int algorithm = 1;
  if (argc > 1){
    istringstream ss(argv[1]);
    if (!(ss >> windowSize)){
      std::cout << "No argument \n";
    }else{
      std::cout << "windowSize " << windowSize << '\n';
    }

    istringstream saa(argv[2]);
    if (!(saa >> algorithm)){
      std::cout << "No argument \n";
    }else{
      std::cout << "algorithm " << algorithm << '\n';
    }
}else{
  std::cout << "No Argument "<< '\n';
}

  //////////////////////////////////////
  ros::init(argc,argv,"FeatureTrackingCuda");
  ros::NodeHandle nh;

  //define the subscriber and publisher
  // ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
  // ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
  GetImageClass rightImageSubClass(nh, "right");
  GetImageClass leftImageSubClass(nh, "left");
  cv::Rect windowSizeRectangule = returnRectanguleSizeOfCenterImage(imageSize,windowSize);
  MotorController motorController(nh, "right");
  motorController.moveToZero();
  ros::Rate r(20); // 10 hz
  while(nh.ok()){
    ros::spinOnce();
    char ikey;
    ikey = cv::waitKey(1);
    right_img = rightImageSubClass.getImage();
    left_img = leftImageSubClass.getImage();

    if(!left_img.empty() && !right_img.empty()){
      // imshow("master Image", right_img);
      // cv::waitKey(3);
      Mat MasterImage = left_img(windowSizeRectangule);
      bool state;
      cv::Point2f targetPosition;

      tie (state, targetPosition) = whichAlgorithm(right_img, MasterImage,algorithm);
      // Computet the differences for the image size  and move the motors
      cv::Point2f difference = motorController.converteToImageCoordinate(imageSize, targetPosition);
      // cout << difference <<endl;
      motorController.movePanMotor(difference.x); // 0 to move the pan motor
      motorController.moveTiltMotor(difference.y); // 1 to move the tilt motor
      int lineSize = 30;
      line(right_img, cv::Point((right_img.cols/2)-lineSize, right_img.rows/2), cv::Point((right_img.cols/2)+lineSize, right_img.rows/2), cv::Scalar(0,0,255), 2);  //crosshair horizontal
      line(right_img, cv::Point(right_img.cols/2, (right_img.rows/2)-lineSize), cv::Point(right_img.cols/2, (right_img.rows/2)+lineSize), cv::Scalar(0,0,255), 2);  //crosshair vertical
      cv::imshow("slave image", right_img);
      cv::waitKey(1);
      if (!state){
        break;
      }
      // MasterImage.release();
    }
    r.sleep();
  }
  std::cout<< "Return To Zero Position !!" << std::endl;
  motorController.moveToZero();
  return 1;
}
