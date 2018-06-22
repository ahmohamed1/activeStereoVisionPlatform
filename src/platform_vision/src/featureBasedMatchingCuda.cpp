#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sstream>


#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"
#include "include/featurematchingbasedcuda.h"

using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cv::xfeatures2d;


Size imageSize = Size(2048 , 1080);
static void help()
{
    cout << "\nThis program demonstrates using SURF_CUDA features detector, descriptor extractor and BruteForceMatcher_CUDA" << endl;
    cout << "\nUsage:\n\tmatcher_simple_gpu --left <image1> --right <image2>" << endl;
}

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// global virable
Mat left_img, right_img;
//Size sizee = Size(4096,2160);
Size sizee = Size(2048 , 1080);
//Size sizee = Size(1200,900);
// the blow functions are the function use to get the values
void left_img_callback(const sensor_msgs::ImageConstPtr& img_msg){

    // create storage for the comming image in cv format
    cv_bridge::CvImagePtr cv_img_msg;

    //copy the image and save it in opencv formate
    try{
    cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR("cv_bridge expection: %s", e.what());
        return;
    }

    // left_img =  equalize_image_using_histograme(cv_img_msg->image);
    left_img =  cv_img_msg->image;
}

void right_img_callback(const sensor_msgs::ImageConstPtr& img_msg){

    // create storage for the comming image in cv format
    cv_bridge::CvImagePtr cv_img_msg;

    //copy the image and save it in opencv formate
    try{
    cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR("cv_bridge expection: %s", e.what());
        return;
    }

    // right_img =  equalize_image_using_histograme(cv_img_msg->image);
    right_img =  cv_img_msg->image;
}



int main(int argc,char** argv)
{

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


  // cv::Point2f point1 = cv::Point2f((imageSize.height/2 - windowSize), (imageSize.width/2 - windowSize));
  // cv::Point2f point2 = cv::Point2f((imageSize.height/2 + windowSize), (imageSize.width/2 + windowSize));
  cv::Rect windowSizeRectangule;
  windowSizeRectangule.y = imageSize.height/2 - windowSize/2;
  windowSizeRectangule.x = imageSize.width/2 - windowSize/2;
  windowSizeRectangule.height = windowSize;
  windowSizeRectangule.width = windowSize;
  //////////////////////////////////////
  ros::init(argc,argv,"FeatureTrackingCuda");
  ros::NodeHandle nh;

  //define the subscriber and publisher
  ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
  ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);

  while(1){
    ros::spinOnce();
    char ikey;
    ikey = cv::waitKey('q');
    if(!left_img.empty() && !right_img.empty()){
      // imshow("master Image", right_img);
      cv::waitKey(3);
      Mat MasterImage = left_img(windowSizeRectangule);
      bool state;
      switch (algorithm) {
        case 0:
          state = processWithGpu(right_img, MasterImage, 100);
          break;
        case 1:
          state = ORBFullGpu(right_img, MasterImage);
          break;
        case 2:
          state = ORBGpuMatching(right_img, MasterImage);
          break;
        default:
          std::cout << "d";
          break;
      }

      if (!state){
        break;
      }
      MasterImage.release();
  }
}

return 1;


}
