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
#include "opencv2/ximgproc/disparity_filter.hpp"


#include <opencv2/core/cuda.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/cudafilters.hpp>

#include <generate_disparity_map.h>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// global virable
Mat left_img, right_img;
// the blow functions are the function use to get the values
void left_img_callback(const sensor_msgs::ImageConstPtr& img_msg){

    // create storage for the comming image in cv format
    cv_bridge::CvImagePtr cv_img_msg;

    //copy the image and save it in opencv formate
    try{
    cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::MONO8);
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
    cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::MONO8);
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

  Generate_disparity_map generate_disparity_map;
  //////////////////////////////////////
    ros::init(argc,argv,"DisparityMap");
    ros::NodeHandle nh;

    //define the subscriber and publisher
    ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_rect_mono",10,left_img_callback);
    ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_rect_mono",10,right_img_callback);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher disparity_image_pub = it.advertise("stereo/depth/disparity", 1);

    char ikey = '4';
    // start the while loop
    while(nh.ok()){

        ros::spinOnce();
        char ikey;
        if(!left_img.empty() && !right_img.empty()){
          Mat disp;
          disp = generate_disparity_map.Disparity(left_img, right_img, '2', true);
          generate_disparity_map.show_disparity_map(disp);
          ikey = waitKey(10);
          generate_disparity_map.check_key_press(ikey, left_img, right_img, disp);

        }// end of the if statment of the images

    }

return 1;

}
