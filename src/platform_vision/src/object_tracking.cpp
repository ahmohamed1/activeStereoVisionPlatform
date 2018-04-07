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

using namespace cv;
using namespace std;


cv::Mat img;


////////////////////////////////////////////////////////////////



int win_nam = 1;

string window_name = "image_";
string window_param = "parameters_";
string window_thresholder = "threshold_";

struct color_thresh{
int Hmin;
int Hmax;
int Smin;
int Smax;
int Vmin;
int Vmax;
};

int Hmin = 0;
int Hmax = 15;
int Smin = 0;
int Smax = 255;
int Vmin = 0;
int Vmax = 255;


void img_callback(const sensor_msgs::ImageConstPtr& img_msg){

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


    img = cv_img_msg->image;



}


Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(5, 5));
Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(5,5));

///
geometry_msgs::Vector3 difference;
std_msgs::Bool motor_mode;

int main(int argc,char** argv)
{
    struct color_thresh red;
    red.Hmin = 0;
    red.Hmax = 101;
    red.Smin = 150;
    red.Smax = 239;
    red.Vmin = 40;
    red.Vmax = 217;

    struct color_thresh blue;
    blue.Hmin=34;
    blue.Hmax=110;
    blue.Smin=94;
    blue.Smax=220;
    blue.Vmin=128;
    blue.Vmax=255;
    blue.Hmin=34;

    ros::init(argc,argv,"object_tracking");
    ros::NodeHandle nh;

    //define the subscriber and publisher
    ros::Subscriber img_sub = nh.subscribe("/image_raw",10,img_callback);
    ros::Publisher angle_pub = nh.advertise<geometry_msgs::Vector3>("/move",10);

    if(nh.getParam("win_nam", win_nam)){

    ROS_INFO_STREAM("window parameter from param" << win_nam);
    int a = win_nam;
    stringstream ss;
    ss << a;
    string str = ss.str();

    window_name = "image_" + str;
    window_param = "parameters_"  + str;
    window_thresholder = "threshold_"  + str;
}



    namedWindow(window_param);
    namedWindow(window_name);
    namedWindow(window_thresholder);
    moveWindow(window_thresholder, 100, 100);
    moveWindow(window_param, 100, 100);
    cv::createTrackbar("Hmin",window_param,&Hmin,255);
    cv::createTrackbar("Hmax",window_param,&Hmax,255);
    cv::createTrackbar("Smin",window_param,&Smin,255);
    cv::createTrackbar("Smax",window_param,&Smax,255);
    cv::createTrackbar("Vmin",window_param,&Vmin,255);
    cv::createTrackbar("Vmax",window_param,&Vmax,255);

    int cof = 209;
     cv::createTrackbar("coffi",window_param,&cof,1000);
    //set the motor mode to wheel mode
    difference.x = 0.0;
    difference.z = 1.0;
    while(nh.ok()){
        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey('q');
        if(!img.empty()){
            //here do the image process
            //track a circle
            cv::Mat dis_img;

            // convert image to gray
            cv::resize(img,img,cv::Size(800,600));
            cv::cvtColor(img,dis_img,CV_BGR2HSV);
            //cv::inRange(dis_img,Scalar(red.Hmin,red.Smin,red.Vmin),Scalar(red.Hmax,red.Smax,red.Vmax),dis_img);
            cv::inRange(dis_img,Scalar( Hmin, Smin, Vmin),Scalar( Hmax, Smax, Vmax),dis_img);
            //remove noise from image
            erode(dis_img,dis_img,erodeElmt);
            dilate(dis_img,dis_img,dilateElmt);


            imshow(window_thresholder,dis_img);

           // Find the centroid of the object
           // Tacke the momentom
           Moments mu;
           mu = moments(dis_img, false);

           Point CenterPt = Point((mu.m10 / mu.m00),  (mu.m01 / mu.m00));

           //Draw a circle in the original image and show it
           circle(img,CenterPt,50,Scalar(0,255,0),1,8,0);

        // calculate the differences
           float diff = ((dis_img.cols/2) - (mu.m10 / mu.m00))*0.06;
           if(std::abs(diff) > 2){
           difference.y = diff * cof /10;

           }else
           {
               difference.y = 0;
           }
           angle_pub.publish(difference);

           imshow(window_name,img);

        if(ikey =='q'){
            break;
        }
        }
    }

    difference.x = 0.0;
    difference.y = 0.0;
    difference.z = 0.0;
angle_pub.publish(difference);

return 1;


}
