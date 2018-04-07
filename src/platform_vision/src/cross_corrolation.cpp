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


//import dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <platform_vision/pidConfig.h>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

///////////////////////////////////////
// PID parameters
double int_error = 0;
double last_rime = 0, current_time = 0;
double Kp=0.2, Ki=0, Kd=0;
double dt = 100e-3;
double diff = 0;
double last_error = 0;

// this function use to call the dynamic parameters
void callback(platform_vision::pidConfig &config, uint32_t level) {

    //ROS_INFO("Reconfigure Request: %f %f %f",config.p, config.i, config.d);
    Kp = config.Kp;
    Ki = config.Ki;
    Kd = config.Kd;

}

//////////////////////////////////

// global virable
Mat left_img, right_img;
//Size sizee = Size(640,420);
Size sizee = Size(2048,1080);

float left_pan = 0,right_pan = 0;
float baseline_value = 0.0;
// These virables are the value form the offline calibration
float angle_z = -0.425903209166667;
float angle_x = 0.537310137283333;
float old_angle_sum = 1.0;
int BloackSize = 5;
int NoDisparity = 16;
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

    left_img = cv_img_msg->image;
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

    right_img = cv_img_msg->image;
}



int main(int argc,char** argv)
{


    ros::init(argc,argv,"Cross_corrolation_Left_master");
    ros::NodeHandle nh;

    //define the subscriber and publisher
    ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
    ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);

    //Define the publisher
    ros::Publisher right_motor_pub = nh.advertise< geometry_msgs::Vector3>("/right/pan/move",10);
    geometry_msgs::Vector3 right_move;
    right_move.y = 0.0;
    right_move.z = 2.0; // Set it to the rotating mode
    float kp = 0.8;

    Mat temp;
    int rectSize = 150; // this int describe the size of the templet multiply by 2
    Point2f center_img = Point(sizee.width/2, sizee.height/2);
    Rect ROI = Rect(sizee.width/2 - rectSize,sizee.height/2 - rectSize, rectSize, rectSize);
    Mat result;

    dynamic_reconfigure::Server<platform_vision::pidConfig> server;
    dynamic_reconfigure::Server<platform_vision::pidConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    namedWindow("image", WINDOW_NORMAL);
    resizeWindow("image",640,480);

//    namedWindow("result_window",WINDOW_NORMAL);
//    resizeWindow("result_window",640,480);

    namedWindow("left_image", WINDOW_NORMAL);
    resizeWindow("left_image",640,480);

    int match_method = 5;
    // start the while loop
    while(nh.ok()){

        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey('q');
        if(!left_img.empty() && !right_img.empty()){

            //Step 1 - Resize the images and and convert them to gray
            //resize(left_img,left_img,sizee);
            //resize(right_img,right_img,sizee);
            Mat left_img_gray, right_img_gray;
            cvtColor(left_img,left_img_gray,CV_BGR2GRAY);
            cvtColor(right_img,right_img_gray, CV_BGR2GRAY);

            //Step 2 - create the templet and create the result image
            temp = left_img_gray(ROI);
            int result_cols =  right_img.cols - temp.cols + 1;
            int result_rows = right_img.rows - temp.rows + 1;
            result.create( result_cols, result_rows, CV_32FC1 );

            //Step 3 - Do the matching process and normalize the result
            matchTemplate(right_img_gray,temp, result, match_method);
            normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat() );

            //Step 4 - Localizing the best matching with minMaxLoc
            double minVal; double maxVal; Point minLoc; Point maxLoc;
            Point matchLoc;

            minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

            /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
              if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
                { matchLoc = minLoc; }
              else
                { matchLoc = maxLoc; }
            //Step 5 - show the image
            rectangle( left_img, Point(ROI.x, ROI.y),Point(ROI.x+ROI.width, ROI.y+ROI.height), Scalar(0,255,0),2,8,0);
            rectangle( right_img, Point(ROI.x, ROI.y),Point(ROI.x+ROI.width, ROI.y+ROI.height), Scalar(0,255,0),2,8,0);
            rectangle( right_img, matchLoc, Point( matchLoc.x + temp.cols , matchLoc.y + temp.rows ), Scalar(0,255,0), 2, 8, 0 );
            //rectangle( result, matchLoc, Point( matchLoc.x + temp.cols , matchLoc.y + temp.rows ), Scalar::all(1), 2, 8, 0 );
            circle(right_img,Point( matchLoc.x + temp.cols/2 , matchLoc.y + temp.rows/2 ),5,Scalar(0,255,0),5);



            imshow( "image", right_img );
            //imshow( "result_window", result );
            imshow( "left_image", left_img );

            // Step 6 - Move the motor Using the PID Controller by implementing the P controll
            ///Get the points if the templets
            Point2f temp_pos;
            temp_pos = Point( matchLoc.x + temp.cols/2 , matchLoc.y + temp.rows/2 );

            ///calculate the differences between the templete pos and the center of image
            /// first check if there is any object in the image to move

            float diff_x = center_img.x - temp_pos.x;
            // calculate the integral
            int_error += diff * dt;
            double error = (diff_x * Kp) + (int_error * Ki) + (Kd * (diff_x - last_error));
            last_error = diff_x;

            if(abs(diff_x) != 2){
                right_move.y = error;
                right_motor_pub.publish(right_move);
            }else {// if there is no matching don't move the motor
                cout<<"center"<<endl;
                right_move.x = 0;
                right_move.y = 0;
                right_move.z = 2.0;
                right_motor_pub.publish(right_move);
            }
            //Step 7 - this step use to select the right parameters of the PID
            char ikey = waitKey(1);

            if(ikey == 'q'){
                //send the motor to the servo mode and return it to zero
                right_move.x = 0;
                right_move.y = 0;
                right_move.z = 0;
                right_motor_pub.publish(right_move);
                break;

            }else if(ikey == 'w'){
                kp = kp + 0.1;
                cout<<"Kp: "<<kp<<endl;
            }else if(ikey == 's'){
                kp = kp - 0.1;
                cout<<"Kp: "<<kp<<endl;
            }else if(ikey == 'e'){
                if(match_method == 6){
                    cout<<"match_method: "<<match_method<<endl;
                }else{
                    match_method += 1;
                    cout<<"match_method: "<<match_method<<endl;
                }
            }else if(ikey == 'd'){
                if(match_method == 0){
                    cout<<"match_method: "<<match_method<<endl;
                }else{
                    match_method -= 1;
                    cout<<"match_method: "<<match_method<<endl;
                }
            }


        }// end of the if statment of the images
    }


return 1;


}
