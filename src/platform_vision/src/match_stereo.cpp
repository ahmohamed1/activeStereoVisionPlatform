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
#include <opencv2/objdetect.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>
#include "robustMatcher.h"



using namespace cv;
using namespace std;



// global virable
Mat left_img, right_img;
Size sizee = Size(4096,2160);

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

 float old_left_pan = 0,old_right_pan = 0;
void left_pan_callback(const std_msgs::Float64& val){

  left_pan = val.data;


}

void right_pan_callback(const std_msgs::Float64& val){

  right_pan = val.data;
}


void baseline_callback(const std_msgs::Float64& val){

  baseline_value = val.data;
}

int main(int argc,char** argv)
{

    ros::init(argc,argv,"StereoMatchingBaseFeatures");
    ros::NodeHandle nh;

    //define the subscriber and publisher
    ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
    ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
    // define the angle subscriper
    ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",10,right_pan_callback);
    ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",10,left_pan_callback);
    ros::Subscriber baseline_sub = nh.subscribe("/baseline/position",10,baseline_callback);

    float old_config_sum = 10;
    cv::Mat h1, h2;

    // start the while loop
    while(nh.ok()){

        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey('q');
        if(!left_img.empty() && !right_img.empty()){

            // Read input images
            cv::Mat image1= left_img;
            cv::Mat image2= right_img;
            if (!image1.data || !image2.data)
               { std::cout << "image fail"<<std::endl;

                return 0;
            }
            cv::resize(image1,image1,cv::Size(800,600));
            cv::resize(image2,image2,cv::Size(800,600));

            // if the configuratrion change update the matcher
            float config_sum = right_pan + left_pan +baseline_value;
            if (config_sum != old_config_sum){
                // Prepare the matcher (with default parameters)
                // here SIFT detector and descriptor
                RobustMatcher rmatcher(cv::xfeatures2d::SIFT::create(250));


                // Match the two images
                std::vector<cv::DMatch> matches;

                std::vector<cv::KeyPoint> keypoints1, keypoints2;
                cv::Mat fundamental = rmatcher.match(image1, image2, matches,
                    keypoints1, keypoints2);

                // draw the matches
                cv::Mat imageMatches;
                cv::drawMatches(image1, keypoints1,  // 1st image and its keypoints
                    image2, keypoints2,  // 2nd image and its keypoints
                    matches,			// the matches
                    imageMatches,		// the image produced
                    cv::Scalar(255, 255, 255),  // color of the lines
                    cv::Scalar(255, 255, 255),  // color of the keypoints
                    std::vector<char>(),
                    2);
               /* cv::namedWindow("Matches",cv::WINDOW_OPENGL);
                cv::imshow("Matches", imageMatches);
    */
                // Convert keypoints into Point2f
                std::vector<cv::Point2f> points1, points2;

                for (std::vector<cv::DMatch>::const_iterator it = matches.begin();
                it != matches.end(); ++it) {

                    // Get the position of left keypoints
                    float x = keypoints1[it->queryIdx].pt.x;
                    float y = keypoints1[it->queryIdx].pt.y;
                    points1.push_back(keypoints1[it->queryIdx].pt);
                    // Get the position of right keypoints
                    x = keypoints2[it->trainIdx].pt.x;
                    y = keypoints2[it->trainIdx].pt.y;
                    points2.push_back(keypoints2[it->trainIdx].pt);
                }

                // Compute homographic rectification

                cv::stereoRectifyUncalibrated(points1, points2, fundamental, image1.size(), h1, h2);
                old_config_sum =config_sum;

                points1.clear();
                points2.clear();
}
            // Rectify the images through warping
            cv::Mat rectified1;
            cv::Mat rectified2;

            cv::warpPerspective(image1, rectified1, h1, image1.size());
            cv::warpPerspective(image2, rectified2, h2, image1.size());
            // Display the images
          /*  cv::namedWindow("Left Rectified Image",cv::WINDOW_OPENGL);
            cv::imshow("Left Rectified Image", rectified1);
            cv::namedWindow("Right Rectified Image",cv::WINDOW_OPENGL);
            cv::imshow("Right Rectified Image", rectified2);
*/
/*
            for (int i = 20; i < image1.rows - 20; i += 20) {

                points1.push_back(cv::Point(image1.cols / 2, i));
                points2.push_back(cv::Point(image2.cols / 2, i));
            }

         // Draw the epipolar lines
            std::vector<cv::Vec3f> lines1;
            cv::computeCorrespondEpilines(points1, 1, fundamental, lines1);
            for (std::vector<cv::Vec3f>::const_iterator it = lines1.begin();
            it != lines1.end(); ++it) {

                cv::line(image2, cv::Point(0, -(*it)[2] / (*it)[1]),
                    cv::Point(image2.cols, -((*it)[2] + (*it)[0] * image2.cols) / (*it)[1]),
                    cv::Scalar(255, 255, 255));
            }

            std::vector<cv::Vec3f> lines2;
            cv::computeCorrespondEpilines(points2, 2, fundamental, lines2);

            for (std::vector<cv::Vec3f>::const_iterator it = lines2.begin();
            it != lines2.end(); ++it) {

                cv::line(image1, cv::Point(0, -(*it)[2] / (*it)[1]),
                    cv::Point(image1.cols, -((*it)[2] + (*it)[0] * image1.cols) / (*it)[1]),
                    cv::Scalar(255, 255, 255));
            }
*/
            // Display the images with epipolar lines
           /* cv::namedWindow("Left Epilines",cv::WINDOW_AUTOSIZE);
            cv::imshow("Left Epilines", image1);
            cv::namedWindow("Right Epilines",cv::WINDOW_AUTOSIZE);
            cv::imshow("Right Epilines", image2);*/

         /*   // draw the pair
            cv::drawMatches(image1, keypoints1,  // 1st image
                image2, keypoints2,              // 2nd image
                std::vector<cv::DMatch>(),
                imageMatches,		             // the image produced
                cv::Scalar(255, 255, 255),
                cv::Scalar(255, 255, 255),
                std::vector<char>(),
                2);
            cv::namedWindow("A Stereo pair",cv::WINDOW_AUTOSIZE);
            cv::imshow("A Stereo pair", imageMatches);
*/
            // Compute disparity
            cv::Mat disparity;
            cv::Ptr<cv::StereoMatcher> pStereo = cv::StereoSGBM::create(0,   // minimum disparity
                                                                        64,  // maximum disparity
                                                                        7);  // block size
            pStereo->compute(rectified1, rectified2, disparity);

            // draw the rectified pair
            /*
            cv::warpPerspective(image1, rectified1, h1, image1.size());
            cv::warpPerspective(image2, rectified2, h2, image1.size());
            cv::drawMatches(rectified1, keypoints1,  // 1st image
                rectified2, keypoints2,              // 2nd image
                std::vector<cv::DMatch>(),
                imageMatches,		                // the image produced
                cv::Scalar(255, 255, 255),
                cv::Scalar(255, 255, 255),
                std::vector<char>(),
                2);
            cv::namedWindow("Rectified Stereo pair");
            cv::imshow("Rectified Stereo pair", imageMatches);
            */

            double minv, maxv;
            disparity = disparity * 64;
            cv::minMaxLoc(disparity, &minv, &maxv);
            std::cout << minv << "+" << maxv << std::endl;
            // Display the disparity map
            cv::namedWindow("Disparity Map",cv::WINDOW_AUTOSIZE);
            cv::imshow("Disparity Map", disparity);
            char ikey = waitKey('q');
            if(ikey =='q'){
                break;
            }
        }
    }


return 1;


}
