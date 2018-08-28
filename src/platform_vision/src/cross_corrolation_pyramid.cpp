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
///////////////////////////////////////
// PID parameters
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

void fastMatchTemplate(cv::Mat& reference_image,  // The reference image
                       cv::Mat& template_image,  // The template image
                       cv::Mat& output_image,   // Template matching result
                       int maxlevel)   // Number of levels
{
    std::vector<cv::Mat> reference_image_list, template_image_list, results;

    // Build Gaussian pyramid
    cv::buildPyramid(reference_image, reference_image_list, maxlevel);
    cv::buildPyramid(template_image, template_image_list, maxlevel);

    cv::Mat ref, tpl, res;

    // Process each level
    for (int level = maxlevel; level >= 0; level--)
    {
        ref = reference_image_list[level];
        tpl = template_image_list[level];
        res = cv::Mat::zeros(ref.size() + cv::Size(1,1) - tpl.size(), CV_32FC1);

        if (level == maxlevel)
        {
            // On the smallest level, just perform regular template matching
            cv::matchTemplate(ref, tpl, res, CV_TM_CCORR_NORMED);
        }
        else
        {
            // On the next layers, template matching is performed on pre-defined
            // ROI areas.  We define the ROI using the template matching result
            // from the previous layer.

            cv::Mat mask;
            cv::pyrUp(results.back(), mask);

            cv::Mat mask8u;
            mask.convertTo(mask8u, CV_8U);
            // Find matches from previous layer
            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(mask8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

            // Use the contours to define region of interest and
            // perform template matching on the areas
            for (unsigned i = 0; i < contours.size(); i++)
            {
                cv::Rect r = cv::boundingRect(contours[i]);
                cv::matchTemplate(
                    ref(r + (tpl.size() - cv::Size(1,1))),
                    tpl,
                    res(r),
                    CV_TM_CCORR_NORMED
                );
            }
        }

        // Only keep good matches
        cv::threshold(res, res, 0.94, 1., CV_THRESH_TOZERO);
        results.push_back(res);
    }

    res.copyTo(output_image);
}

Size imageSize = Size(2048 , 1080);//Size(4096,2160);

int main(int argc,char** argv)
{


    ros::init(argc,argv,"Cross_corrolation_Left_master");
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


    namedWindow("image", WINDOW_NORMAL);
    resizeWindow("image",640,480);
    cv::moveWindow("image", 1000,20);
//    namedWindow("result_window",WINDOW_NORMAL);
//    resizeWindow("result_window",640,480);

    // namedWindow("left_image", WINDOW_NORMAL);
    // resizeWindow("left_image",640,480);

    int match_method = 5;
    // start the while loop
    while(nh.ok()){

        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey(1);
        right_img = rightImageSubClass.getImage();
        left_img = leftImageSubClass.getImage();
        if(!left_img.empty() && !right_img.empty()){

            //Step 1 - Resize the images and and convert them to gray
            Mat left_img_gray, right_img_gray;
            cvtColor(left_img,left_img_gray,CV_BGR2GRAY);
            cvtColor(right_img,right_img_gray, CV_BGR2GRAY);

            //Step 2 - create the templet and create the result image
            temp = left_img_gray(windowSizeRectangule);
            int result_cols =  right_img.cols - temp.cols + 1;
            int result_rows = right_img.rows - temp.rows + 1;
            result.create( result_cols, result_rows, CV_32FC1 );
            //Step 3 - Do the matching process and normalize the result
            cv::Mat output_image;
            fastMatchTemplate(right_img_gray, temp, output_image, 1);
            normalize(output_image, output_image, 0, 1, NORM_MINMAX, -1, Mat() );
            //Step 4 - Localizing the best matching with minMaxLoc
            double minVal; double maxVal; Point minLoc; Point maxLoc;
            Point matchLoc;

            minMaxLoc( output_image, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

            /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
              if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
                { matchLoc = minLoc; }
              else
                { matchLoc = maxLoc; }
            //Step 5 - show the image
            // rectangle( right_img, matchLoc, Point( matchLoc.x + temp.cols , matchLoc.y + temp.rows ), Scalar(0,255,0), 2, 8, 0 );
            //rectangle( result, matchLoc, Point( matchLoc.x + temp.cols , matchLoc.y + temp.rows ), Scalar::all(1), 2, 8, 0 );
            circle(right_img,Point( matchLoc.x + temp.cols/2 , matchLoc.y + temp.rows/2 ),5,Scalar(0,255,0),5);


            Mat tempRgb = left_img(windowSizeRectangule);
            imshow( "image", right_img );
            //imshow( "result_window", result );
            imshow( "tempRgb", tempRgb );

            // Step 6 - Move the motor Using the PID Controller by implementing the P controll
            ///Get the points if the templets
            Point2f temp_pos;
            temp_pos = Point( matchLoc.x + temp.cols/2 , matchLoc.y + temp.rows/2 );

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
