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

#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>

#include "robustMatcher.h"

using namespace cv;
using namespace std;

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// global virable
Mat left_img, right_img;
Size sizee = Size(2048 , 1080);

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


/**
 * Structure from motion from 2 cameras, using farneback optical flow as the 'features'
 * No, this doesn't work on more than 2 cams, because that requires bundle adjustment, which
 * I'm still searching if there's an OpenCV implementation of it
 */
Mat sfm( Mat& img1, Mat& img2 , Mat cam_matrix[2], Mat dist_coeff[2] ) {
    Mat gray1, gray2;
    cvtColor( img1, gray1, CV_BGR2GRAY );
    cvtColor( img2, gray2, CV_BGR2GRAY );

    /*  Find the optical flow using farneback dense algorithm
        Note that you might need to tune the parameters, especially window size.
        Smaller window size param, means more ambiguity when calculating the flow.
     */
    Mat flow_mat;
    calcOpticalFlowFarneback( gray1, gray2, flow_mat, 0.5, 3, 12, 3, 5, 1.2, 0 );

    vector<Point2f> left_points, right_points;
    for ( int y = 0; y < img1.rows; y+=6 ) {
        for ( int x = 0; x < img1.cols; x+=6 ) {
            /* Flow is basically the delta between left and right points */
            Point2f flow = flow_mat.at<Point2f>(y, x);

            /*  There's no need to calculate for every single point,
                if there's not much change, just ignore it
             */
            if( fabs(flow.x) < 0.1 && fabs(flow.y) < 0.1 )
                continue;

            left_points.push_back(  Point2f( x, y ) );
            right_points.push_back( Point2f( x + flow.x, y + flow.y ) );
        }
    }

    /* Undistort the points based on intrinsic params and dist coeff */
    undistortPoints( left_points, left_points, cam_matrix[0], dist_coeff[0] );
    undistortPoints( right_points, right_points, cam_matrix[1], dist_coeff[1] );

    /* Try to find essential matrix from the points */
    Mat fundamental = findFundamentalMat( left_points, right_points, FM_RANSAC, 3.0, 0.99 );
    Mat essential   = cam_matrix[0].t() * fundamental * cam_matrix[0];

    /* Find the projection matrix between those two images */
    SVD svd( essential );
    static const Mat W = (Mat_<double>(3, 3) <<
                         0, -1, 0,
                         1, 0, 0,
                         0, 0, 1);

    static const Mat W_inv = W.inv();

    Mat_<double> R1 = svd.u * W * svd.vt;
    Mat_<double> T1 = svd.u.col( 2 );

    Mat_<double> R2 = svd.u * W_inv * svd.vt;
    Mat_<double> T2 = -svd.u.col( 2 );

    static const Mat P1 = Mat::eye(3, 4, CV_64FC1 );
    Mat P2 =( Mat_<double>(3, 4) <<
             R1(0, 0), R1(0, 1), R1(0, 2), T1(0),
             R1(1, 0), R1(1, 1), R1(1, 2), T1(1),
             R1(2, 0), R1(2, 1), R1(2, 2), T1(2));

   // Mat P2 =( Mat_<double>(3, 4) <<
   //          R1(0, 0), R1(0, 1), R1(0, 2), 0.1,
   //          R1(1, 0), R1(1, 1), R1(1, 2), 0,
   //          R1(2, 0), R1(2, 1), R1(2, 2), 0);

    /*  Triangulate the points to find the 3D homogenous points in the world space
        Note that each column of the 'out' matrix corresponds to the 3d homogenous point
     */
    Mat out;
    triangulatePoints( P1, P2, left_points, right_points, out );

    /* Since it's homogenous (x, y, z, w) coord, divide by w to get (x, y, z, 1) */
    vector<Mat> splitted = {
        out.row(0) / out.row(3),
        out.row(1) / out.row(3),
        out.row(2) / out.row(3)
    };

    merge( splitted, out );

    return out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(Mat xyz)
 {
   pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new   pcl::PointCloud<pcl::PointXYZ>());
  //  Mat xyz;
  //  reprojectImageTo3D(disp, xyz, Q, false, CV_32F);
   pointcloud->is_dense = false;
   pcl::PointXYZ point;
   for (int i = 0; i < xyz.rows; ++i)
       {

           double* xyz_ptr = xyz.ptr<double>(i);

           for (int j = 0; j < xyz.cols; ++j)
           {

               Point3f p = xyz.at<Point3f>(i, j);

               point.z = p.z;   // I have also tried p.z/16
               point.x = p.x;
               point.y = p.y;
               // cout <<  p.x<< "," << p.y<< "," << p.z << endl;
               pointcloud->points.push_back(point);
           }
       }

    pointcloud->width = (int)pointcloud->points.size();
    pointcloud->height = 1;

     return pointcloud;

 }

int main(int argc,char** argv){
  ros::init(argc,argv,"SFM");
  ros::NodeHandle nh;
  //define the subscriber and publisher
  ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
  ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
  ros::Publisher pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_output",1);
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");


  // Load camera matric
  Mat cam_matrix[2];
  Mat dist_coeff[2];
  //Load the calibraton date from the raw dat
  string filename = "/home/abdulla/dev/Active-stereo-Vision-Platform/calibration_data/003_used_data.xml";
  FileStorage fr(filename, FileStorage::READ);
  fr["interinsic1"] >> cam_matrix[0];
  fr["interinsic2"] >> cam_matrix[1];
  fr["distCoeffs1"] >> dist_coeff[0];
  fr["distCoeffs2"] >> dist_coeff[1];
  fr.release();

  while(nh.ok()){
    ros::spinOnce();
    if(!left_img.empty() && !right_img.empty()){
      // Process Image
      Mat pointCloudMat = sfm( left_img, right_img , cam_matrix, dist_coeff );
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud =  MatToPoinXYZ(pointCloudMat);
      viewer.showCloud(pointcloud);
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*pointcloud, output);
      output.header.stamp = ros::Time::now();
      output.header.frame_id = "camera_link";
      pointCloud_pub.publish(output);
    }
  }
}


// int main(int argc,char** argv){
//   ros::init(argc,argv,"SFM");
//   ros::NodeHandle nh;
//   //define the subscriber and publisher
//   ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
//   ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
//   ros::Publisher pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_output",1);
//   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//
//   while(ros::ok()){
//     ros::spinOnce();
//     // Read input images
//     if(!left_img.empty() && !right_img.empty()){
//   cv::Mat image1= left_img;
//   cv::Mat image2= right_img;
// //    cv::Mat image =stf(image1,image1, true);
//   cv::waitKey(0);
//   cv::resize(image1,image1,cv::Size(800,600));
//   cv::resize(image2,image2,cv::Size(800,600));
//   cv::namedWindow("Matches",cv::WINDOW_OPENGL);
//   cv::imshow("Matches", image1);
//
//   // Prepare the matcher (with default parameters)
//   // here SIFT detector and descriptor
//   RobustMatcher rmatcher(cv::xfeatures2d::SIFT::create(250));
//
//
//   // Match the two images
//   std::vector<cv::DMatch> matches;
//
//   std::vector<cv::KeyPoint> keypoints1, keypoints2;
//   cv::Mat fundamental = rmatcher.match(image1, image2, matches,
//       keypoints1, keypoints2);
//
//   // draw the matches
//   cv::Mat imageMatches;
//   cv::drawMatches(image1, keypoints1,  // 1st image and its keypoints
//       image2, keypoints2,  // 2nd image and its keypoints
//       matches,			// the matches
//       imageMatches,		// the image produced
//       cv::Scalar(255, 255, 255),  // color of the lines
//       cv::Scalar(255, 255, 255),  // color of the keypoints
//       std::vector<char>(),
//       2);
//    cv::namedWindow("Matches",cv::WINDOW_OPENGL);
//    cv::imshow("Matches", imageMatches);
//
//   // Convert keypoints into Point2f
//   std::vector<cv::Point2f> points1, points2;
//
//   for (std::vector<cv::DMatch>::const_iterator it = matches.begin();
//   it != matches.end(); ++it) {
//
//       // Get the position of left keypoints
//       float x = keypoints1[it->queryIdx].pt.x;
//       float y = keypoints1[it->queryIdx].pt.y;
//       points1.push_back(keypoints1[it->queryIdx].pt);
//       // Get the position of right keypoints
//       x = keypoints2[it->trainIdx].pt.x;
//       y = keypoints2[it->trainIdx].pt.y;
//       points2.push_back(keypoints2[it->trainIdx].pt);
//   }
//
//   // Compute homographic rectification
//   cv::Mat h1, h2;
//   cv::stereoRectifyUncalibrated(points1, points2, fundamental, image1.size(), h1, h2);
//
//   // Rectify the images through warping
//   cv::Mat rectified1;
//   cv::warpPerspective(image1, rectified1, h1, image1.size());
//   cv::Mat rectified2;
//   cv::warpPerspective(image2, rectified2, h2, image1.size());
//   // Display the images
// //    cv::namedWindow("Left Rectified Image",cv::WINDOW_OPENGL);
// //    cv::imshow("Left Rectified Image", rectified1);
// //    cv::namedWindow("Right Rectified Image",cv::WINDOW_OPENGL);
// //    cv::imshow("Right Rectified Image", rectified2);
//
//   points1.clear();
//   points2.clear();
//   for (int i = 20; i < image1.rows - 20; i += 20) {
//
//       points1.push_back(cv::Point(image1.cols / 2, i));
//       points2.push_back(cv::Point(image2.cols / 2, i));
//   }
//
//   // Draw the epipolar lines
//   std::vector<cv::Vec3f> lines1;
//   cv::computeCorrespondEpilines(points1, 1, fundamental, lines1);
//
//   for (std::vector<cv::Vec3f>::const_iterator it = lines1.begin();
//   it != lines1.end(); ++it) {
//
//       cv::line(image2, cv::Point(0, -(*it)[2] / (*it)[1]),
//           cv::Point(image2.cols, -((*it)[2] + (*it)[0] * image2.cols) / (*it)[1]),
//           cv::Scalar(255, 255, 255));
//   }
//
//   std::vector<cv::Vec3f> lines2;
//   cv::computeCorrespondEpilines(points2, 2, fundamental, lines2);
//
//   for (std::vector<cv::Vec3f>::const_iterator it = lines2.begin();
//   it != lines2.end(); ++it) {
//
//       cv::line(image1, cv::Point(0, -(*it)[2] / (*it)[1]),
//           cv::Point(image1.cols, -((*it)[2] + (*it)[0] * image1.cols) / (*it)[1]),
//           cv::Scalar(255, 255, 255));
//   }
//
//   // Display the images with epipolar lines
// //    cv::namedWindow("Left Epilines",cv::WINDOW_OPENGL);
// //    cv::imshow("Left Epilines", image1);
// //    cv::namedWindow("Right Epilines",cv::WINDOW_OPENGL);
// //    cv::imshow("Right Epilines", image2);
//
//   // draw the pair
//   cv::drawMatches(image1, keypoints1,  // 1st image
//       image2, keypoints2,              // 2nd image
//       std::vector<cv::DMatch>(),
//       imageMatches,		             // the image produced
//       cv::Scalar(255, 255, 255),
//       cv::Scalar(255, 255, 255),
//       std::vector<char>(),
//       2);
//    cv::namedWindow("A Stereo pair",cv::WINDOW_OPENGL);
//    cv::imshow("A Stereo pair", imageMatches);
//
//   // Compute disparity
//   cv::Mat disparity;
//   cv::Ptr<cv::StereoMatcher> pStereo = cv::StereoSGBM::create(0,   // minimum disparity
//                                                               128,  // maximum disparity
//                                                               11);  // block size
//   pStereo->compute(rectified1, rectified2, disparity);
//
//   // draw the rectified pair
//
//   cv::warpPerspective(image1, rectified1, h1, image1.size());
//   cv::warpPerspective(image2, rectified2, h2, image1.size());
//   cv::drawMatches(rectified1, keypoints1,  // 1st image
//       rectified2, keypoints2,              // 2nd image
//       std::vector<cv::DMatch>(),
//       imageMatches,		                // the image produced
//       cv::Scalar(255, 255, 255),
//       cv::Scalar(255, 255, 255),
//       std::vector<char>(),
//       2);
//    cv::namedWindow("Rectified Stereo pair");
//    cv::imshow("Rectified Stereo pair", imageMatches);
//
//
//   double minv, maxv;
//   disparity = disparity * 64;
//   cv::minMaxLoc(disparity, &minv, &maxv);
//   std::cout << minv << "+" << maxv << std::endl;
//   // Display the disparity map
//   cv::namedWindow("Disparity Map",cv::WINDOW_OPENGL);
//   cv::imshow("Disparity Map", disparity);
//     }
//   }
// }
