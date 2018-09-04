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

using namespace cv;
using namespace std;

#include "include/helpFunctions.h"

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// global virable
Mat left_img, right_img;
Size sizee = Size(2048 , 1080);

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
    /*calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg, InputArray prevPts,
                           InputOutputArray nextPts, OutputArray status, OutputArray err,
                           Size winSize=Size(21,21), int maxLevel=3,
                           TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
                           int flags=0, double minEigThreshold=1e-4 )
    */
    calcOpticalFlowFarneback(gray1, gray2, flow_mat, 0.5, 3, 12, 3, 5, 1.2, 0 );

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
  //define the subscriber and publisher
  GetImageClass rightImageSubClass(nh, "right");
  GetImageClass leftImageSubClass(nh, "left");

  ros::Publisher pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_output",1);
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");


  // Load camera matric
  Mat cam_matrix[2];
  Mat dist_coeff[2];
  //Load the calibraton date from the raw dat
  string filename = "/home/abdulla/dev/activeStereoVisionPlatform/src/platform_vision/calibration/003_used_data.xml";
  FileStorage fr(filename, FileStorage::READ);
  fr["interinsic1"] >> cam_matrix[0];
  fr["interinsic2"] >> cam_matrix[1];
  fr["distCoeffs1"] >> dist_coeff[0];
  fr["distCoeffs2"] >> dist_coeff[1];
  fr.release();

  while(nh.ok()){
    ros::spinOnce();
    right_img = rightImageSubClass.getImage();
    left_img = leftImageSubClass.getImage();
    if(!left_img.empty() && !right_img.empty()){
      // Process Image
      Mat pointCloudMat = sfm( left_img, right_img , cam_matrix, dist_coeff);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud =  MatToPoinXYZ(pointCloudMat);
      viewer.showCloud(pointcloud);
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*pointcloud, output);
      output.header.stamp = ros::Time::now();
      output.header.frame_id = "camera_link";
      pointCloud_pub.publish(output);
    }
  }

  return 0;
}
