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

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()


ros::Publisher pointcloud_publisher;
void pointCloud_callback(const sensor_msgs::PointCloud2& input)
{
    pcl::PointCloud<pcl::PointXYZ> colorCloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;


    // Convert to PCL data type
    pcl::fromROSMsg(input, colorCloud);
    // pcl::PointCloud<pcl::PointXYZ> cloud(colorCloud.x, colorCloud.y, colorCloud.z);


    // Create the filtering object
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud (cloud_filtered.makeShared());
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (0.5);
    // sor.filter (cloud_filtered);


    //Preforme a passthrough filter to work with a range of the depth
    pcl::PassThrough<pcl::PointXYZ> passZAxis;
    passZAxis.setInputCloud (colorCloud.makeShared());
    passZAxis.setFilterFieldName ("z");
    passZAxis.setFilterLimits (-0.5, 1.7);
    //pass.setFilterLimitsNegative (true);
    passZAxis.filter (cloud_filtered);

    // // //Preforme a passthrough filter to work with a range of the Y axis
    // pcl::PassThrough<pcl::PointXYZ> passYAxis;
    // passYAxis.setInputCloud (cloud_filtered.makeShared());
    // passYAxis.setFilterFieldName ("y");
    // passYAxis.setFilterLimits (-0.3, 0.25);
    // // passYAxis.setFilterLimitsNegative (true);
    // passYAxis.filter (cloud_filtered);


    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PointXYZ> vox_obj;
    vox_obj.setInputCloud (cloud_filtered.makeShared());
    vox_obj.setLeafSize (0.01f, 0.01f, 0.01f);
    vox_obj.filter(cloud_filtered);

    // pcl::io::savePCDFileASCII	("/home/abdulla/write_pcd_test.pcd",	cloud_filtered);
    pcl::io::savePLYFileBinary("/home/abdulla/write_pcd_test.ply", cloud_filtered);

    // Publish the data
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_filtered, output);
    output.header.frame_id = "camera_link";
    pointcloud_publisher.publish(output);
}


int main(int argc,char** argv)
{
  ros::init(argc,argv,"pointCloud_processing");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, pointCloud_callback);
  // Create a ROS publisher for the output point cloud
  pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/segment_object", 1);

  while (nh.ok()){
    ros::spinOnce();

    //Do nothing
  }
}
