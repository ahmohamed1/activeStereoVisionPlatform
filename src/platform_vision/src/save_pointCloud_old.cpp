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

int numberOfFileSave = 0;
ros::Publisher pointcloud_publisher;
void pointCloud_callback(const sensor_msgs::PointCloud2& input)
{
    pcl::PointCloud<pcl::PointXYZ> colorCloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;


    // Convert to PCL data type
    pcl::fromROSMsg(input, colorCloud);

    stringstream ssl;
    ssl<<"/home/abdulla/pointCloud_data/Ball_120/001_1_meter/002_100/007_12_degree/"<< numberOfFileSave << ".ply";
    string dir_filename = ssl.str();
    ssl.str("");

    pcl::io::savePLYFileBinary(dir_filename, colorCloud);

    numberOfFileSave ++;
    ROS_INFO("%i Point cloud saved successfully!!", numberOfFileSave);
    ros::Duration(0.5).sleep(); // sleep for half a second

}


pcl::PointCloud<pcl::PointXYZ> groundTruthPointCloud;


int main(int argc,char** argv)
{

  pcl::PLYReader Reader;
  Reader.read("/home/abdulla/GroundTruthBall120.ply", groundTruthPointCloud);

  ros::init(argc,argv,"pointCloud_saver");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/segment_object", 1, pointCloud_callback);

  while (nh.ok()){
    ros::spinOnce();
    if(numberOfFileSave  >=  10){
      break;
    }
    ros::Duration(0.5).sleep(); // sleep for half a second
    //Do nothing
  }

  return 0;

}
