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

const string files_names [10] = {"001_0_degree" , "002_2_degree" , "003_4_degree" ,
                          "004_6_degree", "005_8_degree" , "006_10_degree" ,
                          "007_12_degree", "008_-2_degree" , "009_-4_degree" ,
                          "010_-6_degree"};

int numberOfFileSave = 0;

ros::Publisher pointcloud_publisher;
string direct_name;
void pointCloud_callback(const sensor_msgs::PointCloud2& input)
{
    pcl::PointCloud<pcl::PointXYZ> colorCloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;


    // Convert to PCL data type
    pcl::fromROSMsg(input, colorCloud);

    stringstream ssl;
    ssl<< direct_name << numberOfFileSave << ".ply";
    string dir_filename = ssl.str();
    ssl.str("");

    pcl::io::savePLYFileBinary(dir_filename, colorCloud);

    numberOfFileSave ++;
    ROS_INFO("%i Point cloud saved successfully!!", numberOfFileSave);
    // ROS_INFO("%s", direct_name.c_str() );
    ros::Duration(0.2).sleep(); // sleep for half a second

}


pcl::PointCloud<pcl::PointXYZ> groundTruthPointCloud;

bool change_name = true;
int main(int argc,char** argv)
{

  ros::init(argc,argv,"pointCloud_saver");
  ros::NodeHandle nh;

  //n is in the node's namespace
  int folder_number = -1;
  // ros::NodeHandle n("~");
  // n.getParam("No", folder_number);
  // string folder_name = files_names[folder_number];
  // ROS_INFO("calibration file loaded was %s", folder_number );
  // direct_name = "/home/abdulla/pointCloud_data/Ball_120/001_1_meter/003_150/" + folder_name + "/";
  ROS_INFO("%s", direct_name.c_str() );
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/segment_object", 1, pointCloud_callback);

  while (nh.ok()){
    ros::spinOnce();
    if(numberOfFileSave  >=  10 && change_name == false){
      change_name = true;
    }

    if(change_name == true){
      cout << "Enter filder number: ";
      cin >> folder_number ;
      if(folder_number == -2){
        break;
      }
      string folder_name = files_names[folder_number];
      // ROS_INFO("calibration file loaded was %s", folder_number );
      direct_name = "/home/abdulla/pointCloud_data/Cone/001_1_meter/001_55/" + folder_name + "/";
      cout << "one  " << endl;
      ROS_INFO("%s", direct_name.c_str() );
      cout << "two  " << endl;
      numberOfFileSave = 0;
      change_name = false;
    }
    // ros::Duration(0.2).sleep(); // sleep for half a second
    //Do nothing
  }

  return 0;

}
