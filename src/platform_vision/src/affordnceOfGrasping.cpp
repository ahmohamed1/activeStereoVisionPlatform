#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <string>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

using namespace std;

#include "include/pointCloudHelpFunctions.h"

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

//Reports the RMS displacement between nearest neighbors in a point cloud in mm (assuming clouds use units of meters).
//Will ignore correspondences with distances greater than max_range (given in meters).
double computeCloudRMS(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, double max_range){
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(target);

  double fitness_score = 0.0;

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < source->points.size (); ++i){
          //Avoid NaN points as they crash nn searches
          if(!pcl_isfinite((*source)[i].x)){
                  continue;
          }

          // Find its nearest neighbor in the target
          tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists);

          // Deal with occlusions (incomplete targets)
          if (nn_dists[0] <= max_range*max_range){
                  // Add to the fitness score
                  fitness_score += nn_dists[0];
                  nr++;
          }
  }

  if (nr > 0){
          return sqrt(fitness_score / nr)*1000.0;
  }else{
          return (std::numeric_limits<double>::max ());
  }
}


ros::Publisher pointcloud_publisher, RMS_Publisher;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundTrueth (new pcl::PointCloud<pcl::PointXYZRGB>);

void pointCloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_POINTS_msgs(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);
    PointCloudXYZRGBAtoXYZRGB(temp_cloud, *cloud_POINTS_msgs);

    /////////////////////////////////////////////
    // The point clouds we will be using

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZRGB>);  // ICP output point cloud

    int iterations = 1;  // Default number of ICP iterations

    pcl::console::TicToc time;
    time.tic ();

    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (cloud_POINTS_msgs);
    icp.setInputTarget (groundTrueth);
    icp.align (*cloud_icp);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    if (icp.hasConverged ())
    {
      std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
      std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
      // transformation_matrix = icp.getFinalTransformation ().cast<double>();
      // print4x4Matrix (transformation_matrix);
    }
    else
    {
      PCL_ERROR ("\nICP has not converged.\n");
    }


    // compute rms
    double RMS = computeCloudRMS(cloud_icp, groundTrueth, 0.01);
    cout << "RMS is =" << RMS <<endl;

    std_msgs::Float64 rms_msg;
    rms_msg.data = RMS;
    RMS_Publisher.publish(rms_msg);
    // Object to store the centroid coordinates.
  	Eigen::Vector4f centroid;
  	pcl::compute3DCentroid(*cloud_POINTS_msgs, centroid);

    ///////////////////////////////////////////////
    // sensor_msgs::PointCloud2::Ptr pc2_clusters;
    // sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
    // pcl::toROSMsg(*cloud_icp, *pc2_clusters);
    //
    // pc2_clusters.header.frame_id = "depth_camera";
    // pointcloud_publisher.publish(pc2_clusters);
}


int main(int argc,char** argv)
{

  if (pcl::io::loadPLYFile (argv[1], *groundTrueth) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }

  ros::init(argc,argv,"pointCloud_processing");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, pointCloud_callback);
  // Create a ROS publisher for the output point cloud
  pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/segment_object", 1);
  RMS_Publisher = nh.advertise<std_msgs::Float64> ("/pointCloud_rms", 1);

  while (nh.ok()){
    ros::spinOnce();

    //Do nothing
  }
}
