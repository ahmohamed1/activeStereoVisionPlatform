/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Abdulla H Mohamed
   Email: abdll1@hotmail.com
    Date: 19/08/2018        */

#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/extract_indices.h>

inline void PointCloudXYZRGBAtoXYZRGB(pcl::PointCloud<pcl::PointXYZRGBA>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
  out.width   = in.width;
  out.height  = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size (); i++)
  {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
    out.points[i].r = in.points[i].r;
    out.points[i].g = in.points[i].g;
    out.points[i].b = in.points[i].b;
  }
}
inline void PointCloudChangeColour(pcl::PointCloud<pcl::PointXYZRGB>& out, int ID){
  uint8_t colourList [10][3] = {{0,0,0}, {255,0,0},{0,255,0},{0,0,255},{255,255,255},{255,255,0},{255,0,255},{100,6,95},{6,255,158}, {255,32,69}};
  std::cout << "Pointcloud ID: " << ID << std::endl;
  for (size_t i = 0; i < out.points.size (); i++)
  {
    out.points[i].r = colourList[ID-1][0];
    out.points[i].g = colourList[ID-1][1];
    out.points[i].b = colourList[ID-1][2];
  }
}
inline void PointCloudXYZRGBAtoXYZ(pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZ>& out){
  out.width   = in.width;
  out.height  = in.height;
  out.points.resize(in.points.size());
  for (size_t i = 0; i < in.points.size (); i++)
  {
    out.points[i].x = in.points[i].x;
    out.points[i].y = in.points[i].y;
    out.points[i].z = in.points[i].z;
  }
}

/// This function to convert to cloud point
pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPoinXYZ(cv::Mat depth, cv::Mat colourImage, cv::Mat cameraMatrix) {


   double centerX = cameraMatrix.at<double>(0,2);
   double centerY = cameraMatrix.at<double>(1,2);
   double focal_x = cameraMatrix.at<double>(0,0);
   double focal_y = cameraMatrix.at<double>(1,1);
   double scaleFactorX = 1;
   double scaleFactorY = 1;
   // cout << centerX << "  " << centerY<<"  " << focal_x <<"  " << focal_y <<endl;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
   //  Mat xyz;
   //  reprojectImageTo3D(disp, xyz, Q, false, CV_32F);
   pointcloud->height = (uint32_t) depth.rows;
   pointcloud->width = (uint32_t) depth.cols;
   pointcloud->is_dense = false;
   pointcloud->points.resize(pointcloud->width * pointcloud->height);
   pointcloud->is_dense = false;
   pcl::PointXYZRGB point;
   for (int i = 0; i < depth.rows; ++i)
       {
           uchar* rgb_ptr = colourImage.ptr<uchar>(i);
           uchar* disp_ptr = depth.ptr<uchar>(i);
           for (int j = 0; j < depth.cols; ++j)
           {
               float d = depth.at<float>(i, j);;
               if (d == 0) continue;
               float p = depth.at<float>(i, j);

               point.x = (static_cast<float>(j) - centerX) * p / focal_x * 0.001;
               point.y = (static_cast<float>(i) - centerY) * p / focal_y * 0.001;
               point.z = p * 0.001;

               point.b = rgb_ptr[3 * j];
               point.g = rgb_ptr[3 * j + 1];
               point.r = rgb_ptr[3 * j + 2];
               pointcloud->points.push_back(point);
           }
       }
    pointcloud->width = (int)pointcloud->points.size();
    pointcloud->height = 1;
    return pointcloud;
 }


/// This function apply linear regression to the point clouds
pcl::PointIndices PCLColourLinearRegression(pcl::PointCloud<pcl::PointXYZRGB>& out, float probabilityThreshold = 0.7){
  pcl::PointIndices indices;
  for (size_t i = 0; i < out.points.size (); i++)
  {
    float probability = -0.00258638*((float)out.points[i].b) + -0.00602994*((float)out.points[i].g) + 0.00700861*((float)out.points[i].r) + 0.49576749971493594;
    // std::cout << (float)out.points[i].b << " " << (uint32_t)out.points[i].b << " "<<(float)out.points[i].r <<std::endl;
    if (probability < probabilityThreshold){
      // if the point less than the threshold add it to non-tomato to be removed
      indices.indices.push_back(i);
    }
  }
  return indices;
  }


  /// This function use to compute the sphere

  float ProbabilitySphere(pcl::PointCloud<pcl::PointXYZRGB>& PCL_input, Eigen::Vector4f centroid, float radise){
    float sum = 0;
    int pointNumber = 0;
    for (size_t i = 0; i < PCL_input.points.size (); i++)
    {
      float raduseOfPoint = sqrt( pow((((float)PCL_input.points[i].x) - centroid[0]), 2) +
                                  pow((((float)PCL_input.points[i].y) - centroid[1]), 2) +
                                  pow((((float)PCL_input.points[i].z) - centroid[2]), 2));
      sum+= pow(radise-raduseOfPoint,2);
      pointNumber += 1;
    }

    return sqrt(sum/pointNumber);
  }


// Keep sphere only
// #include <pcl/filters/model_outlier_removal.h>
// void detectSphereOnly(pcl::PointCloud<pcl::PointXYZ> cloud,
//                       pcl::PointCloud<pcl::PointXYZ>& cloud_sphere_filtered)
// {
//   // 2. filter sphere:
//   // 2.1 generate model:
//   // modelparameter for this sphere:
//   // position.x: 0, position.y: 0, position.z:0, radius: 1
//   pcl::ModelCoefficients sphere_coeff;
//   sphere_coeff.values.resize (4);
//   sphere_coeff.values[0] = 0;
//   sphere_coeff.values[1] = 0;
//   sphere_coeff.values[2] = 0;
//   sphere_coeff.values[3] = 0.08;
//
//   pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
//   sphere_filter.setModelCoefficients (sphere_coeff);
//   sphere_filter.setThreshold (0.05);
//   sphere_filter.setModelType (pcl::SACMODEL_SPHERE);
//   sphere_filter.setInputCloud (cloud);
//   sphere_filter.filter (*cloud_sphere_filtered);
// }
