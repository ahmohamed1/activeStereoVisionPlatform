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
#include <std_msgs/String.h>

#include <generate_disparity_map.h>


using namespace cv;
using namespace std;

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()


void featureExtractionAndMatching(Mat image1, Mat image2){
  std::vector<KeyPoint> keypts1, keypts2;
  Mat descriptor1, descriptor2;

  //Detect keypoints and exractORBdescriptors
  Ptr<Feature2D> = ORB::create(2000);
  orb->detectAndCompute(image1, noArray(), keypts1, descriptor1);
  orb->detectAndCompute(image2, noArray(), keypts2, descriptor2);

  // Matching
  exractORBdescriptorsPtr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<DMatch> matches;
  matcher->match(descriptor1, descriptor2, matches);

}

Mat computeEssentialMatrix(std::vector<KeyPoint> keypts1, keypts2, std::vector<DMatch> matches){
  std::vector<Point2f> image1_align, image2_align;
  for(size_t i =0; i <matches.size(); i++){
    // queryIdx is image 1
    image1_align.push_back(keypts1[matches[i].queryIdx].pt)

    // trainIdx is image 1
    image2_align.push_back(keypts2[matches[i].trainIdx].pt)
  }

  //robust find the Essential Matrix
  Mat status;
  Mat E = findEssentialMat(image1_align,
                          image2_align,
                          focal,         //Focal length of the camera
                          pp,            //camera principle points
                          cv::RANSAC,   // RANSAC for a robust solutions
                          0.999,
                          1.0,
                          status);
  return E;
}

void decomposeEssentialMatrixIntoRotationTranslation(Mat essentialMatrix,
                                                     Mat rotationMatrix,
                                                     Mat translationMatrix,
                                                     vector<Point2f> image1_align,
                                                     vector<Point2f> image2_align){
  recoverPose(E, image1_align, image2_align, rotationMatrix, translationMatrix, focal, pp, mask);
}

int main(){

}
