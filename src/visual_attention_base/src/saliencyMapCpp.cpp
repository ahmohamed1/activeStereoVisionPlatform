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

#include <tuple>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;
#include "/home/abdulla/dev/activeStereoVisionPlatform/src/platform_vision/src/include/motorController.h"
#include "/home/abdulla/dev/activeStereoVisionPlatform/src/platform_vision/src/include/helpFunctions.h"
#include "/home/abdulla/dev/activeStereoVisionPlatform/src/platform_vision/src/include/FastMatchTemplate.h"

#include "include/visual_attention_base/IttiSaliencyMap.h"
#include "include/focus_of_attention.h"
////////////////////////////////////////////////////////

// global virable
Mat left_img, right_img;
Size imageSize = Size(2048 , 1080);//Size(4096,2160);
Point2f windowsCenter(imageSize.width/2, imageSize.height/2);
bool updateTracker = false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        windowsCenter.x = x;
        windowsCenter.y = y;
        // cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        updateTracker = true;
    }

}

////Gloables Names
string windowsNameString = "MasterCamera";
string saliencymapName = "SaliencyMap";


// Define saliency map  and gaze controller
FastTemplateMatch gazeFastMatchTemplate;
IttiSaliencyMap saliencyMap(0);
FocalOfAttentionBasedWatershed FOA(false);

tuple<cv::Mat ,bool> computeSaliencyMap(cv::Mat image, int windowSize, std::vector<cv::Mat> targetsList){
  cv::Mat imageSmaller;
  cv::resize(image,imageSmaller,cv::Size(640,420));
  // Compute saliency map
  cv::Mat SMImage = saliencyMap.ComputeSaliencyMap(imageSmaller);
  // Compute FOA
  // focalOfAttentionBasedWatershed.ComputeFOA(imgSM, left_imgSmaller);
  cv::Mat temp = FOA.FOASingleTarget(SMImage, image, windowSize);

  // check if the template has been visited
  if(targetsList.size() > 0){
    for(int i = 0; i < targetsList.size(); i++){
      float diff = FOA.computeDifferences(temp,targetsList[i]);
      // cout <<"diff: "<<  100 * diff << "%" << endl;
      if(100*diff < 40){
        cout<< 100 * diff << "% this target have similar feature to target " << i << endl;
      }
    }
  }else{
    // break;
  }
  cv::imshow("Target", temp);
  cv::imshow(saliencymapName, SMImage);

  return make_tuple(temp, false);
}

tuple<cv::Mat ,bool> computeSaliencyMapTest2(cv::Mat image, int windowSize, std::vector<cv::Mat> targetsList){
  cv::Mat editedImage;
  image.copyTo(editedImage);
  // check if the template has been visited
  if(targetsList.size() > 0){
    for(int i = 0; i < targetsList.size(); i++){
      cv::Rect _tempRect;
      cv::Point2f difference;
      cv::Mat _editedImage;
      int scalarDiff = 10;
      tie(_editedImage, difference, _tempRect) = gazeFastMatchTemplate.trackTargetPNCC(editedImage, targetsList[i], 85);
      rectangle(editedImage,
               cv::Point(_tempRect.x+scalarDiff, _tempRect.y+scalarDiff),
               cv::Point(_tempRect.width-scalarDiff, _tempRect.height-scalarDiff),
               cv::Scalar::all(0) , -1);
    }
  }

  cv::Mat imageSmaller;
  cv::resize(editedImage,imageSmaller,cv::Size(640,420));
  // Compute saliency map
  cv::Mat SMImage = saliencyMap.ComputeSaliencyMap(imageSmaller);
  // Compute FOA
  // focalOfAttentionBasedWatershed.ComputeFOA(imgSM, left_imgSmaller);
  cv::Mat temp = FOA.FOASingleTarget(SMImage, editedImage, windowSize);

  cv::imshow("Target", temp);
  cv::imshow(saliencymapName, SMImage);

  editedImage.release();
  SMImage.release();
  return make_tuple(temp, false);
}



int main(int argc,char** argv)
{
    /// which approche to use to verge on the target
    // approch = 1 use saliency map
    // approch = 2 use mouse
    //
    int approche = 1;

    int windowSize = 250;
    if (argc > 1){
      istringstream ss(argv[1]);
      if (!(ss >> windowSize)){
        std::cout << "No argument \n";
      }

      istringstream saa(argv[2]);
      if (!(saa >> approche)){
        std::cout << "No argument \n";
      }
  }else{
    std::cout << "No Argument "<< '\n';
  }

  std::cout << "approche " << approche << '\n';
  std::cout << "windowSize " << windowSize << '\n';




    std::vector<cv::Mat> targetList;
    bool motorState = true;

    ros::init(argc,argv,"SaliencyMap");

    ros::NodeHandle nh;

    //define the subscriber and publisher
    GetImageClass leftImageSubClass(nh, "left");
    cv::Rect windowSizeRectangule = returnRectanguleSizeOfCenterImage(imageSize,windowSize);
    //Define the publisher
    MotorController motorController(nh, "left", false);
    motorController.moveToZero();
    Mat temp;

    namedWindow(windowsNameString, WINDOW_NORMAL);
    resizeWindow(windowsNameString,640,420);
    cv::moveWindow(windowsNameString, 1000,20);

    cv::namedWindow(saliencymapName, WINDOW_NORMAL);
    cv::resizeWindow(saliencymapName, 640,420);
    ros::Rate r(15); // 10 hz
    // start the while loop
    //set the callback function for any mouse event
    setMouseCallback(windowsNameString, CallBackFunc, NULL);
    while(nh.ok()){
        ros::spinOnce();
        left_img = leftImageSubClass.getImage();
        if(!left_img.empty() ){

          // if SaliencyMap is used to gaze on the target
          if(motorState && approche == 1){
            tie(temp, motorState) = computeSaliencyMap(left_img, windowSize, targetList);
            // tie(temp, motorState) = computeSaliencyMapTest2(left_img, windowSize, targetList);
            targetList.push_back(temp);
          }

          // mouse click gaze controller
          if(updateTracker && approche == 2){
            Rect2d bbox = returnRectanguleTemplate(windowsCenter, windowSize, left_img.size());
            // cout<<"bbox: "<< bbox.size() << endl;
            temp = left_img(bbox);
            updateTracker = false;
          }

          if (!temp.empty()){
            // temp = left_img(windowSizeRectangule);
            cv::Point2f difference;
            cv::Mat editedImage;
            // cout<<left_img.size() <<endl;
            cv::Rect _tempRect;
            tie(editedImage, difference, _tempRect) = gazeFastMatchTemplate.trackTargetPNCC(left_img, temp, 90);
            imshow(windowsNameString, editedImage);

            // std::cout<< "Difference: " << difference << endl;
            bool panState = motorController.movePanMotor(difference.x); // 0 to move the pan motor
            bool tiltState = motorController.moveTiltMotor(-difference.y); // 1 to move the tilt motor
            motorState = motorController.checkVergeCorrectely(panState,tiltState);
            // cout<< "Motor state = " << motorState << endl;
          }else{
            imshow(windowsNameString, left_img);
          } // End of gaze controller if statment

      }// end of the if statment of the images
      char ikey;
      ikey = cv::waitKey(1);
      if(ikey == 'q'){
        motorController.moveToZero();
        break;
      }
      r.sleep();
  }// End of while loop

motorController.moveToZero();
return 1;


}
