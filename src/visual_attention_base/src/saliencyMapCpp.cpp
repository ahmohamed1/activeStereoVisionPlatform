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

bool vergenceStatus = false;
void vergeStatue_callback(const std_msgs::Bool &data){
  vergenceStatus = data.data;
}

geometry_msgs::Vector3 targetPose;
void targetPose_callback(const geometry_msgs::Vector3 &data){
  targetPose = data;
}
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
  // std::vector<SaliencyData> targets = FOA.ComputeFOA(imgSM, left_imgSmaller);
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
      tie(_editedImage, difference, _tempRect) = gazeFastMatchTemplate.trackTargetPNCC(editedImage, targetsList[i], 95);
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

cv::Rect extendTectangle(cv::Rect actualRect, float extandSize, cv::Size imageSize){
  float x1 = actualRect.x - extandSize;
  float y1 = actualRect.y - extandSize;
  float x2 = x1 + actualRect.width + extandSize*2;
  float y2 = y1 + actualRect.height + extandSize*2;
  float rectWidth = actualRect.width + extandSize*2;
  float rectHeight = actualRect.height + extandSize*2;
  if(x1 < 0){
    x1 = 0;
    x2 = rectWidth;
  }
  if(x2 > imageSize.width){
    x2 = imageSize.width;
    x1 = imageSize.width - rectWidth;
  }
  if(y1 < 0){
    y1 = 0;
    y2 = rectHeight;
  }
  if(y2 > imageSize.height){
    y2 = imageSize.height;
    y1 = imageSize.height - rectHeight;
  }

// cout << x1 << "," << y1 << "," << x2 <<","<< y2<<endl;
Rect2f bbox(x1, y1, x2-x1, y2-y1);
return bbox;
}


std::vector<cv::Mat> computeMultipleTargets(cv::Mat left_img){
  std::vector<cv::Mat> targetList;
  cv::Mat left_img_copy;
  left_img.copyTo(left_img_copy);
  cout<<"   Computing Saliency Map ======>"<<endl;
  cv::Mat imageSmaller;
  cv::resize(left_img,imageSmaller,cv::Size(640,420));
  // Compute saliency map
  cv::Mat SMImage = saliencyMap.ComputeSaliencyMap(imageSmaller);
  cv::imshow("SaliencyMap", SMImage);
  // Compute FOA
  std::vector<SaliencyData> data = FOA.ComputeFOA(SMImage, imageSmaller);
  for(int i =0; i < data.size(); i++){
    SaliencyData ScalaData = resizeSaliencyData(data[i], imageSmaller.size(), left_img.size());
    ScalaData.boundingBox = extendTectangle(ScalaData.boundingBox, 20, left_img.size());
    // cout << ScalaData.boundingBox <<endl;
    cv::Mat _temp = left_img(ScalaData.boundingBox);
    // cout<<"1"<<endl;
    targetList.push_back(_temp);
    cv::circle(left_img_copy, ScalaData.centerOfTarget, 10, cv::Scalar(0,255,0), -1);
    cv::rectangle(left_img_copy, cv::Rect(ScalaData.boundingBox) ,cv::Scalar(0,255,0), 5);
  }
  cout<<"   Total targets found = " << targetList.size() << endl;
  return targetList;
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
    std::vector<string> machineStateStatusList = {"ComputeSaliency", // 0
                                                  "GazeOnTarget",    // 1
                                                  "VergeOnTarget",   // 2
                                                  "3DPose",          // 3
                                                  "ComputePCL",      // 4
                                                  "PublishData"};    // 5
    string machineStateStatus = machineStateStatusList[0];
    ros::init(argc,argv,"SaliencyMap");

    ros::NodeHandle nh;

    //define the subscriber and publisher
    GetImageClass leftImageSubClass(nh, "left");
    ros::Subscriber vergeStatueSubscriber = nh.subscribe("/right/onTarget" ,1, &vergeStatue_callback);
    ros::Subscriber targetPoseSubscriber = nh.subscribe("/targetPose" ,1, &targetPose_callback);
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
    int targetNumber = 1;
    while(nh.ok()){
        ros::spinOnce();
        left_img = leftImageSubClass.getImage();
        if(!left_img.empty() ){
          // if SaliencyMap is used to gaze on the target
          if(motorState == true && approche == 1 && targetList.size() == 0 && machineStateStatus == machineStateStatusList[0]){
            targetList =  computeMultipleTargets(left_img);
            motorState = false;
            targetNumber = 1;
            machineStateStatus = machineStateStatusList[1];
          }else if (targetList.size() > 0 && motorState==true && machineStateStatus == machineStateStatusList[0] || temp.empty()){
            cout<<"   Gaze on target " << targetNumber<< " ======>"<<endl;
            temp = targetList.back();
            targetList.pop_back();
            motorState = false;
            machineStateStatus = machineStateStatusList[1];
          }
          // mouse click gaze controller
          if(updateTracker && approche == 2){
            Rect2d bbox = returnRectanguleTemplate(windowsCenter, windowSize, left_img.size());
            // cout<<"bbox: "<< bbox.size() << endl;
            temp = left_img(bbox);
            updateTracker = false;
          }

          if (!temp.empty() && machineStateStatus == machineStateStatusList[1]){
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
            if (motorState){
              cout <<"   Gaze Completed " << targetNumber<< " ======> Verge On Target Now"<<endl;
              // Verge on target
              machineStateStatus = machineStateStatusList[2];
            }
          }else{
            imshow(windowsNameString, left_img);
          } // End of gaze controller if statment

          // Start vergence MotorController
          if (machineStateStatus == machineStateStatusList[2]){
            // cout <<"   Verge On Target " << targetNumber<< " ======>"<<endl;
            // Check the statue of the vergence
            if(vergenceStatus == true){
              machineStateStatus = machineStateStatusList[3];
              cout <<"   Verge On Completed " << targetNumber << " ======>"<<endl;
              vergenceStatus = false;
            }

          }
          // Start Compute the 3D pose
          if (machineStateStatus == machineStateStatusList[3]){
            cout <<"   Compute 3D pose " << targetNumber<< " ======>"<<endl;
            // Compute 3D pose
            cout <<"   Target Position = " << targetPose.x << "," << targetPose.y << "," << targetPose.z <<endl;
            machineStateStatus = machineStateStatusList[4];
          }

          // Start Compute PCL
          if (machineStateStatus == machineStateStatusList[4]){
            cout <<"   Compute pointCloud " << targetNumber<< " ======>"<<endl;
            // Compute pointCloud
            machineStateStatus = machineStateStatusList[5];
          }

          // Publish Data
          if (machineStateStatus == machineStateStatusList[5]){
            cout <<"   Publish Data and update main map " << targetNumber<< " ======>"<<endl;
            // PublishData
            machineStateStatus = machineStateStatusList[0];
          }

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
