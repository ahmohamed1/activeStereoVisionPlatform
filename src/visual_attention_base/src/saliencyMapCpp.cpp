#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
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
#include "include/multipleMarkerPublisher.h"
////////////////////////////////////////////////////////

////Gloables Names
string windowsNameString = "MasterCamera";
string saliencymapName = "SaliencyMap";


// Define saliency map  and gaze controller
FastTemplateMatch gazeFastMatchTemplate;
IttiSaliencyMap saliencyMap(0);
FocalOfAttentionBasedWatershed FOA(false);

#include "include/SaliencyMapHelpFunctions.h"


std::vector<string> machineStateStatusList = {"ComputeSaliency", // 0
                                              "GazeOnTarget",    // 1
                                              "VergeOnTarget",   // 2
                                              "3DPose",          // 3
                                              "ComputePCL",      // 4
                                              "PublishData"};    // 5
string machineStateStatus = machineStateStatusList[0];

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




    std::vector<templateWithProbability> targetList;
    bool motorState = true;
    ros::init(argc,argv,"SaliencyMap");

    ros::NodeHandle nh;
    //define the subscriber and publisher
    GetImageClass leftImageSubClass(nh, "left");
    GetImageClass rightImageSubClass(nh, "right");
    ros::Subscriber vergeStatueSubscriber = nh.subscribe("/right/onTarget" ,1, &vergeStatue_callback);
    ros::Subscriber targetPoseSubscriber = nh.subscribe("/targetPose" ,1, &targetPose_callback);

    //Define the publisher
    MultipleMarkerPublisher multipleMarkerPublisher(nh);
    MotorController motorController(nh, "left", false);
    ros::Publisher templateSizePublisher = nh.advertise<geometry_msgs::Pose2D>("/templateSize", 1);
    motorController.moveToZero();
    Mat temp, right_img;
    cv::Rect windowSizeRectangule = returnRectanguleSizeOfCenterImage(imageSize,windowSize);



    namedWindow(windowsNameString, WINDOW_NORMAL);
    resizeWindow(windowsNameString,640,420);
    cv::moveWindow(windowsNameString, 1000,20);

    cv::namedWindow(saliencymapName, WINDOW_NORMAL);
    cv::resizeWindow(saliencymapName, 640,420);
    cv::moveWindow(saliencymapName, 1000,500);
    ros::Rate r(15); // 10 hz
    // start the while loop
    //set the callback function for any mouse event
    setMouseCallback(windowsNameString, CallBackFunc, NULL);
    int targetNumber = 1;
    std::vector<targetInformation> targetsListInformation;
    targetInformation targetdata;
    int imageSavedNumber = 1;
    cv::Mat saliencyMapImage;
    bool checkToSaveData = false;
    while(nh.ok()){
        ros::spinOnce();
        left_img = leftImageSubClass.getImage();
        right_img = rightImageSubClass.getImage();
        if(!left_img.empty() ){
          // if SaliencyMap is used to gaze on the target
          if(motorState == true && approche == 1 && targetList.size() == 0 && machineStateStatus == machineStateStatusList[0]){
            cout<<"//////////////////////////////////////////////////////"<<endl;
            // reset multipleMarkerPublisher to zero
            targetsListInformation.clear();
            tie(targetList, saliencyMapImage) =  computeMultipleTargets(left_img);
            motorState = false;
            targetNumber = 1;
            machineStateStatus = machineStateStatusList[1];
          }else if (targetList.size() > 0 && motorState==true && machineStateStatus == machineStateStatusList[0] || temp.empty()){
            cout<<"   Gaze on target ======>"<<endl;
            templateWithProbability _temp = targetList.back();
            targetList.pop_back();
            temp = _temp.templateImage;
            motorState = false;
            // Publish the template size
            geometry_msgs::Pose2D templateSize;
            templateSize.x = temp.cols;
            templateSize.y = temp.rows;
            templateSizePublisher.publish(templateSize);
            machineStateStatus = machineStateStatusList[1];
            // update targetdata with probability
            targetdata.probability2D = _temp.probability2D;
            targetdata.idea = targetNumber;
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
              cout <<"   Gaze Completed ======> Verge On Target Now"<<endl;
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
              cout <<"   Verge On Completed ======>"<<endl;
              vergenceStatus = false;
            }

          }
          // Start Compute the 3D pose
          if (machineStateStatus == machineStateStatusList[3]){
            cout <<"   Compute 3D pose ======>"<<endl;
            // Compute 3D pose
            cout <<"   Target Position = " << targetPose.x << "," << targetPose.y << "," << targetPose.z <<endl;
            // update the targetsListInformation
            targetdata.targetPose.x = targetPose.x;
            targetdata.targetPose.y = targetPose.y;
            targetdata.targetPose.z = targetPose.z;
            machineStateStatus = machineStateStatusList[4];
          }

          // Start Compute PCL
          if (machineStateStatus == machineStateStatusList[4]){
            cout <<"   Compute pointCloud ======>"<<endl;
            // Compute pointCloud
            targetdata.probability3D = 80;
            machineStateStatus = machineStateStatusList[5];
          }

          // Publish Data
          if (machineStateStatus == machineStateStatusList[5]){
            cout <<"   Publish Data and update main map ======>"<<endl;
            // PublishData
            targetsListInformation.push_back(targetdata);
            multipleMarkerPublisher.publishMarker(targetsListInformation);
            machineStateStatus = machineStateStatusList[0];
            checkToSaveData = true;
          }

      }// end of the if statment of the images
      char ikey;
      ikey = cv::waitKey(1);
      if(ikey == 'q'){
        motorController.moveToZero();
        break;
      }

      if(checkToSaveData){
        cout << "Press s to save the data!!!" << endl;
        ikey = cv::waitKey(0);
        if(ikey == 's'){
          saveData(left_img, right_img, temp, saliencyMapImage, imageSavedNumber);
          cout << "Data Saved Correctly" << endl;
          checkToSaveData = false;
          imageSavedNumber ++;
        }
      }

      r.sleep();
  }// End of while loop

motorController.moveToZero();
return 1;


}
