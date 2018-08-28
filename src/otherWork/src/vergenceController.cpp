#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sstream>
#include <math.h>       /* exp */

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"



using namespace std;
using namespace cv;


// Local headers
#include "/home/abdulla/dev/activeStereoVisionPlatform/src/platform_vision/src/include/motorController.h"
#include "/home/abdulla/dev/activeStereoVisionPlatform/src/platform_vision/src/include/helpFunctions.h"
#include "VergenceControl.h"

using namespace vergencecontrol;
bool STOP = false;
bool LOOP = false;
cv::Point2d C;


Size imageSize = Size(2048 , 1080);//Size(4096,2160);


#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// global virable
Mat left_img, right_img;

int horozintalGain = 200;
int old_horozintalGain = 0;
int main(int argc,char** argv)
{

  string ini_filename     = "/home/abdulla/dev/workshop/vergence_controller/data/Gt43B0.0208f0.063ph7.ini";
  string weights_filename = "/home/abdulla/dev/workshop/vergence_controller/data/vergence-weights.bin";
  int SX,SY;
  MatSize sz = right_img.size;
  SY = sz[0]; SX = sz[1];

  VergenceControl population(SX, SY, ini_filename, weights_filename, 3);

  float GAIN[2] = {(float)horozintalGain,-1};
  population.setVergenceGAIN(GAIN);

  /////////////////////////////////////////////////////////////////////////////////

  string SlaveWindowName = "Slave camera";
  cv::namedWindow(SlaveWindowName, cv::WINDOW_NORMAL);
  cv::resizeWindow(SlaveWindowName, 900, 600);
  cv::moveWindow(SlaveWindowName, 1000,30);

  int windowSize = 250;
  int algorithm = 1;
  if (argc > 1){
    istringstream ss(argv[1]);
    if (!(ss >> windowSize)){
      std::cout << "No argument \n";
    }else{
      std::cout << "windowSize " << windowSize << '\n';
    }

    istringstream saa(argv[2]);
    if (!(saa >> algorithm)){
      std::cout << "No argument \n";
    }else{
      std::cout << "algorithm " << algorithm << '\n';
    }
}else{
  std::cout << "No Argument "<< '\n';
}

  //////////////////////////////////////
  ros::init(argc,argv,"FeatureTrackingCuda");
  ros::NodeHandle nh;

  //define the subscriber and publisher
  // ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
  // ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
  GetImageClass rightImageSubClass(nh, "right");
  GetImageClass leftImageSubClass(nh, "left");
  cv::Rect windowSizeRectangule = returnRectanguleSizeOfCenterImage(imageSize,windowSize);
  MotorController motorController(nh, "right");
  motorController.moveToZero();
  ros::Rate r(10); // 10 hz
  cv::createTrackbar("horozintalGain", SlaveWindowName,  &horozintalGain, 1000);
  while(nh.ok()){
    ros::spinOnce();
    char ikey;
    ikey = cv::waitKey(1);
    right_img = rightImageSubClass.getImage();
    left_img = leftImageSubClass.getImage();

    if(horozintalGain != old_horozintalGain){
      float GAIN[2] = {(float)horozintalGain,-1};
      population.setVergenceGAIN(GAIN);
      old_horozintalGain =  horozintalGain;
    }


    if(!left_img.empty() && !right_img.empty()){
      // Step 1 convert the image to gray if they are in colour
      if(left_img.channels() > 1){
          cv::cvtColor(left_img,left_img,cv::COLOR_BGR2GRAY);
          resize(left_img,left_img, Size(640,420));
      }

      if(right_img.channels() > 1){
          cv::cvtColor(right_img,right_img,cv::COLOR_BGR2GRAY);
          resize(right_img,right_img, Size(640,420));
      }

      // Step 2: load image to the library and compute the vergence angle
      population.loadImg(left_img, 'L');
      population.loadImg(right_img, 'R');

      population.computeVergenceControl();
      cv::Point2f difference;
      difference.x = population.getVergenceH()*100;
      difference.y = population.getVergenceH();
      // Step 3: move the motors
      cout <<" H: " << difference.x <<"   V: " << difference.y <<endl;
      // population.printVergence();
      motorController.moveMotorWithoutExponanetial(difference.x); // 0 to move the pan motor
      motorController.moveTiltMotor(difference.y); // 1 to move the tilt motor

      cv::imshow(SlaveWindowName, right_img);
      char ikey = cv::waitKey(1);
      if (ikey == 'q'){
        break;
      }
    }
    r.sleep();
  }
  std::cout<< "Return To Zero Position !!" << std::endl;
  motorController.moveToZero();
  return 1;
}
