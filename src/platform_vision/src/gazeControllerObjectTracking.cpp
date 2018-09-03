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

#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;
#include "include/motorController.h"
#include "include/helpFunctions.h"
#include "include/FastMatchTemplate.h"

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()
Size imageSize = Size(2048 , 1080);//Size(4096,2160);
Point2f windowsCenter(imageSize.width/2, imageSize.height/2);
bool updateTracker = false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        windowsCenter.x = x;
        windowsCenter.y = y;
        updateTracker = true;
    }

}

// global virable
Mat left_img, right_img;

int main(int argc, char **argv)
{

  
  ros::init(argc,argv,"GazeController");
  string windowsNameString = "MasterCamera";
  ros::NodeHandle nh;
  int windowSize = 250;
  //define the subscriber and publisher
  GetImageClass rightImageSubClass(nh, "right");
  GetImageClass leftImageSubClass(nh, "left");
  cv::Rect windowSizeRectangule = returnRectanguleSizeOfCenterImage(imageSize,windowSize);
  //Define the publisher
  MotorController motorController(nh, "left");
  motorController.moveToZero();
  Mat temp;
  int rectSize = 200; // this int describe the size of the templet multiply by 2
  Mat result;

  // List of tracker types in OpenCV 3.4.1
  string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
  // vector <string> trackerTypes(types, std::end(types));

  // Create a tracker
  string trackerType = trackerTypes[2];

  Ptr<Tracker> tracker;

  #if (CV_MINOR_VERSION < 3)
  {
      tracker = Tracker::create(trackerType);
  }
  #else
  {
      if (trackerType == "BOOSTING")
          tracker = TrackerBoosting::create();
      if (trackerType == "MIL")
          tracker = TrackerMIL::create();
      if (trackerType == "KCF")
          tracker = TrackerKCF::create();
      if (trackerType == "TLD")
          tracker = TrackerTLD::create();
      if (trackerType == "MEDIANFLOW")
          tracker = TrackerMedianFlow::create();
      if (trackerType == "GOTURN")
          tracker = TrackerGOTURN::create();
//        if (trackerType == "MOSSE")
//            tracker = TrackerMOSSE::create();
//        if (trackerType == "CSRT")
//            tracker = TrackerCSRT::create();
  }
  #endif
  // Read first frame
  Mat frame;

  // Define initial bounding box
  int x1 = windowsCenter.x - windowSize/2;
  int x2 = windowsCenter.x + windowSize/2;
  int y1 = windowsCenter.y - windowSize/2;
  int y2 = windowsCenter.y + windowSize/2;
  Rect2d bbox(x1, y1, x2, y2);
  rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
  imshow(windowsNameString, frame);
  tracker->init(frame, bbox);
  //set the callback function for any mouse event
  setMouseCallback(windowsNameString, CallBackFunc, NULL);
  cv::Point2f difference;
  while(nh.ok()){
      ros::spinOnce();
      char ikey;
      ikey = cv::waitKey(1);
      right_img = rightImageSubClass.getImage();
      left_img = leftImageSubClass.getImage();
      if(!left_img.empty() && !right_img.empty()){

        if(updateTracker){
          x1 = windowsCenter.x - windowSize/2;
          x2 = windowsCenter.x + windowSize/2;
          y1 = windowsCenter.y - windowSize/2;
          y2 = windowsCenter.y + windowSize/2;
          Rect2d bbox(x1, y1, x2, y2);
          rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
          imshow(windowsNameString, frame);
          tracker->init(frame, bbox);
        }
        // if the template is empty
        if(bbox.height > 0){ // if the template is empty
          // Start timer
          double timer = (double)getTickCount();
          // Update the tracking result
          bool ok = tracker->update(frame, bbox);
          // Calculate Frames per second (FPS)
          float fps = getTickFrequency() / ((double)getTickCount() - timer);
          if (ok)
          {
              // Tracking success : Draw the tracked object
              rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
              difference = (bbox.br() + bbox.tl())*0.5;
              circle(frame, difference, 3, Scalar(0,0,255));
          }
          else
          {
              // Tracking failure detected.
              putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
          }
          // Display tracker type on frame
          putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
          // Display FPS on frame
          putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
        }
        // Display frame.

        bool panState = motorController.movePanMotor(difference.x); // 0 to move the pan motor
        bool tiltState = motorController.moveTiltMotor(difference.y); // 1 to move the tilt motor
        motorController.checkVergeCorrectely(panState,tiltState);

        imshow(windowsNameString, frame);

        // Exit if ESC pressed.
        int k = waitKey(1);
        if(k == 27)
        {
            break;
        }
      }// End of if statment
    }//End of while loop
}
