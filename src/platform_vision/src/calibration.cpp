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
#include <stdlib.h>
#include <std_msgs/String.h>

using namespace cv;
using namespace std;

// calibration repeated time
int experiment_no = 30;
// this virable will have the position of the motors
int run_array [30][2] = {{0,0},
             {-2,0},
             {-4,0},
             {-6,0},
			 {-8,0},
			 {2,0},
			 {4,0},
			 {6,0},
			 {8,0},
			 {0,2},
			 {0,4},
			 {0,6},
			 {0,8},
			 {0,-2},
			 {0,-4},
			 {0,-6},
			 {0,-8},
			 {2,2},
			 {4,4},
			 {6,6},
			 {8,8},
			 {-2,-2},
			 {-4,-4},
			 {-6,-6},
			 {-2,2},
			 {-4,4},
			 {2,-2},
			 {4,-4},
			 {6,-6},
			 {8,-8}};

//---------------------------------------------------------//
// global virable
Mat left_img, right_img;
Size sizee = Size(1280,720);
float left_pan = 9999,right_pan = 9999;

class calibrationClass{
public:
    ~calibrationClass();
    calibrationClass(Size size);
    // Define the virables use in calibration process
    void calibrateThecamera();
    void SaveTheResult(string file_name, float baseline, float right_angle, float left_angle);
    void FindCornersAndStore(Mat frame, Mat frame2, int *ProcessComplet, bool *cornerFound);
    void clearData();
private:
// Define the input virable from the user
int numBoard ;
int numCornHer;
int numCornVer;
double  err1, err2, rms;
int numSquare;		// From the input data
Size boardSize;		// From the input data
float HerDist, VerDist;		//To store the dimensions of the checker board

// Create a storage for point from the images for calibration
vector<vector<Point3f> > objectPoints;			// This is the phiscal point on the board
vector<vector<Point2f> > imagePoint;				// This to store the point in the image plane location of corner
vector<vector<Point3f> > objectPoints2;			// This is the phiscal point on the board
vector<vector<Point2f> > imagePoint2;			// This to store the point in the image plane location of corner
vector<Point3f> obj;

int sucsses;	// How many sucess we have in finding the corner

Mat grayImg;		// to store the gray image for process
Mat grayImg2;		// to store the gray image for process

Mat frame;
Mat frame2;
Size FrameSize;
// These storage for the intrinsic and distortion coffeicent
Mat Rvect;
Mat Tvect;
Mat Evect;
Mat Fvect, Pvect1, Pvect2, Qvect;
//For camera 2
Mat interinsic2; //Mat(3, 3, CV_64FC1);
Mat distCoeffs2;
Mat Rvect2;
Mat Tvect2;
// For camera 1
Mat interinsic1; //Mat(3, 3, CV_64FC1);
Mat distCoeffs1;
Mat Rvect1;
Mat Tvect1;
Mat ErrorMatrix;
Mat angle_value;
};


// the blow functions are the function use to get the values
void left_img_callback(const sensor_msgs::ImageConstPtr& imgl_msg){

    // create storage for the comming image in cv format
    cv_bridge::CvImagePtr cv_img_msg;

    //copy the image and save it in opencv formate
    try{
    cv_img_msg = cv_bridge::toCvCopy(imgl_msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
	ROS_ERROR("cv_bridge expection: %s", e.what());
	return;
    }

    left_img = cv_img_msg->image;
}


void right_img_callback(const sensor_msgs::ImageConstPtr& imgr_msg){

    // create storage for the comming image in cv format
    cv_bridge::CvImagePtr cv_img_msg;

    //copy the image and save it in opencv formate
    try{
    cv_img_msg = cv_bridge::toCvCopy(imgr_msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
	ROS_ERROR("cv_bridge expection: %s", e.what());
	return;
    }

    right_img = cv_img_msg->image;
}

void left_pan_callback(const std_msgs::Float64& val){

  left_pan = val.data;


}

void right_pan_callback(const std_msgs::Float64& val){

  right_pan = val.data;
}
float baseline_value;
void baseline_callback(const std_msgs::Float64& val){

  baseline_value = val.data;
}

string baxter_confirm ;
void baxter_confirm_callback(const std_msgs::String& st){
    baxter_confirm = st.data;

}

//int main(int argc, char **argv){

//float error = 0.15;

//  ros::init(argc,argv,"Calibration");
//  ros::NodeHandle nh;

//  //define the subscriber and publisher
//  ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
//  ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
//  // define the angle subscriper
//  ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",10,right_pan_callback);
//  ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",10,left_pan_callback);
//  ros::Subscriber baseline_sub = nh.subscribe("/baseline/position",10,baseline_callback);
//  //define the publishers
//  ros::Publisher left_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/left/pan/move", 50);
//  ros::Publisher right_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/right/pan/move",50);

//  // This topics to communicate with baxter
//  ros::Subscriber baxter_confirm_sub = nh.subscribe("send_UDP",200,baxter_confirm_callback);
//  ros::Publisher baxter_pub = nh.advertise<std_msgs::String>("/receive_UDP",100);

//  //Define virables to store the value of joints
//  geometry_msgs::Vector3 lcam_pan, rcam_pan;
//  rcam_pan.y = 0;
//  rcam_pan.z = 0;
//  lcam_pan.y = 0;
//  lcam_pan.z = 0;

//  while(nh.ok()){
//      ros::spinOnce();
//      char ikey;
//      ikey = cv::waitKey('q');

//      for(int j = 20; j < experiment_no;j++)
//	{

//	  stringstream dir;
//	  dir <<  "mkdir /home/abdulla/dev/workshop/calibration_data/" << setfill('0') << setw(3) << j+1 <<"_run/";
//	  string file_dir = dir.str();
//	  dir.str("");
//	  system(file_dir.c_str());
//	  cout<<"File was created..."<<endl;

//      // this for loop is the outer where it move the rig after completing the calibration
//	  for(int i = 0; i < 30; i++)
//	    {
//	      //for each calibration setup open the specific drectory of the calibration
//	      stringstream ssl;
//          ssl<<"/home/abdulla/dev/workshop/calibration_data/" << setfill('0') << setw(3) << j+1 <<"_run/"<<setfill('0') << setw(3) << i+1 <<".xml";
//	      string Filename = ssl.str();
//	      ssl.str("");

//          calibrationClass cal(sizee);
//	      //--- move the rig to the new position---//
//	      //Wait to the platform to get to the new angle
//           /* while(true){
//                ros::spinOnce();
//                float left_angle = run_array[i][0];
//                float right_angle = run_array[i][1];

//                lcam_pan.x = left_angle;
//                rcam_pan.x = right_angle;

//                // publish data
//                right_cam_pan_pub.publish(rcam_pan);
//                left_cam_pan_pub.publish(lcam_pan);

//                ros::Duration(0.1).sleep(); // sleep for 0.8 a second
//                // cout << "Right Difference: " << abs(right_pan - right_angle) <<endl;
//                //cout << "left Difference: " << abs(left_pan - left_angle) <<endl;
//                if(abs(left_pan - left_angle) <= error){
//                    if(abs(right_pan - right_angle) <= error){
//                    cout<< "Both camera set to the new position !!!"<<endl;
//                    break;
//                    }
//                }else{
//                   //cout<<left_angle<<"___"<<right_angle <<endl;
//                }

//            }
//            */
//	      //This loop to calibrate the camera
//          int completloop = 0;  // to exit the loop when the calibration complite
//          while(true)
//                    {

//                      //cout<<"Start calibration "<< i <<endl;


//                      if(!left_img.empty() && !right_img.empty()){	  // wait until get images
//                         //resize the images to sut with the calibration data
//                          //Mat frame, frame2;
//                          //resize(left_img,frame,sizee);
//                          //resize(right_img,frame2,sizee);
//                          // step  - find the corners and store them
//                          bool cornerFound = false;
//                          cout<<left_img.size()<<endl;
//                          cal.FindCornersAndStore(left_img,right_img,&completloop,&cornerFound);
//                          // check if the loop is completed
//                          if(completloop == 1){
//                             break;
//                            }
//                          //if cornerFound return true send command to baxter to move
//                         /* if(cornerFound == true){
//                              std_msgs::String baxter_msg;
//                              baxter_msg.data="OK";
//                              baxter_pub.publish(baxter_msg);
//                              //wait until baxter send back messeges
//                             /* while(1){
//                                  if(baxter_confirm== "OK"){
//                                      baxter_confirm == "NULL";
//                                      sleep(6);
//                                      break;
//                                  }
//                                  ros::spinOnce();
//                              }
//                          }*/
//                          cornerFound=false;
//                          // check to exit the programs
//                          char iKey = waitKey('q');
//                          if(iKey == 'q'){
//                             return 1;
//                          }
//                      }// end of the if statment of the images
//                        ros::spinOnce();
//                    }// end of the while loop
//	      // calibrate the system and save the parameters
//          cal.calibrateThecamera();
//	      //save data
//          cal.SaveTheResult(Filename, baseline_value,left_pan,right_pan);
//          // desroy the calss
//          //cal.~calibrationClass();
//	    }//end of run calibration

//	}//end of repeat experiment
//    }
//}


int main(int argc, char **argv){

float error = 0.15;

  ros::init(argc,argv,"Calibration");
  ros::NodeHandle nh;

  //define the subscriber and publisher
  ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
  ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
  // define the angle subscriper
  ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",10,right_pan_callback);
  ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",10,left_pan_callback);
  ros::Subscriber baseline_sub = nh.subscribe("/baseline/position",10,baseline_callback);
  //define the publishers
  ros::Publisher left_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/left/pan/move", 50);
  ros::Publisher right_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/right/pan/move",50);

  // This topics to communicate with baxter
  ros::Subscriber baxter_confirm_sub = nh.subscribe("send_UDP",200,baxter_confirm_callback);
  ros::Publisher baxter_pub = nh.advertise<std_msgs::String>("/receive_UDP",100);

  //Define virables to store the value of joints
  geometry_msgs::Vector3 lcam_pan, rcam_pan;
  rcam_pan.y = 0;
  rcam_pan.z = 0;
  lcam_pan.y = 0;
  lcam_pan.z = 0;

  while(nh.ok()){
      ros::spinOnce();
      char ikey;
      ikey = cv::waitKey('q');

      for(int j = 20; j < experiment_no;j++)
    {

      stringstream dir;
      dir <<  "mkdir /home/abdulla/dev/workshop/calibration_data/" << setfill('0') << setw(3) << j+1 <<"_run/";
      string file_dir = dir.str();
      dir.str("");
      system(file_dir.c_str());
      cout<<"File was created..."<<endl;

      // this for loop is the outer where it move the rig after completing the calibration
      for(int i = 0; i < 30; i++)
        {
          //for each calibration setup open the specific drectory of the calibration
          stringstream ssl;
          ssl<<"/home/abdulla/dev/workshop/calibration_data/" << setfill('0') << setw(3) << j+1 <<"_run/"<<setfill('0') << setw(3) << i+1 <<".xml";
          string Filename = ssl.str();
          ssl.str("");

          calibrationClass cal(sizee);
          //This loop to calibrate the camera
          int completloop = 0;  // to exit the loop when the calibration complite
          while(true){

              if(!left_img.empty() && !right_img.empty()){	  // wait until get images
                  bool cornerFound = false;
                  //cout<<left_img.size()<<endl;
                  cal.FindCornersAndStore(left_img,right_img,&completloop,&cornerFound);
                  // check if the loop is completed
                  if(completloop == 1){
                     break;
                    }

                  cornerFound=false;
                  // check to exit the programs
                  char iKey = waitKey('q');
                  if(iKey == 'q'){
                     return 1;
                  }
              }// end of the if statment of the images
                ros::spinOnce();
            }// end of the while loop
        // calibrate the system and save the parameters
        cal.calibrateThecamera();
        //save data
        cal.SaveTheResult(Filename, baseline_value,left_pan,right_pan);
        // desroy the calss
        cal.~calibrationClass();
    }//end of run calibration

    }//end of repeat experiment
    }
}

void calibrationClass::calibrateThecamera(){
    cout << "Start calibration...." << endl;
    cout<<"Frame size: "<<FrameSize<<endl;
   // calibrate camera 1
     err1 = calibrateCamera(objectPoints,
			imagePoint,
			FrameSize,
			interinsic1,
			distCoeffs1,
			Rvect1,
			Tvect1);

    // calibrate camera 2
      err2 = calibrateCamera(objectPoints,
			 imagePoint2,
			 FrameSize,
			 interinsic2,
			 distCoeffs2,
			 Rvect2,
			 Tvect2);


    //Calibrate stereo camera
     rms = stereoCalibrate(objectPoints,
	imagePoint, imagePoint2,
	interinsic1, distCoeffs1,
	interinsic2, distCoeffs2,
	FrameSize,
	Rvect,
	Tvect,
	Qvect,
	Fvect,
	CALIB_FIX_INTRINSIC,TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6));


    ErrorMatrix = (Mat_<double>(3,1) <<
		       err1,    err2,      rms);

    cout << "Calibration was done successfully..\n" << endl;

    /*cout << "interinsic1" << interinsic1 << endl;
    cout << "\ninterinsic2" << interinsic2 << endl;
    cout << "\ndistCoeffs1" << distCoeffs1 << endl;
    cout << "\ndistCoeffs2" << distCoeffs2 << endl;
    cout << "\nR" << Rvect << endl;
    cout << "\nT" << Tvect << endl;
    cout << "\nE" << Evect << endl;
    cout << "\nF" << Fvect << endl;*/
    cout << "\nError"<< ErrorMatrix << endl;
}

void calibrationClass::SaveTheResult(string file_name,float baseline, float right_angle, float left_angle){

    angle_value = (Mat_<float>(3,1) <<
                     right_angle,    left_angle,      baseline);

    cout<<"The file name will save as : "<<file_name<<endl;
    FileStorage fs(file_name, FileStorage::WRITE);
    fs << "interinsic1" << interinsic1;
    fs << "interinsic2" << interinsic2;
    fs << "distCoeffs1" << distCoeffs1;
    fs << "distCoeffs2" << distCoeffs2;
    fs << "R" << Rvect;
    fs << "T" << Tvect;
    fs << "E" << Evect;
    fs << "F" << Fvect;
    fs << "Error"<< ErrorMatrix;
    fs << "actual_angle"<<angle_value;

    cout << "\nCalibration parameters saved..." << endl;
    fs.release();
}

// This function use to find the corners
void calibrationClass::FindCornersAndStore(Mat frame, Mat frame2, int *ProcessComplet, bool *cornerFound){

    FrameSize = frame.size();
    // This function to look for corner
    bool BothGood = false;
    // This storage for finding corner
    vector<Point2f> corners;						// to hold the corner position
    vector<Point2f> corners2;						// to hold the corner position
    //Convert Frame to gray
    cvtColor(frame, grayImg, CV_RGB2GRAY);
    cvtColor(frame2, grayImg2, CV_RGB2GRAY);
    //findChessboardCorners(grayImg,patternSize,corners,cornerCount,CV_CALIB_CB_FAST_CHECK);
    bool patternWasFound = findChessboardCorners(grayImg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
    bool patternWasFound2 = findChessboardCorners(grayImg2, boardSize, corners2, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
    if (patternWasFound && patternWasFound2){
	cornerSubPix(grayImg, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
	cornerSubPix(grayImg2, corners2, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
	BothGood = true;	//if both frame find corner make it true
    }
    else{
	BothGood = false;	//if not make it false
    }
    //if key press display images
    char ikey = waitKey('q');
    int  windows = 0;

    namedWindow("windows press");

//    if(ikey =='v'){
//        windows = 1;
//    }
//    if(ikey == 'b'){
//        windows = 0;
//        destroyAllWindows();
//    }
//    if(windows == 1){
    drawChessboardCorners(frame, boardSize, corners, patternWasFound);
    drawChessboardCorners(frame2, boardSize, corners2, patternWasFound2);
    resize(frame,frame,Size(600,400));
    resize(frame2,frame2,Size(600,400));

    Mat combine_img;
    hconcat(frame,frame2,combine_img);

    namedWindow("calibration",CV_WINDOW_FREERATIO);
    resizeWindow("calibration",1200,300);
    imshow("calibration", combine_img);
    //if q press exit the system
//    }else{

//    }

    if ( BothGood == true){	// if the corner found and the user press s it will store the corner data in the imagePoint

    imagePoint.push_back(corners);			// Store corners values in the image point for calibration
    objectPoints.push_back(obj);			// Stroe the phisical position in the object point for calibration

    imagePoint2.push_back(corners2);			// Store corners values in the image point for calibration
    objectPoints2.push_back(obj);			// Stroe the phisical position in the object point for calibration
	cout << "Corners Stored suceessfully.." << sucsses + 1 << endl;
    sucsses++;
    if (sucsses >= numBoard){
	    cout << "ok" << endl;
        *ProcessComplet = 1;
	}
    //Send To Baxter to move to new position and wait the confirmation from Baxter
    *cornerFound = true;
    }	//End if statment

}

void calibrationClass::clearData(){
  // Clear the storage to store new data
  Rvect.release();
  Tvect.release();
  Evect.release();
  Fvect.release();
  Pvect1.release();
  Pvect2.release();
  Qvect.release();
  //For camera 2
  interinsic2.release(); //Mat(3, 3, CV_64FC1);
  distCoeffs2.release();
  Rvect2.release();
  Tvect2.release();
  // For camera 1
  interinsic1.release(); //Mat(3, 3, CV_64FC1);
  distCoeffs1.release();
  Rvect1.release();
  Tvect1.release();
  ErrorMatrix.release();
}

calibrationClass::calibrationClass(Size size){
   // FrameSize = size;

    // Define the input virable from the user
     numBoard = 12;
     numCornHer = 9;//8;
     numCornVer = 6;

     numSquare = numCornHer * numCornVer;		// From the input data
     boardSize = Size(numCornHer,numCornVer);		// From the input data
     HerDist =  24;//34.5;
     VerDist = HerDist;		//To store the dimensions of the checker board
     sucsses = 0;

     // This storage for the actual coordinate refrence we can use the dimension
     // as (0,0,0),(0,1,0),(0,2,0),.....(1,4,0),....
     for (int j = 0; j < numSquare; j++){
         obj.push_back(Point3f((j / numCornHer) * HerDist, (j%numCornHer) * VerDist, 0.0f));		// we use a unit because we are ni=ot the concern to dimension
     }
}

calibrationClass::~calibrationClass(){
    //clearData();
}
