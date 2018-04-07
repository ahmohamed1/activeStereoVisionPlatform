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

        // calibration repeated time
        int repeat_data_collection = 20;
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

// global virable
Mat left_img, right_img;
Size sizee = Size(2048 , 1080);
//Size sizee = Size(1200,900);
float left_pan = 0,right_pan = 0;
float baseline_value = 0.0;
float baseline_value_old = 0.0;
// These virables are the value form the offline calibration
float angle_z = -0.425903209166667;
float angle_x = 0.537310137283333;
float old_angle_sum = 1.0;
// These storage for the intrinsic and distortion coffeicent
Mat R = Mat(3, 3, CV_64FC1);
Mat T;
Mat E;
Mat F, P1, P2,Q;
//For camera 2
Mat A2 = Mat(3, 3, CV_64FC1);
Mat D2;
Mat R2;
Mat T2;
// For camera 1
Mat A1 = Mat(3, 3, CV_64FC1);
Mat D1;
Mat R1;
Mat T1;

// the blow functions are the function use to get the values
void left_img_callback(const sensor_msgs::ImageConstPtr& img_msg){
    // create storage for the comming image in cv format
    cv_bridge::CvImagePtr cv_img_msg;
    //copy the image and save it in opencv formate
    try{
    cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR("cv_bridge expection: %s", e.what());
        return;
    }

    left_img = cv_img_msg->image;
}

void right_img_callback(const sensor_msgs::ImageConstPtr& img_msg){

    // create storage for the comming image in cv format
    cv_bridge::CvImagePtr cv_img_msg;

    //copy the image and save it in opencv formate
    try{
    cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR("cv_bridge expection: %s", e.what());
        return;
    }

    right_img = cv_img_msg->image;
}

float old_left_pan = 0,old_right_pan = 0;
void left_pan_callback(const std_msgs::Float64& val){
  left_pan = val.data;
}

void right_pan_callback(const std_msgs::Float64& val){

  right_pan = val.data;
}


void baseline_callback(const std_msgs::Float64& val){

  baseline_value = val.data / 1000;
}

// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
           1,       0,              0,
           0,       cos(theta[0]),   -sin(theta[0]),
           0,       sin(theta[0]),   cos(theta[0])
           );

    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
           cos(theta[1]),    0,      sin(theta[1]),
           0,                1,      0,
           -sin(theta[1]),   0,      cos(theta[1])
           );

    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
           cos(theta[2]),    -sin(theta[2]),      0,
           sin(theta[2]),    cos(theta[2]),       0,
           0,                0,                   1
        );


    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
    return R;
}

///////////////////////////////////////////
/// \brief calculate_projection_error
double calculate_projection_error(cv::Mat left, cv::Mat right, cv::Size boardSize, bool showimg){
    //double delta_time;
    //delta_time = (double)cv::getTickCount();

    // Initialize the points storage and find the corners
    vector<cv::Point2f> left_corners, right_corners;

    bool patternWasFound_left = findChessboardCorners(left, boardSize, left_corners );
    bool patternWasFound_right = findChessboardCorners(right, boardSize, right_corners );

    // Calculate the subpixel for more accurcy
    if (patternWasFound_left && patternWasFound_right){
        cv::cornerSubPix(left, left_corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
        cv::cornerSubPix(right, right_corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
    }
    if (showimg){
        drawChessboardCorners(left, boardSize, left_corners, patternWasFound_left);
        drawChessboardCorners(right, boardSize, right_corners, patternWasFound_right);
        cv::Mat combine_img;
        cv::hconcat(left,right,combine_img);
        cv::namedWindow("epipolar error",WINDOW_NORMAL);
        imshow("epipolar error", combine_img);
        resizeWindow("epipolar error", 600,300);
        cv::waitKey(5);
    }

    // Check if the corener found in both images
    if(patternWasFound_left && patternWasFound_right){
        // if in both image the corners found calculate  the projection error using SAM
        double total_error = 0;
        for (int i =0; i < (int)left_corners.size(); i++){
//            cout << "left: " << left_corners[i].y <<"\tTwo: "<< right_corners[i].y <<endl;
            double error= sqrt(pow((left_corners[i].y - right_corners[i].y),2));
            total_error = total_error + error;
        }
//        delta_time = ((double)cv::getTickCount() - delta_time)/cv::getTickFrequency();
//        cout<<"FPS: " << 1 / delta_time <<endl;
        return total_error/(boardSize.height * boardSize.width);
    }
    return 0;
}
ros::Publisher left_cam_pan_pub;
ros::Publisher right_cam_pan_pub;
// vector < vector< double > > epipolar_error_data_storage;
// Mat epipolar_error_data_storage = Mat_<double>(30,repeat_data_collection);
double epipolar_error_data_storage[30][30];
void move_camera_to_located_position(int x, ros::NodeHandle);

int main(int argc,char** argv)
{

  //Load the calibraton date from the raw dat
  string filename = "/home/abdulla/dev/Active-stereo-Vision-Platform/calibration_data/002_18sep.xml";
  FileStorage fr(filename, FileStorage::READ);
  fr["interinsic1"] >> A1;
  fr["interinsic2"] >> A2;
  fr["distCoeffs1"] >> D1;
  fr["distCoeffs2"] >> D2;
  fr["R"] >> R;
  fr["T"] >> T;
  fr["E"] >> E;
  fr["F"] >> F;
  fr.release();
  Mat MR,MQ,Qx,Qy,Qz;
  Vec3d angles = cv::RQDecomp3x3(R,MR,MQ,Qx,Qy,Qz);

  ros::init(argc,argv,"RectifyImages");
  ros::NodeHandle nh;

  //define the subscriber and publisher
  ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
  ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
  //define the publishers
  left_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/left/pan/move", 50);
  right_cam_pan_pub = nh.advertise<geometry_msgs::Vector3>("/right/pan/move",50);
  // define the angle subscriper
  ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",10,right_pan_callback);
  ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",10,left_pan_callback);
  ros::Subscriber baseline_sub = nh.subscribe("/baseline/position",10,baseline_callback);
  ros::Duration(2).sleep();

  float old_sum= 0;
    bool angle_chang = true;
    Mat mapx1, mapy1, mapx2, mapy2;
    int np_img =1;
    Rect validRoi[2];

  // start the while loop
  while(nh.ok()){

      ros::spinOnce();
      char ikey;
      ikey = cv::waitKey('q');
      for(int j = 0; j < repeat_data_collection;j++){
        for(int i = 0; i < 30; i++){
          move_camera_to_located_position(i, nh);
          ROS_INFO("RUN: %d  repeat: %d", i, j);

          double epi_error = 999;
          while(epi_error > 995 ){
            // cheack if the total difference bwtween the left angle and the right angle has change
            float angle_sum = (left_pan) - (right_pan);
            angle_sum = angle_sum;
            //cout<<"angle sum: " << angle_sum << endl;
            if(angle_sum != old_angle_sum || baseline_value_old != baseline_value){
                 // calculate the image angle and convert angle to radian
                 float img_angle = 0.964*angle_sum + 0.5786;
                 // Calculate the rotating matrix
                 Vec3f theta = angles;
                 //theta[0] = angle_x;
                // theta[2] = angle_z;
                 theta[1] = img_angle;
                 theta = theta* 0.0174533;

                 R = eulerAnglesToRotationMatrix(theta);
                 //update the baseline
                 T.at<float> (0,0) = baseline_value;

                // calculate the the new projection matrix using bouguet algorithm
                sizee = left_img.size();
                stereoRectify(A1, D1,
                        A2, D2,
                        sizee,
                        R, T,
                        R1, R2,
                        P1, P2,
                        Q,
                        CALIB_ZERO_DISPARITY, // to allowed zero disparity
                               1, sizee,
                              &validRoi[0], &validRoi[1]);


                // initialize undistortion
                // Define the map in x and y direction

                // initialize the undistortion rectify map for each camera
                initUndistortRectifyMap(A1, D1, R1,
                            P1, sizee, CV_32FC1, mapx1, mapy1);

                initUndistortRectifyMap(A2, D2, R2,
                            P2, sizee, CV_32FC1, mapx2, mapy2);

                //return angle change to false so to not do rectification process if the angle doen't change
                old_angle_sum = angle_sum;
                baseline_value_old = baseline_value;
              }
            Size imageSize = sizee;
            // OpenCV can handle left-right
            // or up-down camera arrangements
            bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
            Mat canvas;
            double sf;
            int w, h;
            if( !isVerticalStereo )
            {
                sf = 600./MAX(imageSize.width, imageSize.height);
                w = cvRound(imageSize.width*sf);
                h = cvRound(imageSize.height*sf);
                canvas.create(h, w*2, CV_8UC3);
            }
            else
            {
                sf = 300./MAX(imageSize.width, imageSize.height);
                w = cvRound(imageSize.width*sf);
                h = cvRound(imageSize.height*sf);
                canvas.create(h*2, w, CV_8UC3);
            }

            // remap the image and present them
            RNG rng(0xFFFFF);
            Mat undisFrame1,undisFrame2;
            remap(left_img, undisFrame1, mapx1, mapy1, INTER_LINEAR, BORDER_CONSTANT, Scalar());
            remap(right_img, undisFrame2, mapx2, mapy2, INTER_LINEAR, BORDER_CONSTANT, Scalar());
            Mat image [2] = {undisFrame1,undisFrame2};
            for( int k = 0; k < 2; k++ )// cv::cvtColor(undisFrame1,undisFrame1,CV_BGR2GRAY);
            // cv::cvtColor(undisFrame2,undisFrame2,CV_BGR2GRAY);
            {
                Mat cimg = image[k];
                Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
                resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);

                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 2, 8);
            }

            if( !isVerticalStereo )
                for( int j = 0; j < canvas.rows; j += 16 )
                    line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(rng(255),rng(255),rng(255)), 1, 8);
            else
                for(int j = 0; j < canvas.cols; j += 16 )
                    line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(rng(255),rng(255),rng(255)), 1, 8);

            namedWindow("rectified",WINDOW_AUTOSIZE);
            imshow("rectified", canvas);
            cv::Size boardSize = cv::Size(8,6);
            cv::cvtColor(undisFrame1,undisFrame1,CV_BGR2GRAY);
            cv::cvtColor(undisFrame2,undisFrame2,CV_BGR2GRAY);
            epi_error = calculate_projection_error(undisFrame1, undisFrame2,boardSize, true);
            ROS_INFO("Epipolo error: %f" ,epi_error );
            // epipolar_error_data_storage.at<double> (j,i) = epi_error;
            epipolar_error_data_storage[j][i] = epi_error;
        }
      }
    //ROS_INFO("Epipolare error storage size: %d ", epipolar_error_data_storage.size() );
  }

      // Save data
      string file_name = "/home/abdulla/dev/Active-stereo-Vision-Platform/calibration_data/009_base_450mm_data.txt";
      //FileStorage fs(file_name, FileStorage::WRITE);
      std::fstream fs(file_name, std::ios::out | std::ios::app);
      //fs << "epipolar_error_data_storage" << epipolar_error_data_storage;
      for (int j = 0; j < repeat_data_collection; ++j)
        {
        for (int i = 0; i < 30; ++i)
        {
            fs << epipolar_error_data_storage[j][i]<<",";
        }
        fs<<"\n";
    }
    fs.close();
      break;
  }// End of while loop

  return 1;

}


void move_camera_to_located_position(int x, ros::NodeHandle nh){
  //Define virables to store the value of joints
  geometry_msgs::Vector3 lcam_pan, rcam_pan;
  rcam_pan.y = 0;
  rcam_pan.z = 0;
  lcam_pan.y = 0;
  lcam_pan.z = 0;

  while(nh.ok()){
     ros::spinOnce();
     float left_angle = run_array[x][0];
      float right_angle = run_array[x][1];

      lcam_pan.x = left_angle;
      rcam_pan.x = right_angle;

      // publish data
      right_cam_pan_pub.publish(rcam_pan);
      left_cam_pan_pub.publish(lcam_pan);

      ros::Duration(0.2).sleep(); // sleep for 0.8 a second
      // cout << "Right Difference: " << abs(right_pan - right_angle) <<endl;
      //cout << "left Difference: " << abs(left_pan - left_angle) <<endl;
      if(abs(left_pan - left_angle) <= 0.15){
          if(abs(right_pan - right_angle) <= 0.15){
          // ROS_INFO("Both camera set to the new position !!!");
          //ros::Duration(5).sleep();
          break;
          }
      }else{
         //cout<<left_angle<<"___"<<right_angle <<endl;
      }
    }
}
