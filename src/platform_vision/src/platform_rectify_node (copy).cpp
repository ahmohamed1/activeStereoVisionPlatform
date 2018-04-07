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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>



using namespace cv;
using namespace std;
bool debug = false;
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

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

// void image_sychronizer_callback(const sensor_msgs::ImageConstPtr left_img_msg, const sensor_msgs::ImageConstPtr right_img_msg){
//
//   // create storage for the comming image in cv format
//   cv_bridge::CvImagePtr cv_img_msg_right;
//
//   //copy the image and save it in opencv formate
//   try{
//   cv_img_msg_right = cv_bridge::toCvCopy(right_img_msg,sensor_msgs::image_encodings::BGR8);
//   }
//   catch(cv_bridge::Exception e)
//   {
//       ROS_ERROR("cv_bridge expection: %s", e.what());
//       return;
//   }
//
//   right_img = cv_img_msg_right->image;
//
//   // create storage for the comming image in cv format
//   cv_bridge::CvImagePtr cv_img_msg_left;
//   //copy the image and save it in opencv formate
//   try{
//   cv_img_msg_left = cv_bridge::toCvCopy(left_img_msg,sensor_msgs::image_encodings::BGR8);
//   }
//   catch(cv_bridge::Exception e)
//   {
//       ROS_ERROR("cv_bridge expection: %s", e.what());
//       return;
//   }
//
//   left_img = cv_img_msg_left->image;
//
// }

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

int main(int argc,char** argv)
{

  //Load the calibraton date from the raw dat
  string filename = "/home/abdulla/dev/Active-stereo-Vision-Platform/calibration_data/001_6sep.xml";
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

  if(debug){
    // print them out
    cout << "interinsic1" << A1 << endl;
    cout << "\ninterinsic2" << A2 << endl;
    cout << "\ndistCoeffs1" << D1 << endl;
    cout << "\ndistCoeffs2" << D2 << endl;
    cout << "\nR" << R << endl;
    cout << "\nT" << T << endl;
    cout << "\nE" << E << endl;
    cout << "\nF" << F << endl;
  }
  //////////////////////////////////////
    ros::init(argc,argv,"RectifyImages");
    ROS_INFO("START PUBLISH RECTIFIED IMAGES");
    ros::NodeHandle nh;
    //n is in the node's namespace
    string image_type = "rgb8";
    ros::NodeHandle n("~");
    n.getParam("image_type", image_type);
    ROS_INFO("image type was %d", image_type);

    //define the subscriber and publisher
    ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
    ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
    // message_filters::Subscriber<sensor_msgs::Image> left_img_sub(nh, "/stereo/left/image_raw",1);
    // message_filters::Subscriber<sensor_msgs::Image> right_img_sub(nh, "/stereo/right/image_raw",1);
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_img_sub, right_img_sub, 10);
    // ros::Publisher rectify_right_image_pub = nh.advertise<sensor_msgs::ImageConstPtr>("stereo/right/rect_image");
    // ros::Publisher rectify_left_image_pub = nh.advertise<sensor_msgs::ImageConstPtr>("stereo/left/rect_image");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher rectify_right_image_pub = it.advertise("stereo/right/image_rect_color", 10);
    image_transport::Publisher rectify_left_image_pub = it.advertise("stereo/left/image_rect_color", 10);

    // define the angle subscriper
    ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",10,right_pan_callback);
    ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",10,left_pan_callback);
    ros::Subscriber baseline_sub = nh.subscribe("/baseline/position",10,baseline_callback);

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
        if(!left_img.empty() && !right_img.empty()){
            double matching_time;
            matching_time = (double)getTickCount();
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
              Mat canvas;
              double sf;
              int w, h;
              bool isVerticalStereo;
              if (debug){
                Size imageSize = sizee;
                // OpenCV can handle left-right
                // or up-down camera arrangements
                isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));


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


          }
          // remap the image and present them

            Mat undisFrame1,undisFrame2;
            remap(left_img, undisFrame1, mapx1, mapy1, INTER_LINEAR, BORDER_CONSTANT, Scalar());
            remap(right_img, undisFrame2, mapx2, mapy2, INTER_LINEAR, BORDER_CONSTANT, Scalar());
            Mat image [2] = {undisFrame1,undisFrame2};
            if (debug){
              RNG rng(0xFFFFF);
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
              //
              namedWindow("rectified",WINDOW_AUTOSIZE);
              imshow("rectified", canvas);
              cv::cvtColor(undisFrame1,undisFrame1,CV_BGR2GRAY);
              cv::cvtColor(undisFrame2,undisFrame2,CV_BGR2GRAY);
              undisFrame1 = undisFrame1(validRoi[0]);
              undisFrame2 = undisFrame2(validRoi[1]);

          }
            // Publish the images
            sensor_msgs::ImagePtr left_img_msg = cv_bridge::CvImage(std_msgs::Header(), image_type, undisFrame1).toImageMsg();
            sensor_msgs::ImagePtr right_img_msg = cv_bridge::CvImage(std_msgs::Header(), image_type, undisFrame2).toImageMsg();
            rectify_right_image_pub.publish(right_img_msg);
            rectify_left_image_pub.publish(left_img_msg);
    }
  }// End of while loop

return 1;

}
