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
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <opencv2/core/cuda.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/cudafilters.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
using namespace cv;
using namespace cv::ximgproc;
using namespace std;

#include "include/helpFunctions.h"
#include "include/disparityClass.h"

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << " <option(s)> SOURCES"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-d,--destination DESTINATION\tSpecify the destination path"
              << std::endl;
}

/// This function to convert to cloud point
ros::Publisher pointCloud_pub;
// global virable
Mat left_img, right_img;
//Size sizee = Size(4096,2160);
Size sizee = Size(2048 , 1080);
//Size sizee = Size(1200,900);
float left_pan = 0,right_pan = 0;
float baseline_value = 0.0;
float baseline_value_old = 0.0;
// These virables are the value form the offline calibration
float angle_z = -0.425903209166667;
float angle_x = 0.537310137283333;
float old_angle_sum = 9999;
float old_left_angle = 9999;
float old_right_angle = 9999;
float error_margen_for_motor_controller = 0.4;

// the blow functions are the function use to get the values
void left_img_callback(const sensor_msgs::ImageConstPtr& img_msg){
    left_img =  convertROSMat2OpencvMat(img_msg);
}

void right_img_callback(const sensor_msgs::ImageConstPtr& img_msg){
    right_img =  convertROSMat2OpencvMat(img_msg);
}


float old_left_pan = 0,old_right_pan = 0;
void left_pan_callback(const std_msgs::Float64& val){
  if ((val.data > (old_left_angle + error_margen_for_motor_controller)) || (val.data < (old_left_angle + error_margen_for_motor_controller))){
    left_pan = val.data;
    old_left_angle = left_pan;
  }
}

void right_pan_callback(const std_msgs::Float64& val){

  if ((val.data > (old_right_angle + error_margen_for_motor_controller)) || (val.data < (old_right_angle + error_margen_for_motor_controller))){
    right_pan = val.data;
    old_right_angle = right_pan;
  }
}


void baseline_callback(const std_msgs::Float64& val){

  baseline_value = -val.data / 1000;
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

int Hmin = 0;
int Hmax = 91;
int Smin = 112;
int Smax = 255;
int Vmin = 14;
int Vmax = 160;
Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(13, 13));
Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(13, 13));


int main(int argc,char** argv)
{
  bool resize_image_for_disparity = true;
  // std::cout << argv[1] << std::endl;

  string calibration_file = "005_used_data";
  if(argc > 1 ){
    calibration_file = argv[1];
  }

  string my_bool = argv[2];
  if (my_bool == "true"){
    resize_image_for_disparity = false;
    std::cout << "Full Size Image Processing !!!" << std::endl;
  }else{
    std::cout << "Small Size Image Processing !!!" << std::endl;
  }
  //////////////////////////////////////
  ros::init(argc,argv,"RectifyImages");
  ros::NodeHandle nh;
  DisparityClass disparityClass(sizee);
  //n is in the node's namespace

  ros::NodeHandle n("~");
  n.getParam("calibration_file", calibration_file);
  ROS_INFO("calibration file loaded was %s", calibration_file.c_str());

  //define the subscriber and publisher
  ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
  ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);

  pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_output",1);
  // define the angle subscriper
  ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",10,right_pan_callback);
  ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",10,left_pan_callback);
  ros::Subscriber baseline_sub = nh.subscribe("/baseline/position",10,baseline_callback);
  //////////////////////////////////////////////////////////////////


  // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
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


  //Load the calibraton date from the raw dat
  string filename = "/home/abdulla/dev/activeStereoVisionPlatform/src/platform_vision/calibration/" + calibration_file + ".xml";
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


  // print them out
  bool print_calibration_data = false;
  if(print_calibration_data){
    cout << "interinsic1" << A1 << endl;
    cout << "\ninterinsic2" << A2 << endl;
    cout << "\ndistCoeffs1" << D1 << endl;
    cout << "\ndistCoeffs2" << D2 << endl;
    cout << "\nR" << R << endl;
    cout << "\nT" << T << endl;
    cout << "\nE" << E << endl;
    cout << "\nF" << F << endl;
  }

    float old_sum= 0;
    bool angle_chang = true;
    Mat mapx1, mapy1, mapx2, mapy2;
    int np_img =1;
    Rect validRoi[2];

    bool show_images = false;
    //Show images
    cv::namedWindow("Disparity map", WINDOW_NORMAL);
    resizeWindow("Disparity map",1024,600);
    setMouseCallback("Disparity map", SelectROI);

    if (show_images){
      cv::namedWindow("left", WINDOW_NORMAL);
      resizeWindow("left",1024,600);

      setMouseCallback("left", SelectROI);
      cv::namedWindow("depth image", WINDOW_NORMAL);
      resizeWindow("depth image",1024,600);
    }



    int depth_threshold_threshold = 178;
    int number_image_saved = 0;
    bool start_save_image = false;
    // start the while loop
    Mat newRotating;
    while(nh.ok()){

        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey(1);
        if(!left_img.empty() && !right_img.empty()){
          disparityClass.update_trackbar_reading_and_odds_numbers();
          Mat img_right = right_img;
          Mat img_left = left_img;
            // while(nh.ok()){
              double matching_time;
            matching_time = (double)getTickCount();
            // cheack if the total difference bwtween the left angle and the right angle has change
            float angle_sum = abs((left_pan) - (right_pan));
            // cout<<"angle sum: " << angle_sum << endl;

            if(angle_sum != (old_angle_sum) || baseline_value_old != baseline_value){
              cout<<"Update the transformatio Matrix!!" <<endl;
                 // calculate the image angle and convert angle to radian
                 float img_angle = 0.964*angle_sum + 0.5786;
                 // Calculate the rotating matrix
                 Vec3f theta = angles;
                 //theta[0] = angle_x;
                 //theta[2] = angle_z;
                 theta[1] = img_angle;
                 theta = theta* 0.0174533;
                 newRotating = Mat::zeros(3, 3, CV_64FC1);
                 newRotating = eulerAnglesToRotationMatrix(theta);
                 //update the baseline
                 T.at<float> (0,0) = baseline_value;
                //  T.at<float> (0,1) = T.at<float> (0,1) + baseline_value * tan(-0.6 * 0.0174533);

                // calculate the the new projection matrix using bouguet algorithm
                sizee = left_img.size();
                // cout<<"Image size: " << sizee <<endl;
                P1 = 0;
                P2 = 0;
                stereoRectify(A1, D1,
                        A2, D2,
                        sizee,
                        newRotating, T,
                        R1, R2,
                        P1, P2,
                        Q,
                        CALIB_ZERO_DISPARITY, // to allowed zero disparity
                               1, sizee,
                              &validRoi[0], &validRoi[1]);

                // initialize the undistortion rectify map for each camera
                initUndistortRectifyMap(A1, D1, R1,
                            P1, sizee, CV_32FC1, mapx1, mapy1);

                initUndistortRectifyMap(A2, D2, R2,
                            P2, sizee, CV_32FC1, mapx2, mapy2);

                //return angle change to false so to not do rectification process if the angle doen't change
                old_angle_sum = angle_sum;
                baseline_value_old = baseline_value;
                // cout<< Q << endl;

                // Q.at<double>(2,3)=2524.259244299837;  //Focal
                Q.at<double>(3,2)= -1.0/(baseline_value - 0.005);    //1.0/BaseLine

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
            // remap(left_img, undisFrame1, mapx1, mapy1, INTER_LINEAR, BORDER_CONSTANT, Scalar());
            // remap(right_img, undisFrame2, mapx2, mapy2, INTER_LINEAR, BORDER_CONSTANT, Scalar());
            remap(img_right, undisFrame1, mapx1, mapy1, INTER_LINEAR, BORDER_CONSTANT, Scalar());
            remap( img_left, undisFrame2, mapx2, mapy2, INTER_LINEAR, BORDER_CONSTANT, Scalar());



            Mat image [2] = {undisFrame1,undisFrame2};
            for( int k = 0; k < 2; k++ )
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

            // namedWindow("rectified",WINDOW_AUTOSIZE);
            // imshow("rectified", canvas);
            // imshow("left", undisFrame1);
            // imshow("right", undisFrame2);

            Mat undisFrame1_gray, undisFrame2_gray;
            cv::cvtColor(undisFrame1,undisFrame1_gray,CV_BGR2GRAY);
            cv::cvtColor(undisFrame2,undisFrame2_gray,CV_BGR2GRAY);



            if (false){
              cv::Size boardSize = cv::Size(8,6);
              double epi_error = disparityClass.calculate_projection_error(undisFrame1_gray, undisFrame2_gray,boardSize, true);
              cout << "Epipolo error: " << epi_error <<endl;
            }

            //detect the corners in the tested pattern
            Size patten_size = Size(8, 6);
            // vector <Point2f> left_corners = detect_points_on_test_object(undisFrame1_gray, patten_size);
            // vector <Point2f> right_corners = detect_points_on_test_object(undisFrame2_gray, patten_size);
            // calculate_position_of_given_point(left_corners[1], right_corners[1], P1, P2);

            // cout << "00000"<<endl;
            if(resize_image_for_disparity){
              cv::Size disparity_image = Size(640,420); // Size(1344,376); //Size(640,420); // Size(320,240);
              cv::resize(undisFrame1_gray, undisFrame1_gray, disparity_image);
              cv::resize(undisFrame2_gray, undisFrame2_gray, disparity_image);
            }

            // cout << "undisFrame1_gray" << undisFrame1_gray.size() <<endl;
            // equalizeHist(undisFrame1_gray, undisFrame1_gray);
            // equalizeHist(undisFrame2_gray, undisFrame2_gray);

            Mat disp = disparityClass.ComputerDisparity(undisFrame1_gray,undisFrame2_gray);
            if(resize_image_for_disparity){
              resize(disp, disp,sizee);
              GaussianBlur( disp, disp, Size( 3, 3 ), 0, 0 );
            }
           // cout << "Diaprity Size" << disp.size()<<endl;
           Rect New_window_size;
           New_window_size.x = validRoi[0].x + 20;
           New_window_size.y = validRoi[0].y + 20;
           New_window_size.width = validRoi[0].width - 80;
           New_window_size.height = validRoi[0].height - 80;


           disp = disp(New_window_size);
           undisFrame1 = undisFrame1(New_window_size);
           // Mat processPointCloud;
           Mat RG, BY;
           tie (RG, BY) = BGRColorOppenetProcess(undisFrame1);
           // Mat mask;
           cv::bitwise_and(undisFrame1,undisFrame1,undisFrame1,RG);
           // cout << disp.size() << "  " << RG.size() <<endl;
           Mat dispMask;
           cv::bitwise_and(disp, disp, dispMask, RG);
           // threshold(disp, disp,disp, 255, 3 );
           cv::imshow("dispMask",dispMask);
           /////////////////////////////////////////////////
           // Find the measurement
           Mat pointCloud;
           // drawBourderAroundObject(undisFrame1, &disp);
          reprojectImageTo3D(dispMask, pointCloud, Q, true, CV_32F);



          // cout << "44444"<<endl;
          //Calculate the point cloud
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud =  disparityClass.MatToPoinXYZ(disp, pointCloud, undisFrame1, -baseline_value/2);
          if (mouseMove == true){
           // This just to find the object
           Mat cropedDisparity = disp(selectedRectangle);
           Mat leftImageCroped = undisFrame1(selectedRectangle);
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud =  disparityClass.MatToPoinXYZ(cropedDisparity, pointCloud, leftImageCroped);
         }
         // cout << "555555"<<endl;
           // viewer.showCloud(pointcloud);  // To active this check the begining of the main function to define the viewer
           sensor_msgs::PointCloud2 output;
           pcl::toROSMsg(*pointcloud, output);
           output.header.stamp = ros::Time::now();
           output.header.frame_id = "depth_camera";
           pointCloud_pub.publish(output);

          /////////////////////////////////////////////////////////////////////
           // Compute the depth instead Disparity
           Mat depth_image[3];
           split(pointCloud, depth_image);
           Mat depth_threshold_img;
          //  threshold( depth_image[2], depth_threshold_img,depth_threshold_threshold/100, 0, 4 );

          if (mouseMove == true){
            // draw rec in the senter of area
            int xmin = selectedPoint1.x, xmax= selectedPoint2.x;
            int ymin = selectedPoint1.y, ymax = selectedPoint2.y;

             // calculate the depth
             pointCloud = pointCloud(Range(ymin,ymax),Range(xmin,xmax));
             //pointCloud = pointCloud * 100;
             Mat z_roi(pointCloud.size(),CV_32FC1);
             int from_to[]={2,0};

             cv::mixChannels	(&pointCloud,
             1,
             &z_roi,
             1,
             from_to,
             1);

             cout<<"Depth : "<<mean(pointCloud) << " mm"<<endl;
            //ROS_INFO("Depth: %.2f , %.2f , %.2f", mean(pointCloud) );
         }
            ///////////////////////////////////////////////
            //matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            //cout.precision(2);
            //cout<<"Speed:  "<<1/matching_time<<" FPS"<<endl;
            float FPS =1/matching_time;
            std::string s = SSTR( FPS );
            s = s + " FPS";
            cvtColor(disp,disp,CV_GRAY2BGR);
            putText(disp,s,Point(20,20),5,1,Scalar(0,255,0));
            DrawBox_around_selected_point(&disp);


            imshow("Disparity map", disp);
            // createTrackbar("Threshold","Disparity map",&depth_threshold_threshold,255);
            // disparityClass.showColoredDisparity(disp);

            if(show_images){
              //Color Disparity
              imshow("left", undisFrame1);

              imshow("depth image",  depth_image[2]);
              createTrackbar("Threshold","depth image",&depth_threshold_threshold,255);
            }
            // double min, max;
            // cv::minMaxLoc(depth_thresholdMat color_disparity;_img, &min, &max);
            // cout<<"Max: " << max << "\tMin: " << min <<endl;


            if(start_save_image == true){
              stringstream ssl;
              ssl<<"/home/abdulla/dev/images/001_100mm/001_ball_120mm/"<<number_image_saved <<".jpg";
              string depth_image_name = ssl.str();
              ssl.str("");

              imwrite(depth_image_name,depth_threshold_img);
              number_image_saved ++;
              if(number_image_saved == 50){
                start_save_image = false;
                cout << "Image taken completed!!" << endl;
              }
            }

            //cout << " To take another picture press c, to change the angle a, and q to exit the program: " << endl;

            char ikey = waitKey(1);
            if(ikey == 'x'){
              start_save_image = true;
            }


            if(ikey == 'q'){
              break;
            }else if(ikey == 'z'){
              // save the images
            }
        }// end of the if statment of the images
    }


return 1;
}
