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

vector <Point2f> detect_points_on_test_object(Mat img, Size boardSize){
  // Initialize the points storage and find the corners
  vector<cv::Point2f> detected_corner;

  bool patternWasFound = findChessboardCorners(img, boardSize, detected_corner );

  // Calculate the subpixel for more accurcy
  if (patternWasFound){
      cv::cornerSubPix(img, detected_corner, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
  }
  bool showimg = false;
  if (showimg){
      drawChessboardCorners(img, boardSize, detected_corner, patternWasFound);
      cv::namedWindow("epipolar error",WINDOW_NORMAL);
      imshow("epipolar error", img);
      resizeWindow("epipolar error", 600,300);
      cv::waitKey(5);
  }
  return detected_corner;
}


vector <Point3f> return_3d_position_of_2d_points(Mat pointcloud, std::vector<Point2f> point){

  int mean_size = 5;
  vector <Point3f> points_in_3d;
  for(int i =0; i < point.size(); i++){
    Mat pointCloud = pointcloud;
    int xmin = point[i].x - mean_size , xmax = point[i].x + mean_size;
    int ymin = point[i].y - mean_size , ymax = point[i].y + mean_size;
    // calculate the depth
    pointCloud = pointCloud(Range(ymin,ymax),Range(xmin,xmax));
    Point3f xyz_point;
    xyz_point.x = mean(pointCloud)[0];
    xyz_point.y = mean(pointCloud)[1];
    xyz_point.z = mean(pointCloud)[2];
    points_in_3d.push_back(xyz_point);
    // cout << xyz_pointq << endl;
 }
  // cout<<"Position : "<<points_in_3d << " m"<<endl;
  return points_in_3d;
}

Mat equalize_image_using_histograme(Mat img){
  vector<Mat> channels;
  Mat img_hist_equalized;

  cvtColor(img, img_hist_equalized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format

  split(img_hist_equalized,channels); //split the image into channels

  equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)

  merge(channels,img_hist_equalized); //merge 3 channels including the modified 1st channel into one image

  cvtColor(img_hist_equalized, img_hist_equalized, CV_YCrCb2BGR); //change the color image from YCrCb to BGR format (to display image properly)

  return img_hist_equalized;

}

/// This function to convert to cloud point
ros::Publisher pointCloud_pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPoinXYZ(Mat disp, Mat xyz ,Mat undisFrame1)
 {
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new   pcl::PointCloud<pcl::PointXYZRGB>());
  //  Mat xyz;
  //  reprojectImageTo3D(disp, xyz, Q, false, CV_32F);
   pointcloud->is_dense = false;
   pcl::PointXYZRGB point;
   for (int i = 0; i < disp.rows; ++i)
       {
           uchar* rgb_ptr = undisFrame1.ptr<uchar>(i);
           uchar* disp_ptr = disp.ptr<uchar>(i);
           double* xyz_ptr = xyz.ptr<double>(i);

           for (int j = 0; j < disp.cols; ++j)
           {
               uchar d = disp_ptr[j];
               if (d == 0) continue;
               Point3f p = xyz.at<Point3f>(i, j);

               point.z = p.z;   // I have also tried p.z/16
               point.x = p.x;
               point.y = p.y;

               point.b = rgb_ptr[3 * j];
               point.g = rgb_ptr[3 * j + 1];
               point.r = rgb_ptr[3 * j + 2];
               pointcloud->points.push_back(point);
           }
       }

    pointcloud->width = (int)pointcloud->points.size();
    pointcloud->height = 1;

     return pointcloud;

 }



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
float error_margen_for_motor_controller = 0.3;
int BloackSize = 5;
int NoDisparity = 255;

int SpeckleWindowSize = 0;
int UniquenessRatio = 2;
int setTextureThreshold = 0;
int setMinDisparity = 0;
int setPreFilterCap = 62;
int setPreFilterSize = 51;
int lambda = 110; //5000.0;
int sigma = 15;
int prefilterType = 1;
int setSpeckleRange = 0;
int dispP1 = 100;
int dispP2 = 1000;
int FullDP = 1;
int setDisp12MaxDiff = 20;
int Disparity_color_scales = 2;
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

    // left_img =  equalize_image_using_histograme(cv_img_msg->image);
    left_img =  cv_img_msg->image;
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

    // right_img =  equalize_image_using_histograme(cv_img_msg->image);
    right_img =  cv_img_msg->image;
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


//This function use in opencv to return odd numbers only
void odd_number_callback(int value, void*)
{
    if(value >= 5){
        // kernel size must be positive and odd
        setPreFilterSize = (value % 2) ? value: value + 1;
    }else{
        setPreFilterSize = 5;
    }
}


Mat ComputerDisparity(Mat left, Mat right, char mode = '2');
double calculate_projection_error(cv::Mat left, cv::Mat right, cv::Size boardSize, bool showimg);
void findCorners(Mat imgLeft, Mat imgRight);
Mat FilterDisp(Mat left, Mat right, Rect roi1,Rect roi2);
char mode = '2';

void update_trackbar_reading_and_odds_numbers(){

  namedWindow("trackbar");
  createTrackbar("BloackSize","trackbar",&BloackSize,50);
  createTrackbar("NoDisparity","trackbar",&NoDisparity,1000);
  createTrackbar("setMinDisparity","trackbar",&setMinDisparity,1000);
  createTrackbar("setDisp12MaxDiff","trackbar",&setDisp12MaxDiff,200);
  createTrackbar("SpeckleWindowSize","trackbar",&SpeckleWindowSize,200);
  createTrackbar("setSpeckleRange","trackbar",&setSpeckleRange,10);
  createTrackbar("UniquenessRatio","trackbar",&UniquenessRatio,120);
  createTrackbar("setTextureThreshold","trackbar",&setTextureThreshold,5000);
  createTrackbar("setPreFilterCap","trackbar",&setPreFilterCap,63);
  createTrackbar("setPreFilterSize","trackbar",&setPreFilterSize,255,odd_number_callback);
  createTrackbar("P1","trackbar",&dispP1, 1000);
  createTrackbar("P2","trackbar",&dispP2, 1000);
  createTrackbar("lambda","trackbar",&lambda,10000);
  createTrackbar("sigma","trackbar",&sigma,1000);
  createTrackbar("prefilterType","trackbar",&prefilterType,1);
  createTrackbar("FullDP","trackbar",&FullDP,2);


  if (setPreFilterCap % 2 == 0)
  {
      setPreFilterCap = setPreFilterCap + 1;
  }


  if (setPreFilterCap < 5)
  {
      setPreFilterCap = 5;
    }

  if (BloackSize % 2 == 0)
  {
      BloackSize = BloackSize + 1;
  }

  if (BloackSize < 5)
  {
      BloackSize = 5;
  }

  if (NoDisparity % 16 != 0)
  {
      NoDisparity = NoDisparity + (16 - NoDisparity % 16);
  }
}


/////////////////////////////////////////////////////////////////////
/////These variables for selecting the regoin
bool mouseMove = false;
Point selested_point_for_diaprity_point_1 = Point(sizee.width/2 - 30, sizee.height/2 - 30);
Point selested_point_for_diaprity_point_2 = Point(sizee.width/2 + 30, sizee.height/2 + 30);
Mat Qvect;

//This function use to located a point when mouse click
Rect rectangle_of_disparity_map_to_be_calculated;
void SelectROI(int event, int x, int y, int flags, void* data){
  if (event == EVENT_LBUTTONDOWN && mouseMove == false){
    int rectangle_size = 10;
    selested_point_for_diaprity_point_1 = Point(x - rectangle_size, y - rectangle_size);
    selested_point_for_diaprity_point_2 = Point(x + rectangle_size, y + rectangle_size);
    rectangle_of_disparity_map_to_be_calculated = Rect(selested_point_for_diaprity_point_1, selested_point_for_diaprity_point_2);
    mouseMove = true;
  }else if (event == EVENT_LBUTTONDOWN && mouseMove == true){
    mouseMove = false;
  }


}

// This function to draw the box in the image
void DrawBox_around_selected_point(Mat *orginalImage){

    if (mouseMove == true){
        rectangle(*orginalImage, rectangle_of_disparity_map_to_be_calculated, Scalar(0, 0, 255), 1, 8, 0);
    }
}


int Hmin = 0;
int Hmax = 91;
int Smin = 112;
int Smax = 255;
int Vmin = 14;
int Vmax = 160;
Mat erodeElmt = getStructuringElement(MORPH_RECT, Size(13, 13));
Mat dilateElmt = getStructuringElement(MORPH_RECT, Size(13, 13));

void drawBourderAroundObject(Mat color_img, Mat *output_image){
  Mat dis_img;
  /// Convert it to gray
  // cvtColor( undisFrame1, src_gray, CV_BGR2GRAY );
  cv::cvtColor(color_img, dis_img, CV_BGR2HSV);
  //cv::inRange(dis_img,Scalar(red.Hmin,red.Smin,red.Vmin),Scalar(red.Hmax,red.Smax,red.Vmax),dis_img);
  cv::inRange(dis_img,Scalar( Hmin, Smin, Vmin),Scalar( Hmax, Smax, Vmax),dis_img);
  erode(dis_img,dis_img,erodeElmt);
  dilate(dis_img,dis_img,dilateElmt);
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  // Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( dis_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
       minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
     }


  for( int i = 0; i< contours.size(); i++ )
     {
       if( (int)radius[i] > 80){
         circle( *output_image, center[i], (int)radius[i]+5, Scalar(0,0,0), 10, 8, 0 );
       }
     }
}

int main(int argc,char** argv)
{
  bool resize_image_for_disparity = true;
  // std::cout << argv[1] << std::endl;
  bool my_bool = argv[1];
  if (my_bool != false){
    resize_image_for_disparity = false;
    std::cout << "Full Size Image Processing !!!" << std::endl;
  }else{
    std::cout << "Small Size Image Processing !!!" << std::endl;
  }
  //////////////////////////////////////
  ros::init(argc,argv,"RectifyImages");
  ros::NodeHandle nh;
  //n is in the node's namespace
  string calibration_file = "003_used_data";
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


  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
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
  string filename = "/home/abdulla/dev/Active-stereo-Vision-Platform/calibration_data/" + calibration_file + ".xml";
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

    cv::namedWindow("Color Disparity map", WINDOW_NORMAL);
    resizeWindow("Color Disparity map",1024,600);


    if (show_images){
      cv::namedWindow("left", WINDOW_NORMAL);
      resizeWindow("left",1024,600);

      setMouseCallback("left", SelectROI);
      cv::namedWindow("Color Disparity map", WINDOW_NORMAL);
      resizeWindow("Color Disparity map",1024,600);

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
        ikey = cv::waitKey('q');
        if(!left_img.empty() && !right_img.empty()){
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
              double epi_error = calculate_projection_error(undisFrame1_gray, undisFrame2_gray,boardSize, true);
              cout << "Epipolo error: " << epi_error <<endl;
            }

            //detect the corners in the tested pattern
            Size patten_size = Size(8, 6);
            // vector <Point2f> left_corners = detect_points_on_test_object(undisFrame1_gray, patten_size);
            // vector <Point2f> right_corners = detect_points_on_test_object(undisFrame2_gray, patten_size);
            // calculate_position_of_given_point(left_corners[1], right_corners[1], P1, P2);


            if(resize_image_for_disparity){
              cv::Size disparity_image = Size(640,420); // Size(1344,376); //Size(640,420); // Size(320,240);
              cv::resize(undisFrame1_gray, undisFrame1_gray, disparity_image);
              cv::resize(undisFrame2_gray, undisFrame2_gray, disparity_image);
            }


            // equalizeHist(undisFrame1_gray, undisFrame1_gray);
            // equalizeHist(undisFrame2_gray, undisFrame2_gray);
            update_trackbar_reading_and_odds_numbers();
            Mat disp = ComputerDisparity(undisFrame1_gray,undisFrame2_gray, mode);
            if(resize_image_for_disparity){
              resize(disp, disp,sizee);
              GaussianBlur( disp, disp, Size( 3, 3 ), 0, 0 );
            }

           Rect New_window_size;
           New_window_size.x = validRoi[0].x + 20;
           New_window_size.y = validRoi[0].y + 20;
           New_window_size.width = validRoi[0].width - 80;
           New_window_size.height = validRoi[0].height - 80;


           disp = disp(New_window_size);
           undisFrame1 = undisFrame1(New_window_size);
           // threshold(disp, disp,depth_threshold_threshold, 255, 3 );
           /////////////////////////////////////////////////
           // Find the measurement
           Mat pointCloud;
           // drawBourderAroundObject(undisFrame1, &disp);
          reprojectImageTo3D(disp, pointCloud, Q, true, CV_32F);

          //Calculate the point cloud
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud =  MatToPoinXYZ(disp, pointCloud, undisFrame1);
          if (mouseMove == true){
           // This just to find the object
           Mat cropedDisparity = disp(rectangle_of_disparity_map_to_be_calculated);
           Mat leftImageCroped = undisFrame1(rectangle_of_disparity_map_to_be_calculated);
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud =  MatToPoinXYZ(cropedDisparity, pointCloud, leftImageCroped);
         }

           viewer.showCloud(pointcloud);  // To active this check the begining of the main function to define the viewer
           sensor_msgs::PointCloud2 output;
           pcl::toROSMsg(*pointcloud, output);
           output.header.stamp = ros::Time::now();
           output.header.frame_id = "camera_link";
           pointCloud_pub.publish(output);

          /////////////////////////////////////////////////////////////////////
           // Compute the depth instead Disparity
           Mat depth_image[3];
           split(pointCloud, depth_image);
           Mat depth_threshold_img;
          //  threshold( depth_image[2], depth_threshold_img,depth_threshold_threshold/100, 0, 4 );

          if (mouseMove == true){
            // draw rec in the senter of area
            int xmin = selested_point_for_diaprity_point_1.x, xmax= selested_point_for_diaprity_point_2.x;
            int ymin = selested_point_for_diaprity_point_1.y, ymax = selested_point_for_diaprity_point_2.y;

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
            createTrackbar("Threshold","Disparity map",&depth_threshold_threshold,255);


            Mat color_disparity;
            applyColorMap(disp, color_disparity,Disparity_color_scales);
            createTrackbar("Color Disparity","Color Disparity map",&Disparity_color_scales,12);
            imshow("Color Disparity map", color_disparity);

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

            char ikey = waitKey('q');
            if(ikey == 'x'){
              start_save_image = true;
            }


            if(ikey == 'q'){
              break;
            }else if(ikey == 'z'){  // save the images
            stringstream ssl;
            ssl<<"/home/abdulla/dev/images/001_BM/"<<np_img << "_left" <<".jpg";
            string left_name = ssl.str();
            ssl.str("");

            stringstream ssr;
            ssr<<"/home/abdulla/dev/images/001_BM/"<<np_img << "_right" <<".jpg";
            string right_name = ssr.str();
            ssr.str("");

            stringstream ssb;
            ssb<<"/home/abdulla/dev/images/001_BM/" <<np_img << "_both_img" << ".jpg";
            string both_name = ssb.str();
            ssb.str("");

            stringstream depth_ss;
            depth_ss<<"/home/abdulla/dev/images/001_BM/disparity/" <<np_img << "disparity" << ".jpg";
            string depth_name = depth_ss.str();
            ssb.str("");

            stringstream depth_color_ss;
            depth_color_ss<<"/home/abdulla/dev/images/001_BM/disparity_color/" <<np_img << "disparity" << ".jpg";
            string depth_color_name = depth_color_ss.str();
            ssb.str("");

            imwrite(left_name,undisFrame1);
            imwrite(right_name,undisFrame2);
            imwrite(both_name,canvas);
            imwrite(depth_name,disp);
            imwrite(depth_color_name,color_disparity);
            cout<< "image save : "<<np_img<<endl;
            np_img++;

            }else if (ikey == '1'){
                mode = '1';
                cout<< "Mode: " << mode<< "  BM" << endl;
            }else if (ikey == '2'){
                mode = '2';
                cout<< "Mode: " << mode<< "  SGBM" << endl;
            }else if (ikey == '3'){
                mode = '3';
                cout<< "Mode: " << mode<< "  BM_filter" << endl;
            }else if (ikey == '4'){
                mode = '4';
                cout<< "Mode: " << mode<< "  SGBM_filter" << endl;
            }else if (ikey == '5'){
                mode = '5';
                cout<< "Mode: " << mode<< "   Stereo Belief Propagation" << endl;
            }else if (ikey == '6'){
                mode = '6';
                cout<< "Mode: " << mode<< "   Stereo Constant Space BP" << endl;
            }else if (ikey == '7'){
                mode = '7';
                cout<< "Mode: " << mode<< "   Cuda SGBM" << endl;
            }
        }// end of the if statment of the images
    }


return 1;


}

//this function use to calculate the diaprity
Mat ComputerDisparity(Mat left, Mat right,char mode){
    int Disparity_formate = CV_8U; //CV_32F; //CV_8U
    Mat g1, g2;
    Mat disp, disp8, filtered_disp;
    left.copyTo(g1);
    right.copyTo(g2);
    Mat left_disp, right_disp;
    ///////////////////////////////////////////////

    if(mode == '1'){
         Ptr<StereoBM> left_matcher = StereoBM::create(NoDisparity,7);
        left_matcher->setDisp12MaxDiff(1);
        left_matcher->setSpeckleRange(8);
        left_matcher->setBlockSize(BloackSize);
        left_matcher->setSpeckleWindowSize(SpeckleWindowSize);
        left_matcher->setUniquenessRatio(UniquenessRatio);
        left_matcher->setTextureThreshold(setTextureThreshold);
        left_matcher->setMinDisparity(-setMinDisparity);
        left_matcher->setPreFilterType(prefilterType);
        left_matcher->setPreFilterSize(setPreFilterSize);
        left_matcher->setPreFilterCap(setPreFilterCap);
        left_matcher->compute(g1,g2,disp);
        normalize(disp, disp8, 0, 256, CV_MINMAX, Disparity_formate);

    }else if(mode == '2'){
      int blockSize = 9 ;
      int Disp12MaxDiff = 3 ;
      int SpeckleRange = 32 ;

      cv :: Ptr <cv :: StereoSGBM> sgbm = cv :: StereoSGBM :: create (
      -setMinDisparity,
      NoDisparity,
      blockSize,dispP1, dispP2, setDisp12MaxDiff, setPreFilterCap, UniquenessRatio, SpeckleWindowSize, setSpeckleRange, FullDP);
      // sgbm->setDisp12MaxDiff(1);
      // sgbm->setSpeckleRange(setSpeckleRange);
      sgbm->setBlockSize(BloackSize);

      sgbm->compute(g1,g2,disp);

      normalize(disp, disp8, 0, 256, CV_MINMAX, Disparity_formate);
      // disp.convertTo(disp8, Disparity_formate, 255. /(NoDisparity * 16));

    }else if(mode == '3'){
      Ptr<DisparityWLSFilter> wls_filter;
       Ptr<StereoBM> left_matcher = StereoBM::create(NoDisparity,7);
       wls_filter = createDisparityWLSFilter(left_matcher);

       left_matcher->setDisp12MaxDiff(1);
       left_matcher->setSpeckleRange(8);
       left_matcher->setBlockSize(BloackSize);
       left_matcher->setSpeckleWindowSize(SpeckleWindowSize);
       left_matcher->setUniquenessRatio(UniquenessRatio);
       left_matcher->setTextureThreshold(setTextureThreshold);
       left_matcher->setMinDisparity(-setMinDisparity);
       left_matcher->setPreFilterType(prefilterType);
       left_matcher->setPreFilterSize(setPreFilterSize);
       left_matcher->setPreFilterCap(setPreFilterCap);

      Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
      left_matcher-> compute(g1, g2,left_disp);
      right_matcher->compute(g2, g1, right_disp);


      //preform filter process
      wls_filter->setLambda(lambda);
      wls_filter->setSigmaColor(sigma/10);

      wls_filter->filter(left_disp,left,filtered_disp,right_disp);

    //  // finalize the image
    //  Mat filtered_disp_vis;
    //  double vis_mult = 12.0;
    //  getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    //  disp8 = filtered_disp_vis;
    normalize(filtered_disp, disp8, 0, 256, CV_MINMAX, Disparity_formate);

    }else if (mode == '4'){

            Ptr<DisparityWLSFilter> wls_filter;
            int Window_size = BloackSize ;
            Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,NoDisparity,Window_size);
            left_matcher->setMinDisparity(-setMinDisparity);
            left_matcher->setP1(24*Window_size*Window_size);
            left_matcher->setP2(96*Window_size*Window_size);
            left_matcher->setPreFilterCap(setPreFilterCap);
            left_matcher->setUniquenessRatio(UniquenessRatio);
            left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            left_matcher-> compute(g1, g2,left_disp);
            right_matcher->compute(g2,g1, right_disp);
            //preform filter process
            wls_filter->setLambda(lambda);
            wls_filter->setSigmaColor(sigma/10);
            wls_filter->filter(left_disp,left,filtered_disp,right_disp);

            // finalize the image
//           Mat filtered_disp_vis;
//           double vis_mult = 12.0;
//           getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
//           disp8 = filtered_disp_vis;
            normalize(filtered_disp, disp8, 0, 256, CV_MINMAX, Disparity_formate);
    }else if(mode == '5'){

        cuda::GpuMat d_left, d_right, d_disp;
        d_left.upload(g1);
        d_right.upload(g2);

        Ptr<cuda::StereoBeliefPropagation> bp;
        bp = cuda::createStereoBeliefPropagation(NoDisparity,5,5,5);
        bp->compute(d_left, d_right, d_disp);

        d_disp.download(disp);
        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    }else if(mode == '6'){
        cuda::GpuMat d_left, d_right, d_disp;
        d_left.upload(g1);
        d_right.upload(g2);
        Ptr<cuda::StereoConstantSpaceBP> csbp;
        csbp = cv::cuda::createStereoConstantSpaceBP(NoDisparity,8,4,4,5);
        csbp->compute(d_left, d_right, d_disp);
        d_disp.download(disp);
        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    }else if(mode == '7'){
        cuda::GpuMat d_left, d_right, d_disp;
        d_left.upload(g1);
        d_right.upload(g2);
        Ptr<cuda::StereoBM> bm;
        bm = cuda::createStereoBM(NoDisparity,BloackSize);
        bm->setSpeckleWindowSize(SpeckleWindowSize);
        bm->setUniquenessRatio(UniquenessRatio);
        bm->setTextureThreshold(setTextureThreshold);
        bm->setMinDisparity(-setMinDisparity);
        bm->setPreFilterType(prefilterType);
        bm->setPreFilterSize(setPreFilterSize);
        bm->setPreFilterCap(setPreFilterCap);
        bm->compute(d_left, d_right, d_disp);
        d_disp.download(disp);
        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    }else{
        mode = '1';
    }



    char ikey = waitKey(10);
    if(ikey == 'w'){
        BloackSize +=2;
        cout <<"Blaock Size: "<<BloackSize<<endl;
    }else if(ikey == 's'){
        if (BloackSize <= 5){
             BloackSize =5;
        }else{
             BloackSize -=2;
        }

        cout <<"Blaock Size: "<<BloackSize<<endl;
    }else if(ikey == 'e'){
        NoDisparity +=16;
        cout <<"NoDisparity: "<<NoDisparity<<endl;
    }else if(ikey == 'd'){
        if(NoDisparity <= 16){
            NoDisparity =16;
        }else{
            NoDisparity -= 16;
        }
        cout <<"NoDisparity: "<<NoDisparity<<endl;
    }


    return disp8;
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
