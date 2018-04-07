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

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

class Generate_disparity_map{

public:
  Mat Disparity(Mat left, Mat right,char mode, bool resize_image_for_disparity = true);
  double calculate_projection_error(cv::Mat left, cv::Mat right, cv::Size boardSize, bool showimg);
  Mat calculate_the_depth_map_from_disparity(Mat disparity, Mat Q);
  Generate_disparity_map(){ //Set the parameters to calculate the disparity
    BloackSize = 5;
    NoDisparity = 255;
    SpeckleWindowSize = 0;
    UniquenessRatio = 2;
    setTextureThreshold = 0;
    setMinDisparity = 0;
    setPreFilterCap = 62;
    setPreFilterSize = 51;
    lambda = 5000.0;
    sigma = 15;
    prefilterType = 0;
    setSpeckleRange = 0;
    dispP1 = 8 * 3 * BloackSize * BloackSize;
    dispP2 = 32 * 3 * BloackSize * BloackSize;
    FullDP = 0;
    setDisp12MaxDiff = 12;
    Disparity_color_scales = 2;
    mode = '4';
    np_img = 0;

    LButtonPress = false;
    mouseMove = false;
    ROISelected = false;
    p1 = Point(sizee.width/2 - 20, sizee.height/2 - 20);
    p2 = Point(sizee.width/2 + 20, sizee.height/2 + 20);

    //Show images
    namedWindow("trackbar");
    cv::namedWindow("Disparity map", WINDOW_NORMAL);
    resizeWindow("Disparity map",1024,600);

    cv::namedWindow("Color Disparity", WINDOW_NORMAL);
    resizeWindow("Color Disparity",1024,600);
  }

  void convert_to_color_disparity(Mat disp){
    //Color Disparity
    Mat color_disparity;
    applyColorMap(disp, color_disparity,Disparity_color_scales);
    createTrackbar("Color Disparity","Color Disparity",&Disparity_color_scales,12);
    imshow("Color Disparity", color_disparity);
  }

  void show_disparity_map(Mat disp){
    cvtColor(disp,disp,CV_GRAY2BGR);
    DrawBox(disp);
    rectangle(disp,SlectedBox,Scalar(0,255,0));
    imshow("Disparity map", disp);
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


  // This function to draw the box in the image
  void DrawBox(Mat orginalImage){

      if (mouseMove == true){
          rectangle(orginalImage, p1,p2, Scalar(0, 0, 255), 1, 8, 0);
      }
  }

  void update_trackbar_reading_and_odds_numbers(){

    createTrackbar("BloackSize","trackbar",&BloackSize,50);
    createTrackbar("NoDisparity","trackbar",&NoDisparity,1000);
    createTrackbar("setMinDisparity","trackbar",&setMinDisparity,1000);
    createTrackbar("setDisp12MaxDiff","trackbar",&setDisp12MaxDiff,200);
    createTrackbar("SpeckleWindowSize","trackbar",&SpeckleWindowSize,200);
    createTrackbar("setSpeckleRange","trackbar",&setSpeckleRange,10);
    createTrackbar("UniquenessRatio","trackbar",&UniquenessRatio,120);
    createTrackbar("setTextureThreshold","trackbar",&setTextureThreshold,5000);
    createTrackbar("setPreFilterCap","trackbar",&setPreFilterCap,63);
    createTrackbar("setPreFilterSize","trackbar",&setPreFilterSize,255);
    createTrackbar("P1","trackbar",&dispP1, 1000);
    createTrackbar("P2","trackbar",&dispP1, 1000);
    createTrackbar("lambda","trackbar",&lambda,10000);
    createTrackbar("sigma","trackbar",&sigma,1000);
    createTrackbar("prefilterType","trackbar",&prefilterType,1);
    createTrackbar("FullDP","trackbar",&FullDP,1);

    if(setPreFilterSize >= 5){
        // kernel size must be positive and odd
        setPreFilterSize = (setPreFilterSize % 2) ? setPreFilterSize: setPreFilterSize + 1;
    }else{
        setPreFilterSize = 5;
    }

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


  void check_key_press(char ikey, Mat img_left, Mat img_right, Mat disp){
    if(ikey == 'q'){
      // break;
    }else if(ikey == 'z'){  // save the images
    stringstream ssl;
    ssl<<"/home/abdulla/dev/workshop/PhD/Trctification_based_on_bouguet/003_image_test/"<<np_img << "_left" <<".jpg";
    string left_name = ssl.str();
    ssl.str("");

    stringstream ssr;
    ssr<<"/home/abdulla/dev/workshop/PhD/Trctification_based_on_bouguet/003_image_test/"<<np_img << "_right" <<".jpg";
    string right_name = ssr.str();
    ssr.str("");

    stringstream depth_ss;
    depth_ss<<"/home/abdulla/dev/workshop/PhD/Trctification_based_on_bouguet/003_image_test/" <<np_img << "disparity" << ".jpg";
    string depth_name = depth_ss.str();
    depth_ss.str("");

    imwrite(left_name,img_left);
    imwrite(right_name,img_right);
    imwrite(depth_name,disp);
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
    }
  }


public:
  // These virables are the value form the offline calibration
  int BloackSize;
  int NoDisparity;
  int SpeckleWindowSize;
  int UniquenessRatio;
  int setTextureThreshold;
  int setMinDisparity;
  int setPreFilterCap;
  int setPreFilterSize;
  int lambda;
  int sigma;
  int prefilterType;
  int setSpeckleRange;
  int dispP1;
  int dispP2;
  int FullDP;
  int setDisp12MaxDiff;
  int Disparity_color_scales;
  char mode;
  // global virable
  Mat left_img, right_img;
  Size sizee = Size(2048 , 1080);

  /////////////////////////////////////////////////////////////////////
  /////These variables for selecting the regoin
  bool LButtonPress;
  bool mouseMove;
  bool ROISelected;
  Point p1;
  Point p2;
  Rect SlectedBox;
  Mat Qvect;

  int np_img;
};

//this function use to calculate the diaprity
Mat Generate_disparity_map::Disparity(Mat left, Mat right,char mode, bool resize_image_for_disparity){

  update_trackbar_reading_and_odds_numbers();
    Mat g1, g2;
    Mat disp, disp8, filtered_disp;
    Size image_zise_before_resize;
    if(resize_image_for_disparity){
      image_zise_before_resize = left.size();
      cv::Size disparity_image = Size(1344,376); //Size(640,420); // Size(320,240);
      cv::resize(left, g1, disparity_image);
      cv::resize(right, g2, disparity_image);
    }else{
      left.copyTo(g1);
      right.copyTo(g2);
    }

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
        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

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

      normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

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
      normalize(filtered_disp, disp8, 0, 255, CV_MINMAX, CV_8U);

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
            normalize(filtered_disp, disp8, 0, 255, CV_MINMAX, CV_8U);
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


    if(resize_image_for_disparity){
      resize(disp8, disp8,image_zise_before_resize);
      GaussianBlur( disp8, disp8, Size( 3, 3 ), 0, 0 );
    }

    return disp8;
}

Mat Generate_disparity_map::calculate_the_depth_map_from_disparity(Mat disparity, Mat Q){

  Mat pointCloud;
  reprojectImageTo3D(disparity,pointCloud,Q, false, CV_32F);
  Mat depth_image[3];
  split(pointCloud, depth_image);
  Mat depth_threshold;
  return depth_image[2];
}
///////////////////////////////////////////
/// \brief calculate_projection_error
double Generate_disparity_map::calculate_projection_error(cv::Mat left, cv::Mat right, cv::Size boardSize, bool showimg){
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
