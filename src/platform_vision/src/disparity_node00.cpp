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

// global virable
Mat left_img, right_img;
Size sizee = Size(2048 , 1080);
float left_pan = 0,right_pan = 0;
float baseline_value = 0.0;
float baseline_value_old = 0.0;
// These virables are the value form the offline calibration
float angle_z = -0.425903209166667;
float angle_x = 0.537310137283333;
float old_angle_sum = 1.0;
int BloackSize = 5;
int NoDisparity = 64;

int SpeckleWindowSize = 3;
int UniquenessRatio = 0;
int setTextureThreshold = 0;
int setMinDisparity = 0;
int setPreFilterCap = 61;
int setPreFilterSize = 5;
int lambda = 5000.0;
int sigma = 15;
int prefilterType = 0;
int setSpeckleRange = 0;
int dispP1 = 8 * 3 * BloackSize * BloackSize;
int dispP2 = 32 * 3 * BloackSize * BloackSize;
int FullDP = 0;
int setDisp12MaxDiff = 0;
int Disparity_color_scales = 2;
// the blow functions are the function use to get the values
void left_img_callback(const sensor_msgs::ImageConstPtr& img_msg){

    // create storage for the comming image in cv format
    cv_bridge::CvImagePtr cv_img_msg;

    //copy the image and save it in opencv formate
    try{
    cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::MONO8);
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
    cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::MONO8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR("cv_bridge expection: %s", e.what());
        return;
    }

    // right_img =  equalize_image_using_histograme(cv_img_msg->image);
    right_img =  cv_img_msg->image;
}

/////////////////////////////////////////////////////////////////////
/////These variables for selecting the regoin
bool LButtonPress = false;
bool mouseMove = false;
bool ROISelected = false;
Point p1 = Point(sizee.width/2 - 20, sizee.height/2 - 20);
Point p2 = Point(sizee.width/2 + 20, sizee.height/2 + 20);
Rect SlectedBox;
Mat Qvect;
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


// This function to draw the box in the image
void DrawBox(Mat orginalImage){

    if (mouseMove == true){
        rectangle(orginalImage, p1,p2, Scalar(0, 0, 255), 1, 8, 0);
    }
}


Mat Disparity(Mat left, Mat right, char mode = '1');
double calculate_projection_error(cv::Mat left, cv::Mat right, cv::Size boardSize, bool showimg);
void findCorners(Mat imgLeft, Mat imgRight);
Mat FilterDisp(Mat left, Mat right, Rect roi1,Rect roi2);
char mode = '1';

void update_trackbar_reading_and_odds_numbers(){

  namedWindow("trackbar");
  createTrackbar("BloackSize","trackbar",&BloackSize,50);
  createTrackbar("NoDisparity","trackbar",&NoDisparity,255);
  createTrackbar("setMinDisparity","trackbar",&setMinDisparity,1000);
  createTrackbar("setDisp12MaxDiff","trackbar",&setDisp12MaxDiff,200);
  createTrackbar("SpeckleWindowSize","trackbar",&SpeckleWindowSize,200);
  createTrackbar("setSpeckleRange","trackbar",&setSpeckleRange,10);
  createTrackbar("UniquenessRatio","trackbar",&UniquenessRatio,120);
  createTrackbar("setTextureThreshold","trackbar",&setTextureThreshold,5000);
  createTrackbar("setPreFilterCap","trackbar",&setPreFilterCap,63);
  createTrackbar("setPreFilterSize","trackbar",&setPreFilterSize,255,odd_number_callback);
  createTrackbar("P1","trackbar",&dispP1, 1000);
  createTrackbar("P2","trackbar",&dispP1, 1000);
  createTrackbar("lambda","trackbar",&lambda,10000);
  createTrackbar("sigma","trackbar",&sigma,1000);
  createTrackbar("prefilterType","trackbar",&prefilterType,1);
  createTrackbar("FullDP","trackbar",&FullDP,1);


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

int np_img = 0;
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

int main(int argc,char** argv)
{

  //////////////////////////////////////
    ros::init(argc,argv,"DisparityMap");
    ros::NodeHandle nh;

    //define the subscriber and publisher
    ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_rect_mono",10,left_img_callback);
    ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_rect_mono",10,right_img_callback);


    image_transport::ImageTransport it(nh);
    image_transport::Publisher disparity_image_pub = it.advertise("stereo/depth/disparity", 1);



    //Show images
    cv::namedWindow("Disparity map", WINDOW_NORMAL);
    resizeWindow("Disparity map",1024,600);

    cv::namedWindow("Color Disparity", WINDOW_NORMAL);
    resizeWindow("Color Disparity",1024,600);



    // start the while loop
    while(nh.ok()){

        ros::spinOnce();
        char ikey;
        if(!left_img.empty() && !right_img.empty()){
          Mat img_right = right_img;
          Mat img_left = left_img;

          cv::Size boardSize = cv::Size(8,6);
          bool check_epi_error = false;
          if (check_epi_error){
            Mat undisFrame1_gray, undisFrame2_gray;
            cv::cvtColor(img_left,undisFrame1_gray,CV_BGR2GRAY);
            cv::cvtColor(img_right,undisFrame2_gray,CV_BGR2GRAY);
            double epi_error = calculate_projection_error(undisFrame1_gray, undisFrame2_gray,boardSize, true);
            cout << "Epipolo error: " << epi_error <<endl;
          }

          // equalizeHist(img_right, img_right);
          // equalizeHist(img_left, img_left);
          update_trackbar_reading_and_odds_numbers();
          cv::Size disparity_image = Size(320,240);
          cv::resize(img_left, img_left, disparity_image);
          cv::resize(img_right, img_right, disparity_image);
          Mat disp = Disparity(img_left,img_right, mode);
          cvtColor(disp,disp,CV_GRAY2BGR);
          DrawBox(disp);
          rectangle(disp,SlectedBox,Scalar(0,255,0));
          imshow("Disparity map", disp);

          //Color Disparity
          Mat color_disparity;
          applyColorMap(disp, color_disparity,Disparity_color_scales);
          createTrackbar("Color Disparity","Color Disparity",&Disparity_color_scales,12);
          imshow("Color Disparity", color_disparity);
          ikey = waitKey('q');
          check_key_press(ikey,img_left, img_right, disp );

          // // Publish image
          // sensor_msgs::ImagePtr disparity_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", disp).toImageMsg();
          // disparity_image_pub.publish(disparity_image_msg);


        }// end of the if statment of the images
    }

return 1;

}

//this function use to calculate the diaprity
Mat Disparity(Mat left, Mat right,char mode){
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
      // sgbm->setSpeckleWindowSize(SpeckleWindowSize);
      // sgbm->setUniquenessRatio(UniquenessRatio);
      // sgbm->setMinDisparity(-setMinDisparity);
      // sgbm->setP1(dispP1);
      // sgbm->setP2(dispP1);
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

    //  // finalize the image
    //  Mat filtered_disp_vis;
    //  double vis_mult = 12.0;
    //  getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    //  disp8 = filtered_disp_vis;
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

            // finalize the image
//           Mat filtered_disp_vis;
//           double vis_mult = 12.0;
//           getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
//           disp8 = filtered_disp_vis;
            normalize(filtered_disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    }
//    else if(mode == '5'){
//        if( debug){
//            time_convert_img = (double)getTickCount();
//        }
//        cuda::GpuMat d_left, d_right, d_disp;
//        d_left.upload(g1);
//        d_right.upload(g2);
//        if( debug){
//            time_convert_img = ((double)getTickCount() - time_convert_img)/getTickFrequency();
//            cout<< "Time to convert Image is: " <<time_convert_img<<endl;
//            time_convert_img = (double)getTickCount();
//        }
//        Ptr<cuda::StereoBeliefPropagation> bp;
//        bp = cuda::createStereoBeliefPropagation(NoDisparity,5,5,5);
//        bp->compute(d_left, d_right, d_disp);
//        if (debug){
//            time_convert_img = ((double)getTickCount() - time_convert_img)/getTickFrequency();
//            cout<< "Time to process Image is: " <<time_convert_img<<endl;
//        }
//        d_disp.download(disp);
//        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

//    }else if(mode == '6'){
//        cuda::GpuMat d_left, d_right, d_disp;
//        d_left.upload(g1);
//        d_right.upload(g2);
//        Ptr<cuda::StereoConstantSpaceBP> csbp;
//        csbp = cv::cuda::createStereoConstantSpaceBP(NoDisparity,8,4,4,5);
//        csbp->compute(d_left, d_right, d_disp);
//        d_disp.download(disp);
//        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
//    }else if(mode == '7'){
//        cuda::GpuMat d_left, d_right, d_disp;
//        d_left.upload(g1);
//        d_right.upload(g2);
//        Ptr<cuda::StereoBM> bm;
//        bm = cuda::createStereoBM(NoDisparity,BloackSize);
//        bm->compute(d_left, d_right, d_disp);
//        d_disp.download(disp);
//        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
//    }
    else{
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
