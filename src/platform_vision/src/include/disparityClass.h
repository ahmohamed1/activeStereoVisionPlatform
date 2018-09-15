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

class DisparityClass{
public:
  DisparityClass(Size imageSize = Size(2048 , 1080)){
    imageSize = imageSize;
    BloackSize = 5;
    NoDisparity = 64;
    SpeckleWindowSize = 0;
    UniquenessRatio = 2;
    setTextureThreshold = 0;
    setMinDisparity = 64;
    setPreFilterCap = 62;
    setPreFilterSize = 51;
    lambda = 110; //5000.0;
    sigma = 15;
    prefilterType = 1;
    setSpeckleRange = 0;
    dispP1 = 100;
    dispP2 = 1000;
    FullDP = 1;
    setDisp12MaxDiff = 20;
    Disparity_color_scales = 2;
    disparityMode = 4;
    mode = (char)disparityMode;
    np_img = 0;
  }
  int disparityMode;

  void update_trackbar_reading_and_odds_numbers();
  // static void odd_number_callback(int value, void*);
  void saveImage(Mat undisFrame1, Mat undisFrame2,Mat canvas , Mat disp , Mat color_disparity);
  Mat ComputerDisparity(Mat left, Mat right);
  double calculate_projection_error(cv::Mat left, cv::Mat right, cv::Size boardSize, bool showimg);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPoinXYZ(Mat disp, Mat xyz ,Mat undisFrame1, float baselineSize = 0);
  void showColoredDisparity(cv::Mat disparity);

private:
  Size imageSize;
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
  int np_img;

};

//This function use in opencv to return odd numbers only
// static void DisparityClass::odd_number_callback(int value, void*)
// {
//     if(value >= 5){
//         // kernel size must be positive and odd
//         setPreFilterSize = (value % 2) ? value: value + 1;
//     }else{
//         setPreFilterSize = 5;
//     }
// }
/// This function to convert to cloud point
pcl::PointCloud<pcl::PointXYZRGB>::Ptr DisparityClass::MatToPoinXYZ(Mat disp, Mat xyz ,Mat undisFrame1, float baselineSize)
 {
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
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
               point.x = p.x + baselineSize;
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


void DisparityClass::update_trackbar_reading_and_odds_numbers(){

  namedWindow("trackbar");
  createTrackbar("Mode","trackbar",&disparityMode,7);
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
  createTrackbar("P2","trackbar",&dispP2, 1000);
  createTrackbar("lambda","trackbar",&lambda,10000);
  createTrackbar("sigma","trackbar",&sigma,1000);
  createTrackbar("prefilterType","trackbar",&prefilterType,1);
  createTrackbar("FullDP","trackbar",&FullDP,2);

  if (setPreFilterCap % 2 == 0)
  {
      setPreFilterCap = setPreFilterCap + 1;
  }
  if (setPreFilterSize % 2 == 0)
  {
      setPreFilterSize = setPreFilterSize + 1;
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

  if (disparityMode == 0){
    disparityMode = 1;
  }
  mode =  disparityMode + '0';
  // cout << "Mode : " << mode <<std::endl;
}


void DisparityClass::showColoredDisparity(cv::Mat disparity){
  Mat color_disparity;
  applyColorMap(disparity, color_disparity,Disparity_color_scales);
  cv::namedWindow("Color Disparity map", WINDOW_NORMAL);
  resizeWindow("Color Disparity map",1024,600);
  createTrackbar("Color Disparity","Color Disparity map",&Disparity_color_scales,12);
  imshow("Color Disparity map", color_disparity);
}
void DisparityClass::saveImage(Mat undisFrame1, Mat undisFrame2,Mat canvas , Mat disp , Mat color_disparity){
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
}
//this function use to calculate the diaprity
Mat DisparityClass::ComputerDisparity(Mat left, Mat right){
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
double DisparityClass::calculate_projection_error(cv::Mat left, cv::Mat right, cv::Size boardSize, bool showimg){
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
            //cout << "left: " << left_corners[i].y <<"\tTwo: "<< right_corners[i].y <<endl;
            double error= sqrt(pow((left_corners[i].y - right_corners[i].y),2));
            total_error = total_error + error;
        }
        //delta_time = ((double)cv::getTickCount() - delta_time)/cv::getTickFrequency();
        //cout<<"FPS: " << 1 / delta_time <<endl;
        return total_error/(boardSize.height * boardSize.width);
    }
    return 0;
}
