#include <iostream>
#include <string>
#include <tuple>

using namespace std;

cv::Mat drawCross(cv::Mat image, int lineSize = 30){
  cv::Mat copyImage = image.clone();
  line(copyImage, cv::Point((image.cols/2)-lineSize, image.rows/2), cv::Point((image.cols/2)+lineSize, image.rows/2), cv::Scalar(0,0,255), 2);  //crosshair horizontal
  line(copyImage, cv::Point(image.cols/2, (image.rows/2)-lineSize), cv::Point(image.cols/2, (image.rows/2)+lineSize), cv::Scalar(0,0,255), 2);  //crosshair vertical

  return copyImage;
}


cv::Mat convertROSMat2OpencvMat(const sensor_msgs::ImageConstPtr& img_msg){
  // create storage for the comming image in cv format
  cv_bridge::CvImagePtr cv_img_msg;

  //copy the image and save it in opencv formate
  try{
  cv_img_msg = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
  return cv_img_msg->image;
  }
  catch(cv_bridge::Exception e)
  {
      ROS_ERROR("cv_bridge expection: %s", e.what());
      return cv_img_msg->image;
  }
}


tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> ExtractRGBI(cv::Mat image){
  Mat channel[3];
  Mat I;
  cv::split(image, channel);

  cv::cvtColor(image, I, cv::COLOR_BGR2GRAY);

  return make_tuple(channel[0], channel[1], channel[2], I);

}

tuple<cv::Mat, cv::Mat> BGRColorOppenetProcess(Mat image){
    Mat R, G, B, I;
    tie(B, G, R, I)= ExtractRGBI(image);

    // Find the maximum in in the images
    Mat MaxBGR = cv::max(B,cv::max(R,G));
    Mat RGMin = cv::min(R,G);

    // find the oppenet R-G
    int thresholder = 20;
    Mat RG = (R-G);
    RG.setTo(0, RG < thresholder);
    RG.setTo(1, RG > thresholder);
    //Apply Mask
    Mat mask;
    // cv::bitwise_and(image,image,mask,RG);
    // cv::imshow("mask", mask);

    Mat BY = (B - RGMin);// / MaxBGR;
    // cv::imshow("RG", RG);
    return make_tuple(RG, BY);
}

class GetImageClass{
public:
  GetImageClass(ros::NodeHandle nh, string ImageName){
    string imageNameTopic = "/stereo/"+ ImageName + "/image_raw"; // "/image_mono";
    img_sub = nh.subscribe(imageNameTopic,10, &GetImageClass::img_callback, this);
  }

  GetImageClass(){

  }
  void img_callback(const sensor_msgs::ImageConstPtr& img_msg){
      image =  convertROSMat2OpencvMat(img_msg);
  }

  Mat getImage(){
    return image;
  }

private:
  ros::Subscriber img_sub;
  Mat image;
};


cv::Rect returnRectanguleSizeOfCenterImage(cv::Size imageSize, int windowSize){
  cv::Rect windowSizeRectangule;
  windowSizeRectangule.y = imageSize.height/2 - windowSize/2;
  windowSizeRectangule.x = imageSize.width/2 - windowSize/2;
  windowSizeRectangule.height = windowSize;
  windowSizeRectangule.width = windowSize;
  return windowSizeRectangule;
}

cv::Rect returnRectanguleSizeOfCenterImage2D(cv::Size imageSize, cv::Size windowSize){
  cv::Rect windowSizeRectangule;
  windowSizeRectangule.y = imageSize.height/2 - windowSize.height/2;
  windowSizeRectangule.x = imageSize.width/2 - windowSize.width/2;
  windowSizeRectangule.height = windowSize.height;
  windowSizeRectangule.width = windowSize.width;
  return windowSizeRectangule;
}

cv::Rect returnRectanguleTemplate(cv::Point2f center, int windowSize,cv::Size imageSize){
  int x1 = center.x - windowSize/2;
  int y1 = center.y - windowSize/2;
  int x2 = center.x + windowSize/2;
  int y2 = center.y + windowSize/2;

  if(x1 < 0){
    x1 = 0;
    x2 = windowSize;
  }
  if(x2 > imageSize.width){
    x2 = imageSize.width;
    x1 = imageSize.width - windowSize;
  }
  if(y1 < 0){
    y1 = 0;
    y2 = windowSize;
  }
  if(y2 > imageSize.height){
    y2 = imageSize.height;
    y1 = imageSize.height - windowSize;
  }

Rect2d bbox(x1, y1, windowSize, windowSize);
return bbox;
}
/////////////////////////////////////////////////////////////////////
/////These variables for selecting the regoin
bool mouseMove = false;
cv::Size imageSizes = cv::Size(2048 , 1080);
Point selectedPoint1 = Point(imageSizes.width/2 - 30, imageSizes.height/2 - 30);
Point selectedPoint2 = Point(imageSizes.width/2 + 30, imageSizes.height/2 + 30);
Mat Qvect;

//This function use to located a point when mouse click
Rect selectedRectangle;
void SelectROI(int event, int x, int y, int flags, void* data){
  if (event == EVENT_LBUTTONDOWN && mouseMove == false){
    int rectangle_size = 10;
    selectedPoint1 = Point(x - rectangle_size, y - rectangle_size);
    selectedPoint2 = Point(x + rectangle_size, y + rectangle_size);
    selectedRectangle = Rect(selectedPoint1, selectedPoint2);
    mouseMove = true;
  }else if (event == EVENT_LBUTTONDOWN && mouseMove == true){
    mouseMove = false;
  }


}

// This function to draw the box in the image
void DrawBox_around_selected_point(Mat *orginalImage){

    if (mouseMove == true){
        rectangle(*orginalImage, selectedRectangle, Scalar(0, 0, 255), 1, 8, 0);
    }
}



// Mat equalize_image_using_histograme(Mat img){
//   vector<Mat> channels;
//   Mat img_hist_equalized;
//
//   cvtColor(img, img_hist_equalized, CV_BGR2YCrCb); //change the color image from BGR to YCrCb format
//
//   split(img_hist_equalized,channels); //split the image into channels
//
//   equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)
//
//   merge(channels,img_hist_equalized); //merge 3 channels including the modified 1st channel into one image
//
//   cvtColor(img_hist_equalized, img_hist_equalized, CV_YCrCb2BGR); //change the color image from YCrCb to BGR format (to display image properly)
//
//   return img_hist_equalized;
//
// }
