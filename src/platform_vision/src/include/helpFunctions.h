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

class GetImageClass{
public:
  GetImageClass(ros::NodeHandle nh, string ImageName){
    string imageNameTopic = "/stereo/"+ ImageName + "/image_raw"; // "/image_mono";
    img_sub = nh.subscribe(imageNameTopic,10, &GetImageClass::img_callback, this);
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