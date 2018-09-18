#include <geometry_msgs/Pose2D.h>

class ThresholdPointCloud{
public:
  ThresholdPointCloud(ros::NodeHandle nh){
    templateSizeSubscriber = nh.subscribe("/templateSize" ,1, &ThresholdPointCloud::templateSize_callback, this);
    templateSize2D = cv::Size(100, 100);
  }


  void templateSize_callback(const geometry_msgs::Pose2D &tempSize){
    templateSize2D = cv::Size(tempSize.x, tempSize.y);
  }

  cv::Mat returnMaskForTemplate(cv::Mat inputImage){

    Mat mask(inputImage.rows, inputImage.cols, CV_8UC1, Scalar(0));
    //Draw rectangle white in the center
    int x1 = inputImage.cols/2 - templateSize2D.width/2;
    int y1 = inputImage.rows/2 - templateSize2D.height/2;
    int x2 = inputImage.cols/2 + templateSize2D.width/2;
    int y2 = inputImage.rows/2 + templateSize2D.height/2;

    cv::rectangle(mask, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(1), -1);
    return mask;

  }

private:
  ros::Subscriber templateSizeSubscriber;
  cv::Size templateSize2D;
};
