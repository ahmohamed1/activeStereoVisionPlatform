#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>


//import dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <platform_vision/pidConfig.h>

using namespace cv;
using namespace std;


cv::Mat img;


////////////////////////////////////////////////////////////////



int win_nam = 1;

string window_name = "image_";

void img_callback(const sensor_msgs::ImageConstPtr& img_msg){

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


    img = cv_img_msg->image;

}

// This function used to recived the ide number for tracking
int Ide_number;
void ide_num_callback(const std_msgs::Int32& val){

    Ide_number = val.data;
}

Point2f track(Mat *inputImage, bool *found);
///
geometry_msgs::Vector3 difference, object_pos;
std_msgs::Bool motor_mode;
std::string global_name;

// PID parameters
double int_error = 0;
double last_rime = 0, current_time = 0;
double Kp=0.2, Ki=0, Kd=0;
double dt = 100e-3;
double diff = 0;
double last_error = 0;

// this function use to call the dynamic parameters
void callback(platform_vision::pidConfig &config, uint32_t level) {

    //ROS_INFO("Reconfigure Request: %f %f %f",config.p, config.i, config.d);
    // Kp = config.Kp;
    // Ki = config.Ki;
    // Kd = config.Kd;

}



/// Start the main function
int main(int argc,char** argv)
{
    Ide_number = 1;
    ros::init(argc,argv,"marker_tracking");
    ros::NodeHandle nh;
    nh.setParam("global_param", 5);
    //define the subscriber and publisher
    ros::Subscriber img_sub = nh.subscribe("/stereo/right/image_raw",5,img_callback);
    ros::Subscriber ide_num_sub = nh.subscribe("/ide_number",10,ide_num_callback);
    ros::Publisher angle_pub = nh.advertise<geometry_msgs::Vector3>("/left/pan/move",5);

    dynamic_reconfigure::Server<platform_vision::pidConfig> server;
    dynamic_reconfigure::Server<platform_vision::pidConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    if(nh.getParam("global_name", global_name)){

    ROS_INFO_STREAM("window parameter from param" << global_name);

    window_name = "image_" + global_name;
    }

    namedWindow(window_name,WINDOW_NORMAL);
    resizeWindow(window_name,680,420);
    //set the motor mode to wheel mode
    difference.x = 0.0;
    difference.z = 2.0;
    bool found = false; // this virable use to move the motor if there is object and stop if there is no object


    while(nh.ok()){

        ros::spinOnce();
        char ikey;
        ikey = cv::waitKey('q');
        if(!img.empty()){
           Mat img_proc;
           //resize(img,img_proc, Size(900,600));
           img_proc = img;
           Point CenterPt = track(&img_proc, &found);

          //calculate the differences
           diff = (img_proc.cols/2) - CenterPt.x;

           // calculate the integral
           int_error += diff * dt;
           double error = (diff * Kp) + (int_error * Ki) + (Kd * (diff - last_error));
           last_error = diff;

           if(std::abs(diff) > 10 && found == true){
               difference.y = error;
           }else
           {
               difference.y = 0;
           }
           angle_pub.publish(difference);

           stringstream ss;
           ss << difference.y;
           string str = ss.str();

           putText(img_proc,str,Point(100,100),4,2.5,Scalar(0,255,0));
           imshow(window_name,img_proc);

            if(ikey =='q'){
                // when exit the loop send a command to stop the motor from rotating
                difference.x = 0.0;
                difference.y = 0.0;
                difference.z = 0.0;
                angle_pub.publish(difference);
                destroyAllWindows();
                return 1;
            }

        }
    }

return 1;

}


Point2f track(Mat *inputImage,bool *found) {

vector< int > markerIds;
vector< vector<Point2f> > markerCorners;
cv::Ptr<aruco::Dictionary> dictionary=aruco::getPredefinedDictionary(1);
cv::aruco::detectMarkers(*inputImage, dictionary, markerCorners, markerIds);

Point2f center;
// if at least one marker detected
   if (markerIds.size() > 0){


       // The below statment desided which number to track
       for(int i = 0; i < markerIds.size();i++){
           if( markerIds[i] == Ide_number){
               cv::aruco::drawDetectedMarkers(*inputImage, markerCorners, markerIds);
               center.x = (markerCorners[i][0].x + markerCorners[i][1].x + markerCorners[i][2].x+ markerCorners[i][3].x)/4;
               center.y = (markerCorners[i][0].y + markerCorners[i][1].y + markerCorners[i][2].y+ markerCorners[i][3].y)/4;
               circle(*inputImage,center,5,Scalar(0,0,255), 2, 8, 0);
               // change it to true so the motor moves
                *found = true;
           }
       }

   }else{
       *found = false;
   }
   return center;
}
