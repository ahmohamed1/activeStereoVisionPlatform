#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include<std_msgs/Float64.h>
#include <std_msgs/Bool.h>

using namespace cv;
using namespace std;

// global virable
Mat left_img, right_img;
 Size sizee = Size(900,600);
float left_pan = 0,right_pan = 0;
// These virables are the value form the offline calibration
float angle_x = 0.0;
float angle_z = 0.0;
float trans_y = 0.0;
float trans_z = 0.0;

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

 float old_left_pan = 0,old_right_pan = 0;
void left_pan_callback(const std_msgs::Float64& val){

  //if(val.data != old_left_pan+0.8){
  left_pan = val.data;
  //old_left_pan = left_pan;
  //}

}

void right_pan_callback(const std_msgs::Float64& val){

  //if(val.data != old_right_pan+0.8){
  right_pan = val.data;
 // old_right_pan = right_pan;
  //}
}


int main(int argc,char** argv)
{

  //calculate the rotating matrix for x and z angles
  Mat x_mat = (Mat_<float>(3,3) << 1,	0,	    0,
				   0,cos(angle_x),-sin(angle_x),
				   0, sin(angle_x) , cos(angle_x));

  Mat z_mat = (Mat_<float>(3,3) << cos(angle_z),-sin(angle_z),0,
				   sin(angle_z),cos(angle_z), 0,
				   0,	 0 ,		0);

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


  // ask whic file to read and load the result
  /*cout << "What is the name of the file you want to load??" << endl;
  string fileName;
  cin >> fileName;
  cout << "Reading File...." << endl;
  string filename = "/home/abdulla/dev/workshop/PhD/StereoCalibrationTest/calibrating_data/" +fileName + ".xml";*/
  string filename = "/home/abdulla/dev/workshop/PhD/StereoCalibrationTest/calibrating_data/001_run/001_test.xml";
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

  // print them out
/*  cout << "interinsic1" << A1 << endl;
  cout << "\ninterinsic2" << A2 << endl;
  cout << "\ndistCoeffs1" << D1 << endl;
  cout << "\ndistCoeffs2" << D2 << endl;
  cout << "\nR" << R << endl;
  cout << "\nT" << T << endl;
  cout << "\nE" << E << endl;
  cout << "\nF" << F << endl;*/
  //////////////////////////////////////
    ros::init(argc,argv,"object_tracking");
    ros::NodeHandle nh;

    //define the subscriber and publisher
    ros::Subscriber left_img_sub = nh.subscribe("/stereo/left/image_raw",10,left_img_callback);
    ros::Subscriber right_img_sub = nh.subscribe("/stereo/right/image_raw",10,right_img_callback);
    // define the angle subscriper
    ros::Subscriber right_pan_sub = nh.subscribe("/right/pan/angle",10,right_pan_callback);
    ros::Subscriber left_pan_sub = nh.subscribe("/left/pan/angle",10,left_pan_callback);

    float old_sum= 0;
    bool angle_chang = true;
    Mat mapx1, mapy1, mapx2, mapy2;
    int np_img =1;
    while(nh.ok()){
	ros::spinOnce();
	char ikey;
	ikey = cv::waitKey('q');
	if(!left_img.empty() && !right_img.empty()){

	    // resize the images to sut with the calibration data
	    resize(left_img,left_img,sizee);
	    resize(right_img,right_img,sizee);
	    // cheack if the total difference bwtween the left angle and the right angle has change
	    float angle_sum = (left_pan) + (right_pan);
	    angle_sum = angle_sum;
	    cout<<"angle sum: " << angle_sum << endl;


	     // convert angle to radian
	     float angle = angle_sum * 0.0174533;

	     Mat y_mat = (Mat_<float>(3,3) << cos(angle),0,sin(angle),
					      0,	1,	   0,
					      -sin(angle), 0 , cos(angle));

	     // calculate the rotating matrix
	     Mat Rnew = z_mat * y_mat * x_mat;
	     //cout<<Rnew <<endl;

	     // add the new rotating matrix to the old
	     Rnew.convertTo(Rnew,CV_64FC1);
	     R.convertTo(R,CV_64FC1);
	     R = R * Rnew;
	     // update the old sum in update the angle change to true to redo the rectification
	     angle_chang = true;

	    if(angle_chang == true){

		// calculate the the new projection matrix using bouguet algorithm
		stereoRectify(A1, D1,
		    A2, D2,
		    sizee,
		    R, T,
		    R1, R2,
		    P1, P2,
		    Q,
		    CALIB_ZERO_DISPARITY);  // to allowed zero disparity


		// initialize undistortion
		// Define the map in x and y direction

		// initialize the undistortion rectify map for each camera
		initUndistortRectifyMap(A1, D1, R1,
					P1, sizee, CV_32FC1, mapx1, mapy1);

		initUndistortRectifyMap(A2, D2, R2,
					P2, sizee, CV_32FC1, mapx2, mapy2);

		//return angle change to false so to not do rectification process if the angle doen't change
		angle_chang = false;
	}

	    // remap the image and present them
	    Mat undisFrame1,undisFrame2;
	    remap(left_img, undisFrame1, mapx1, mapy1, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	    remap(right_img, undisFrame2, mapx2, mapy2, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	    // combine both images in one image and draw lines
	    Mat combine_img;
	    hconcat(undisFrame1,undisFrame2,combine_img);
	    //draw lines
	    for(int i=0; i< 20 ; i++){
	      line(combine_img,Point(0,(i*combine_img.rows/20)),Point(combine_img.cols,(i*combine_img.rows/20)),Scalar(0,0,255),1);
	      }

      namedWindow("rectified image",WINDOW_AUTOSIZE);
	    imshow("rectified image",combine_img);
	    //cout << " To take another picture press c, to change the angle a, and q to exit the program: " << endl;

	    char ikey = waitKey('q');
	    if(ikey == 'q'){
		      break;
	      }else if(ikey == 's'){  // save the images
		stringstream ssl;
		ssl<<"/home/abdulla/dev/workshop/PhD/Trctification_based_on_bouguet/001_image_test_2/"<<np_img << "_left" <<".jpg";
		string left_name = ssl.str();
		ssl.str("");

		stringstream ssr;
		ssr<<"/home/abdulla/dev/workshop/PhD/Trctification_based_on_bouguet/001_image_test_2/"<<np_img << "_right" <<".jpg";
		string right_name = ssr.str();
		ssr.str("");

		stringstream ssb;
		ssb<<"/home/abdulla/dev/workshop/PhD/Trctification_based_on_bouguet/001_image_test_2/" <<np_img << "_both_img" << ".jpg";
		string both_name = ssb.str();
		ssb.str("");

		imwrite(left_name,undisFrame1);
		imwrite(right_name,undisFrame2);
		imwrite(both_name,combine_img);
		cout<< "image save : "<<np_img<<endl;
		np_img++;
	      }
	}// end of the if statment of the images
    }


return 1;


}
