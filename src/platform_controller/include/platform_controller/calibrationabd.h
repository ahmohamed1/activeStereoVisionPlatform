#ifndef CALIBRATIONABD_H
#define CALIBRATIONABD_H

#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;


class calibrationabd{
public:
    calibrationabd();

    ~calibrationabd();

    //This function work to find the corner and ster them
    void FindCornersAndStore(VideoCapture vid, VideoCapture vid2,bool NumberOfCam);

    //This function work to find the corner and ster them

    void FlyCaptureCamFindCornersAndStore(Mat frame, Mat frame2, int *CornerFound);


    void calibrateThecamera();

    void SaveTheResult();

    void loadTheParameters();

private:

    void SliptVideo(VideoCapture vid, Mat *frame1, Mat *frame2);

    // Define the input virable from the user
    int numBoard;
    int numCornHer;
    int numCornVer;

    int numSquare;		// From the input data
    Size boardSize;		// From the input data
    float HerDist, VerDist;		//To store the dimensions of the checker board

    // Create a storage for point from the images for calibration
    vector<vector<Point3f> > objectPoints;			// This is the phiscal point on the board
    vector<vector<Point2f> > imagePoint;				// This to store the point in the image plane location of corner
    vector<vector<Point3f> > objectPoints2;			// This is the phiscal point on the board
    vector<vector<Point2f> > imagePoint2;			// This to store the point in the image plane location of corner
    vector<Point3f> obj;
    // This storage for finding corner
    vector<Point2f> corners;						// to hold the corner position
    vector<Point2f> corners2;						// to hold the corner position
    int sucsses;	// How many sucess we have in finding the corner

    Mat grayImg;		// to store the gray image for process
    Mat grayImg2;		// to store the gray image for process

    Mat frame;
    Mat frame2;
    Size FrameSize;
    // These storage for the intrinsic and distortion coffeicent
    Mat Rvect;
    Mat Tvect;
    Mat Evect;
    Mat Fvect, Pvect1, Pvect2, Qvect;
    //For camera 2
    Mat interinsic2; //Mat(3, 3, CV_64FC1);
    Mat distCoeffs2;
    Mat Rvect2;
    Mat Tvect2;
    // For camera 1
    Mat interinsic1; //Mat(3, 3, CV_64FC1);
    Mat distCoeffs1;
    Mat Rvect1;
    Mat Tvect1;
};
#endif  CALIBRATIONABD_H
