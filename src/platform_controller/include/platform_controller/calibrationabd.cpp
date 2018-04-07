#include<opencv2/opencv.hpp>
#include<iostream>
#include"calibrationabd.h"

using namespace cv;
using namespace std;


    calibrationabd::calibrationabd(){
        // ask the user of the input
        cout << "Enter the number of board you need for the calibration: " << endl;
        cin >> numBoard;
        /*cout << "Enter number of corner in Horezintal: ";
        cin >> numCornHer;
        cout << "Enter number of conrenr in vertical: ";
        cin >> numCornVer;
        cout << "Enter distance between corners in Horesantal: ";
        cin >> HerDist;*/
        numCornHer = 8 ;
        numCornVer = 6;
        HerDist = 34.5;

        VerDist = HerDist;
        //cout << "Enter distance between corners in Vertical: ";
        //cin >> VerDist;
        //evaluate the input data for future work
        numSquare = numCornHer*numCornVer;
        boardSize = Size(numCornHer, numCornVer);

        interinsic2 = Mat(3, 3, CV_64FC1);
        interinsic1 = Mat(3, 3, CV_64FC1);
        sucsses = 0;
        // This storage for the actual coordinate refrence we can use the dimension
        // as (0,0,0),(0,1,0),(0,2,0),.....(1,4,0),....
        for (int j = 0; j < numSquare; j++){
            obj.push_back(Point3f((j / numCornHer) * HerDist, (j%numCornHer) * VerDist, 0.0f));		// we use a unit because we are ni=ot the concern to dimension
        } // End for loop

    }//end of construction

    calibrateAbd::~calibrateAbd(){
        cout << "calibration was done good bye.." << endl;
    }

    //This function work to find the corner and ster them
    void calibrateAbd::FindCornersAndStore(VideoCapture vid, VideoCapture vid2,bool NumberOfCam){
        bool bSucssCornerFind = true;
        int sucsses = 0;	// How many sucess we have in finding the corner
        while (bSucssCornerFind){
            if (NumberOfCam == true){
                //Capture frames
                bool bSucsCapture = vid.read(frame);
                bool bSucsCapture2 = vid2.read(frame2);
                if (bSucsCapture2 == false){
                    cout << "Error unable to capture frame 2 ...\n";
                    char Ikey = 'c';
                    while (Ikey != 'q'){
                        Ikey = waitKey('q');
                    }
                    //return -1;
                }
                if (bSucsCapture == false){
                    cout << "Error unable to capture frame 1...\n";
                    char Ikey = 'c';
                    while (Ikey != 'q'){
                        Ikey = waitKey('q');
                    }
                    //return -1;
                }
            }
            else{
                SliptVideo(vid, &frame, &frame2);
            }
            // This function to look for corner
            bool BothGood = false;
            //Convert Frame to gray
            cvtColor(frame, grayImg, CV_BGR2GRAY);
            cvtColor(frame2, grayImg2, CV_BGR2GRAY);
            //findChessboardCorners(grayImg,patternSize,corners,cornerCount,CV_CALIB_CB_FAST_CHECK);
            bool patternWasFound = findChessboardCorners(grayImg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
            bool patternWasFound2 = findChessboardCorners(grayImg2, boardSize, corners2, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
            if (patternWasFound && patternWasFound2){
                cornerSubPix(grayImg, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
                cornerSubPix(grayImg2, corners2, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
                BothGood = true;	//if both frame find corner make it true
            }
            else{
                BothGood = false;	//if not make it false
            }
            drawChessboardCorners(frame, boardSize, corners, patternWasFound);
            drawChessboardCorners(frame2, boardSize, corners2, patternWasFound2);

            Mat frame_small,frame2_small;
           // resize(frame,frame_small,Size(900,600));
           // resize(frame2,frame2_small,Size(900,600));

            imshow("Left Window", frame);
            imshow("Right Window", frame2);
            char iKey = waitKey('s');
            if (iKey == 's' && BothGood == true){	// if the corner found and the user press s it will store the corner data in the imagePoint

                imagePoint.push_back(corners);			// Store corners values in the image point for calibration
                objectPoints.push_back(obj);			// Stroe the phisical position in the object point for calibration

                imagePoint2.push_back(corners2);			// Store corners values in the image point for calibration
                objectPoints2.push_back(obj);			// Stroe the phisical position in the object point for calibration

                cout << "Corners Stored suceessfully.." << sucsses + 1 << endl;
                sucsses++;

                if (sucsses >= numBoard){
                    bSucssCornerFind = false;
                }
            }	//End if statment

            if (iKey == 'q'){
                break;
            }
        }	//End the while loop

        //destroyAllWindows();
    }


    //This function work to find the corner and ster them

    void calibrationabd::FlyCaptureCamFindCornersAndStore(Mat frame, Mat frame2, int *CornerFound){

            // This function to look for corner
            bool BothGood = false;
            //Convert Frame to gray
            cvtColor(frame, grayImg, CV_BGR2GRAY);
            cvtColor(frame2, grayImg2, CV_BGR2GRAY);
            //findChessboardCorners(grayImg,patternSize,corners,cornerCount,CV_CALIB_CB_FAST_CHECK);
            bool patternWasFound = findChessboardCorners(grayImg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
            bool patternWasFound2 = findChessboardCorners(grayImg2, boardSize, corners2, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS);
            if (patternWasFound && patternWasFound2){
                cornerSubPix(grayImg, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
                cornerSubPix(grayImg2, corners2, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS, 30, 0.1));
                BothGood = true;	//if both frame find corner make it true
            }
            else{
                BothGood = false;	//if not make it false
            }
            drawChessboardCorners(frame, boardSize, corners, patternWasFound);
            drawChessboardCorners(frame2, boardSize, corners2, patternWasFound2);

            imshow("Left Window", frame);
            imshow("Right Window", frame2);
            char iKey = waitKey('s');
            if (iKey == 's' && BothGood == true){	// if the corner found and the user press s it will store the corner data in the imagePoint

                imagePoint.push_back(corners);			// Store corners values in the image point for calibration
                objectPoints.push_back(obj);			// Stroe the phisical position in the object point for calibration

                imagePoint2.push_back(corners2);			// Store corners values in the image point for calibration
                objectPoints2.push_back(obj);			// Stroe the phisical position in the object point for calibration

                cout << "Corners Stored suceessfully.." << sucsses + 1 << endl;
                sucsses++;
                if (sucsses >= numBoard){
                    cout << "ok" << endl;
                    *CornerFound = 0;
                }
            }	//End if statment
            FrameSize = frame.size();
        //destroyAllWindows();
    }


    void calibrationabd::calibrateThecamera(){
        cout << "Start calibration...." << endl;

       // calibrate camera 1
        cv::calibrateCamera(objectPoints,
                            imagePoint,
                            FrameSize,
                            interinsic1,
                            distCoeffs1,
                            Rvect1,
                            Tvect1);

        // calibrate camera 2
         cv::calibrateCamera(objectPoints,
                             imagePoint2,
                             FrameSize,
                             interinsic2,
                             distCoeffs2,
                             Rvect2,
                             Tvect2);

        //Calibrate stereo camera
        stereoCalibrate(objectPoints,
            imagePoint, imagePoint2,
            interinsic1, distCoeffs1,
            interinsic2, distCoeffs2,
            FrameSize,
            Rvect,
            Tvect,
            Qvect,
            Fvect);/*
            cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
            CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);*/
        cout << "Calibration was done successfully..\n" << endl;

        cout << "interinsic1" << interinsic1 << endl;
        cout << "\ninterinsic2" << interinsic2 << endl;
        cout << "\ndistCoeffs1" << distCoeffs1 << endl;
        cout << "\ndistCoeffs2" << distCoeffs2 << endl;
        cout << "\nR" << Rvect << endl;
        cout << "\nT" << Tvect << endl;
        cout << "\nE" << Evect << endl;
        cout << "\nF" << Fvect << endl;

        SaveTheResult();
        //	cout << "THe parameters was saved" << endl;
    }

    void calibrationabd::SaveTheResult(){
        cout << "What is the name of the file you want to save??" << endl;
        string fileName;
        cin >> fileName;

        string filename = "/home/abdulla/dev/workshop/qtProject/StereoCalibrationTest/calibrating_data/" + fileName + ".xml";
        cout<<"The file name will save as : "<<filename<<endl;
        FileStorage fs(filename, FileStorage::WRITE);
        fs << "interinsic1" << interinsic1;
        fs << "interinsic2" << interinsic2;
        fs << "distCoeffs1" << distCoeffs1;
        fs << "distCoeffs2" << distCoeffs2;
        fs << "R" << Rvect;
        fs << "T" << Tvect;
        fs << "E" << Evect;
        fs << "F" << Fvect;

        cout << "\nCalibration parameters saved..." << endl;
        fs.release();
    }

    void calibrationabd::loadTheParameters(){
        string filename = "CameraParameter.xml";
        FileStorage fr(filename, FileStorage::READ);
        fr["interinsic1"] >> interinsic1;
        fr["interinsic2"] >> interinsic2;
        fr["distCoeffs1"] >> distCoeffs1;
        fr["distCoeffs2"] >> distCoeffs2;
        fr["R"] >> Rvect;
        fr["T"] >> Tvect;
        fr["E"] >> Evect;
        fr["F"] >> Fvect;
        fr.release();
    }



    void calibrationabd::SliptVideo(VideoCapture vid, Mat *frame1, Mat *frame2){
        Mat Main;
        bool bSucsCapture = vid.read(Main);
        if (bSucsCapture == false){
            cout << "Error unable to capture frame 2 ...\n";
            char Ikey = 'c';
            while (Ikey != 'q'){
                Ikey = waitKey('q');
            }//end of while loop
            //return -1;
        }//end of if stamnent

        //take the ROI from the image
        Size imgSize = Main.size();
     /*   int ROISize = 50;		//this will be the size times 2
        int ROICenter = imgSize.width / 2;
        Point CenterOfImage = Point(imgSize.height, imgSize.width / 2);*/
        Rect ROIBox1 = Rect(Point(0, 0), Point(imgSize.height, imgSize.width / 2));
        Rect ROIBox2 = Rect(Point(0, imgSize.width / 2), Point(imgSize.height, imgSize.width));
        *frame1 = Main(ROIBox1);
        *frame2 = Main(ROIBox2);
        //imshow("ROI", ROIimg);

    }
