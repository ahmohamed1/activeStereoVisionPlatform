class VergenceClass{
public:
  VergenceClass(ros::NodeHandle nh, int _windowSize = 200){
    FastTemplateMatch vergFastMatchTemplate;
    windowsNameString = "SlaveCamera";
    windowSize = _windowSize;
    //define the subscriber and publisher
    rightImageSubClass = GetImageClass(nh, "right");
    leftImageSubClass = GetImageClass(nh, "left");
    imageSize = Size(2048 , 1080);
    windowSizeRectangule = returnRectanguleSizeOfCenterImage(imageSize,windowSize);
    //Define the publisher
    motorController = MotorController(nh, "right");
    motorController.moveToZero();

    namedWindow(windowsNameString, WINDOW_NORMAL);
    resizeWindow(windowsNameString,640,480);
    cv::moveWindow(windowsNameString, 1000,600);

    // r(15); // 10 hz

  }

  void mainLoop(){

    // start the while loop
      // while(true){
          ros::spinOnce();
          char ikey;
          ikey = cv::waitKey(1);
          right_img = rightImageSubClass.getImage();
          left_img = leftImageSubClass.getImage();
          if(!left_img.empty() && !right_img.empty()){

            cv::Mat temp = left_img(windowSizeRectangule);
            cv::Point2f difference;
            cv::Mat editedImage;
            // cout<<left_img.size() <<endl;
            tie(editedImage, difference) = vergFastMatchTemplate.trackTargetPNCC(right_img, temp);
            imshow(windowsNameString, editedImage);
            // std::cout<< "Difference: " << difference << endl;
            // calculate the integral
            bool panState = motorController.movePanMotor(difference.x); // 0 to move the pan motor
            bool tiltState = motorController.moveTiltMotor(difference.y); // 1 to move the tilt motor
            motorController.checkVergeCorrectely(panState,tiltState);
            //Step 7 - this step use to select the right parameters of the PID
            char ikey = waitKey(1);

            if(ikey == 'q'){
                motorController.moveToZero();
                // break;
            }
        }// end of the if statment of the images
        // r.sleep();
    // }// End of while loop
  }

private:
  FastTemplateMatch vergFastMatchTemplate;
  string windowsNameString;
  int windowSize;
  //define the subscriber and publisher
  GetImageClass rightImageSubClass;
  GetImageClass leftImageSubClass;
  cv::Rect windowSizeRectangule;
  //Define the publisher
  MotorController motorController;
  cv::Mat left_img, right_img;
  cv::Size imageSize ;
};
