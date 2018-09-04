#include<std_msgs/Bool.h>

class MotorController{
public:
  MotorController(){

  }
  MotorController(ros::NodeHandle nh, string whichMotorToMove, bool _Debug=false, bool _tune = false){
    Debug = _Debug;
    std::string PanMotorTopic = "/" + whichMotorToMove + "/pan/move";
    std::string TiltMotorTopic = "/" + whichMotorToMove + "/tilt/move";
    motorPanPublisher = nh.advertise<std_msgs::Float64>(PanMotorTopic,10);
    motorTiltPublisher = nh.advertise<std_msgs::Float64>(TiltMotorTopic,10);
    VergeConpletePublisher =  nh.advertise<std_msgs::Bool>("/"+ whichMotorToMove+ "/onTarget", 1);
    currentPosition[0] = 0.0;
    currentPosition[1] = 0.0;

    exponatialGain[0] = 0.004;
    exponatialGain[1]= 0.0045; // pan, tilt
    mapExponatialValue[0] = 0.09; //0.2;
    mapExponatialValue[1]= 0.1; //0.35;
    thresholdMotorController[0] = 80;
    thresholdMotorController[1] = 12;
    motorMaxLimit = 75;
    motorMinLimit = -75;
    motorMinLimitTilt = -37;
    motorMaxLimitTilt = 37;
    motorState = false;
    countVerge = 0;

    DigitsController = 0;
    // digits[0] = {0.01,0.1,1,10,100,1000};
    digits[0] = 0.01;
    digits[1] = 0.1;
    digits[2] = 1.0;
    digits[3] = 10.0;
    digits[4] = 100.0;
    digits[5] = 1000.0;
    tune = _tune;
    if(tune){
      ROS_INFO("Tuning is activate!!");
      controllerName = whichMotorToMove +" controller";
      cv::namedWindow(controllerName);
      exponatialGainTuner[0] = exponatialGain[0];
      exponatialGainTuner[1] = exponatialGain[1];
      mapExponatialValueTuner[0] = mapExponatialValue[0];
      mapExponatialValueTuner[1] = mapExponatialValue[1];
    }

  }

  void tuneParameter(){
    if(tune){
      cv::createTrackbar("Digits", controllerName, &DigitsController, 6);
      cv::createTrackbar("exponatialGainPan", controllerName, &exponatialGainTuner[0], 100);
      cv::createTrackbar("exponatialGainTilt", controllerName, &exponatialGainTuner[1], 100);
      cv::createTrackbar("mapPan", controllerName, &mapExponatialValueTuner[0], 100);
      cv::createTrackbar("mapTilt", controllerName, &mapExponatialValueTuner[1], 100);

      setExponatialGrain(exponatialGainTuner[0]*digits[DigitsController], exponatialGainTuner[1]*digits[DigitsController]);
      SetMapExponatialValue(mapExponatialValueTuner[0]*digits[DigitsController], mapExponatialValueTuner[1]*digits[DigitsController]);
    }
  }
  void setExponatialGrain(float pan, float tilt){
    exponatialGain[0] = pan;
    exponatialGain[1]= tilt; // pan, tilt
    cout<< "ExponatialGrain: " << pan << " " << tilt<< endl;
  }

  void SetMapExponatialValue(float pan, float tilt){
    mapExponatialValue[0] = pan;
    mapExponatialValue[1]= tilt; // pan, tilt
    cout<< "mapExponatialValue: " << pan << " " << tilt<< endl;
  }

  void moveToZero(){

    for(int j = 0; j< 5; j++){
      ros::spinOnce();
      for(int i =0; i < 2;i++){
        currentPosition[i] = 0.0;
        motorPos[i].data = currentPosition[i];
      }
      motorPanPublisher.publish(motorPos[0]);
      motorTiltPublisher.publish(motorPos[0]);
    }
  }
  int signnum(float x) {
    if (x > 0.0) return 1;
    if (x < 0.0) return -1;
    return 0;
  }

  void moveMotorWithoutExponanetial(float value){
    float speed = value;
    currentPosition[0] += speed;
    motorPos[0].data = currentPosition[0];
    if (Debug){
      std::cout << "Motor Pan Speed: " << currentPosition[0] << std::endl;
    }
    if (currentPosition[0] < motorMaxLimit and currentPosition[0] > motorMinLimit){
      motorPanPublisher.publish(motorPos[0]);
    }
  }


  bool movePanMotor(float value){
    tuneParameter();

    bool state = false;
    value = -value;
    float speed = 0;
    speed = signnum(-value) * exp(abs(value)* exponatialGain[0])*mapExponatialValue[0];
    if (abs(value) > thresholdMotorController[0]){
      currentPosition[0] += speed;
      motorPos[0].data = currentPosition[0];
      if (Debug){
        std::cout << "Motor Pan Speed: " << currentPosition[0] << std::endl;
      }

      if (currentPosition[0] < motorMaxLimit and currentPosition[0] > motorMinLimit){
        motorPanPublisher.publish(motorPos[0]);
      }

    }else if (abs(value) <  thresholdMotorController[0] && abs(value) >  thresholdMotorController[1]){
      currentPosition[0] -= value * 0.001;
      motorPos[0].data = currentPosition[0];
      if (Debug){
        std::cout << "Motor Pan Speed: " << currentPosition[0] << std::endl;
      }
      if (currentPosition[0] < motorMaxLimit and currentPosition[0] > motorMinLimit){
        motorPanPublisher.publish(motorPos[0]);
      }
    }else{
      if (Debug){
        std::cout << "Motor Pan Center " << std::endl;
      }
      state = true;
    }
    return state;
  }

  bool moveTiltMotor(float value){
    bool state = false;
    value = -value;
    float speed = 0;
    speed = signnum(-value) * exp(abs(value)* exponatialGain[1])*mapExponatialValue[1];
    if (abs(value) > thresholdMotorController[1]){
      currentPosition[1] += speed;
      motorPos[1].data = currentPosition[1];
      if (Debug){
        std::cout << "Motor Tilt Speed: " << currentPosition[1] << std::endl;
      }
      if (currentPosition[1] < motorMaxLimit and currentPosition[1] > motorMinLimit){
        motorTiltPublisher.publish(motorPos[1]);
      }

    }else if (abs(value) <  thresholdMotorController[1] && abs(value) >  thresholdMotorController[1]){
      currentPosition[1] -= value * 0.001;
      motorPos[1].data = currentPosition[1];
      if (Debug){
        std::cout << "Motor Tile Speed: " << currentPosition[1] << std::endl;
      }
      if (currentPosition[1] < motorMaxLimit and currentPosition[1] > motorMinLimit){
        motorTiltPublisher.publish(motorPos[1]);
      }
    }else{
      if (Debug){
        std::cout << "Motor Tilte Center " << std::endl;
      }
      state = true;
    }
    return state;
  }

  bool checkVergeCorrectely(bool panState, bool tiltState){
      if (panState && tiltState){
        countVerge ++;
        if(countVerge == 50){
          // Publish True
          std_msgs::Bool msg;
          msg.data = true;
          VergeConpletePublisher.publish(msg);
          countVerge = 0;
          return 1;
        }
      }
      else{
        std_msgs::Bool msg;
        msg.data = false;
        VergeConpletePublisher.publish(msg);
        return 0;
      }
  }


  void moveMotor(float value, int MotorID){
    float speed = 0;
    speed = signnum(-value) * exp(abs(value)* exponatialGain[MotorID])*mapExponatialValue[MotorID];
    if (abs(value) > thresholdMotorController[MotorID]){
      currentPosition[MotorID] += speed;
      motorPos[MotorID].data = currentPosition[MotorID];
      // std::cout << "Motor Tilt Speed: " << currentPosition[MotorID] << std::endl;
      if (currentPosition[MotorID] < motorMaxLimit and currentPosition[MotorID] > motorMinLimit){
        whichMotorToUse(MotorID);
      }

    }else if (abs(value) <  thresholdMotorController[MotorID] && abs(value) >  thresholdMotorController[MotorID]){
      currentPosition[MotorID] -= value * 0.001;
      motorPos[MotorID].data = currentPosition[MotorID];
      // std::cout << "Motor Tile Speed: " << currentPosition[MotorID] << std::endl;
      if (currentPosition[MotorID] < motorMaxLimit and currentPosition[MotorID] > motorMinLimit){
        whichMotorToUse(MotorID);
      }
    }else{
      std::cout << "Motor Tilte Center " << std::endl;
    }
  }

cv::Point2f converteToImageCoordinate(Size imageSize, cv::Point2f position){
  cv::Point2f differences = cv::Point(0.0,0.0);
  if(position.x != 0 & position.y != 0){
    differences.x = position.x - imageSize.width/2;
    differences.y = position.y - imageSize.height/2;
  }

  return differences;

}

void tiltGoto(float newPose){
  currentPosition[1] = newPose;
  motorPos[1].data = currentPosition[1];
}

void whichMotorToUse(int MotorID){
  if ( MotorID == 0){
    motorPanPublisher.publish(motorPos[MotorID]);
  }else if(MotorID == 1){
    motorTiltPublisher.publish(motorPos[MotorID]);
  }
}
private:
  ros::Publisher motorPanPublisher, motorTiltPublisher, VergeConpletePublisher;
  float currentPosition[2];
  float exponatialGain [2];
  float mapExponatialValue [2];
  float thresholdMotorController[2];
  float motorMaxLimit;
  float motorMinLimit;
  float motorMinLimitTilt;
  float motorMaxLimitTilt;
  std_msgs::Float64 motorPos[2];
  bool motorState;
  int countVerge;
  bool Debug;
  bool tune;
  int exponatialGainTuner [2];
  int mapExponatialValueTuner [2];
  int DigitsController;
  float digits[6];
  string controllerName;
};
