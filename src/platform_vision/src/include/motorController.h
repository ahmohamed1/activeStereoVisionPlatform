class MotorController{

public:
  MotorController(ros::NodeHandle nh, string whichMotorToMove){
    std::string PanMotorTopic = "/" + whichMotorToMove + "/pan/move";
    std::string TiltMotorTopic = "/" + whichMotorToMove + "/tilt/move";
    motorPanPublisher = nh.advertise<std_msgs::Float64>(PanMotorTopic,10);
    motorTiltPublisher = nh.advertise<std_msgs::Float64>(TiltMotorTopic,10);
    currentPosition[0] = 0.0;
    currentPosition[1] = 0.0;

    exponatialGain[0] = 0.003;
    exponatialGain[1]= 0.0035; // pan, tilt
    mapExponatialValue[0] = 0.2;
    mapExponatialValue[1]= 0.35;
    thresholdMotorController[0] = 80;
    thresholdMotorController[1] = 12;
    motorMaxLimit = 75;
    motorMinLimit = -75;
    motorMinLimitTilt = -37;
    motorMaxLimitTilt = 37;
  }

  void setExponatialGrain(float pan, float tilt){
    exponatialGain[0] = pan;
    exponatialGain[1]= tilt; // pan, tilt
  }

  void SetMapExponatialValue(float pan, float tilt){
    mapExponatialValue[0] = pan;
    mapExponatialValue[1]= tilt; // pan, tilt
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

  void movePanMotor(float value){
    float speed = 0;
    speed = signnum(-value) * exp(abs(value)* exponatialGain[0])*mapExponatialValue[0];
    if (abs(value) > thresholdMotorController[0]){
      currentPosition[0] += speed;
      motorPos[0].data = currentPosition[0];
      std::cout << "Motor Pan Speed: " << currentPosition[0] << std::endl;
      if (currentPosition[0] < motorMaxLimit and currentPosition[0] > motorMinLimit){
        motorPanPublisher.publish(motorPos[0]);
      }

    }else if (abs(value) <  thresholdMotorController[0] && abs(value) >  thresholdMotorController[1]){
      currentPosition[0] -= value * 0.001;
      motorPos[0].data = currentPosition[0];
      std::cout << "Motor Pan Speed: " << currentPosition[0] << std::endl;
      if (currentPosition[0] < motorMaxLimit and currentPosition[0] > motorMinLimit){
        motorPanPublisher.publish(motorPos[0]);
      }
    }else{
      std::cout << "Motor Pan Center " << std::endl;
    }
  }

  void moveTiltMotor(float value){
    float speed = 0;
    speed = signnum(-value) * exp(abs(value)* exponatialGain[1])*mapExponatialValue[1];
    if (abs(value) > thresholdMotorController[1]){
      currentPosition[1] += speed;
      motorPos[1].data = currentPosition[1];
      std::cout << "Motor Tilt Speed: " << currentPosition[1] << std::endl;
      if (currentPosition[1] < motorMaxLimit and currentPosition[1] > motorMinLimit){
        motorTiltPublisher.publish(motorPos[1]);
      }

    }else if (abs(value) <  thresholdMotorController[1] && abs(value) >  thresholdMotorController[1]){
      currentPosition[1] -= value * 0.001;
      motorPos[1].data = currentPosition[1];
      std::cout << "Motor Tile Speed: " << currentPosition[1] << std::endl;
      if (currentPosition[1] < motorMaxLimit and currentPosition[1] > motorMinLimit){
        motorTiltPublisher.publish(motorPos[1]);
      }
    }else{
      std::cout << "Motor Tilte Center " << std::endl;
    }
  }
  void moveMotor(float value, int MotorID){
    float speed = 0;
    speed = signnum(-value) * exp(abs(value)* exponatialGain[MotorID])*mapExponatialValue[MotorID];
    if (abs(value) > thresholdMotorController[MotorID]){
      currentPosition[MotorID] += speed;
      motorPos[MotorID].data = currentPosition[MotorID];
      std::cout << "Motor Tilt Speed: " << currentPosition[MotorID] << std::endl;
      if (currentPosition[MotorID] < motorMaxLimit and currentPosition[MotorID] > motorMinLimit){
        whichMotorToUse(MotorID);
      }

    }else if (abs(value) <  thresholdMotorController[MotorID] && abs(value) >  thresholdMotorController[MotorID]){
      currentPosition[MotorID] -= value * 0.001;
      motorPos[MotorID].data = currentPosition[MotorID];
      std::cout << "Motor Tile Speed: " << currentPosition[MotorID] << std::endl;
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


void whichMotorToUse(int MotorID){
  if ( MotorID == 0){
    motorPanPublisher.publish(motorPos[MotorID]);
  }else if(MotorID == 1){
    motorTiltPublisher.publish(motorPos[MotorID]);
  }
}
private:
  ros::Publisher motorPanPublisher, motorTiltPublisher;
  float currentPosition[2];
  float exponatialGain [2];
  float mapExponatialValue [2];
  float thresholdMotorController[2];
  float motorMaxLimit;
  float motorMinLimit;
  float motorMinLimitTilt;
  float motorMaxLimitTilt;
  std_msgs::Float64 motorPos[2];
};
