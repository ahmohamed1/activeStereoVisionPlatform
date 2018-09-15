

struct templateWithProbability{
  cv::Mat templateImage;
  float probability2D;
};
bool vergenceStatus = false;
void vergeStatue_callback(const std_msgs::Bool &data){
  vergenceStatus = data.data;
}

geometry_msgs::Vector3 targetPose;
void targetPose_callback(const geometry_msgs::Vector3 &data){
  targetPose = data;
}
// global virable
Mat left_img, right_img;
Size imageSize = Size(2048 , 1080);//Size(4096,2160);
Point2f windowsCenter(imageSize.width/2, imageSize.height/2);
bool updateTracker = false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        windowsCenter.x = x;
        windowsCenter.y = y;
        // cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        updateTracker = true;
    }

}


cv::Rect extendTectangle(cv::Rect actualRect, float extandSize, cv::Size imageSize){
  float x1 = actualRect.x - extandSize;
  float y1 = actualRect.y - extandSize;
  float x2 = x1 + actualRect.width + extandSize*2;
  float y2 = y1 + actualRect.height + extandSize*2;
  float rectWidth = actualRect.width + extandSize*2;
  float rectHeight = actualRect.height + extandSize*2;
  if(x1 < 0){
    x1 = 0;
    x2 = rectWidth;
  }
  if(x2 > imageSize.width){
    x2 = imageSize.width;
    x1 = imageSize.width - rectWidth;
  }
  if(y1 < 0){
    y1 = 0;
    y2 = rectHeight;
  }
  if(y2 > imageSize.height){
    y2 = imageSize.height;
    y1 = imageSize.height - rectHeight;
  }

// cout << x1 << "," << y1 << "," << x2 <<","<< y2<<endl;
Rect2f bbox(x1, y1, x2-x1, y2-y1);
return bbox;
}



tuple<cv::Mat ,bool> computeSaliencyMap(cv::Mat image, int windowSize, std::vector<cv::Mat> targetsList){
  cv::Mat imageSmaller;
  cv::resize(image,imageSmaller,cv::Size(640,420));
  // Compute saliency map
  cv::Mat SMImage = saliencyMap.ComputeSaliencyMap(imageSmaller);
  // Compute FOA
  // std::vector<SaliencyData> targets = FOA.ComputeFOA(imgSM, left_imgSmaller);
  cv::Mat temp = FOA.FOASingleTarget(SMImage, image, windowSize);

  // check if the template has been visited
  if(targetsList.size() > 0){
    for(int i = 0; i < targetsList.size(); i++){
      float diff = FOA.computeDifferences(temp,targetsList[i]);
      // cout <<"diff: "<<  100 * diff << "%" << endl;
      if(100*diff < 40){
        cout<< 100 * diff << "% this target have similar feature to target " << i << endl;
      }
    }
  }else{
    // break;
  }
  cv::imshow("Target", temp);
  cv::imshow(saliencymapName, SMImage);

  return make_tuple(temp, false);
}



tuple<cv::Mat ,bool> computeSaliencyMapTest2(cv::Mat image, int windowSize, std::vector<cv::Mat> targetsList){
  cv::Mat editedImage;
  image.copyTo(editedImage);
  // check if the template has been visited
  if(targetsList.size() > 0){
    for(int i = 0; i < targetsList.size(); i++){
      cv::Rect _tempRect;
      cv::Point2f difference;
      cv::Mat _editedImage;
      int scalarDiff = 10;
      tie(_editedImage, difference, _tempRect) = gazeFastMatchTemplate.trackTargetPNCC(editedImage, targetsList[i], 95);
      rectangle(editedImage,
               cv::Point(_tempRect.x+scalarDiff, _tempRect.y+scalarDiff),
               cv::Point(_tempRect.width-scalarDiff, _tempRect.height-scalarDiff),
               cv::Scalar::all(0) , -1);
    }
  }

  cv::Mat imageSmaller;
  cv::resize(editedImage,imageSmaller,cv::Size(640,420));
  // Compute saliency map
  cv::Mat SMImage = saliencyMap.ComputeSaliencyMap(imageSmaller);
  // Compute FOA
  // focalOfAttentionBasedWatershed.ComputeFOA(imgSM, left_imgSmaller);
  cv::Mat temp = FOA.FOASingleTarget(SMImage, editedImage, windowSize);

  cv::imshow("Target", temp);
  cv::imshow(saliencymapName, SMImage);

  editedImage.release();
  SMImage.release();
  return make_tuple(temp, false);
}


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
tuple<std::vector<templateWithProbability>, cv::Mat> computeMultipleTargets(cv::Mat left_img){
  std::vector<templateWithProbability> targetList;
  cv::Mat left_img_copy;
  left_img.copyTo(left_img_copy);
  cout<<"   Computing Saliency Map ======>"<<endl;
  cv::Mat imageSmaller;
  cv::resize(left_img,imageSmaller,cv::Size(640,420));
  // Compute saliency map
  cv::Mat SMImage = saliencyMap.ComputeSaliencyMap(imageSmaller);
  cv::imshow("SaliencyMap", SMImage);
  // Compute FOA
  std::vector<SaliencyData> data = FOA.ComputeFOA(SMImage, imageSmaller);
  for(int i =0; i < data.size(); i++){
    SaliencyData ScalaData = resizeSaliencyData(data[i], imageSmaller.size(), left_img.size());
    ScalaData.boundingBox = extendTectangle(ScalaData.boundingBox, 20, left_img.size());
    // cout << ScalaData.boundingBox <<endl;
    templateWithProbability _temp;
    _temp.templateImage = left_img(ScalaData.boundingBox);
    _temp.probability2D = ScalaData.probabilityOfTomamto;
    // cout<<"1"<<endl;
    targetList.push_back(_temp);
    cv::circle(left_img_copy, ScalaData.centerOfTarget, 10, cv::Scalar(0,255,0), -1);
    cv::rectangle(left_img_copy, cv::Rect(ScalaData.boundingBox) ,cv::Scalar(0,255,0), 5);
  }
  cout<<"   Total targets found = " << targetList.size() << endl;
  cv::resize(SMImage,SMImage,left_img.size());
  return make_tuple(targetList, SMImage);
}


void saveData(cv::Mat leftImage, cv::Mat rightImage, cv::Mat templateImage, cv::Mat saliencyMap, int targetID){

    stringstream ssl;
    ssl<<"/home/abdulla/dev/cognitiveMapData/"<<targetID <<"_left.jpg";
    string left_image_name = ssl.str();
    ssl.str("");
    imwrite(left_image_name, leftImage);

    stringstream ssr;
    ssr<<"/home/abdulla/dev/cognitiveMapData/"<<targetID <<"_right.jpg";
    string right_image_name = ssr.str();
    ssr.str("");
    imwrite(right_image_name, rightImage);

    stringstream sst;
    sst<<"/home/abdulla/dev/cognitiveMapData/"<<targetID <<"_template.jpg";
    string template_image_name = sst.str();
    sst.str("");
    imwrite(template_image_name, templateImage);

    stringstream sss;
    sss<<"/home/abdulla/dev/cognitiveMapData/"<<targetID <<"_saliencyMap.jpg";
    string saliencyMap_image_name = sss.str();
    sss.str("");
    imwrite(saliencyMap_image_name, saliencyMap);
}
