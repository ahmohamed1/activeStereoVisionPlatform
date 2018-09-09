#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"


/**
 * Function for showing window with image in normalized size
 * @param title Name of the window
 * @param img Image to be displayed
 */

struct SaliencyData{
    cv::Point2f centerOfTarget;
    cv::Rect2f boundingBox;
    float probabilityOfTomamto;


};

SaliencyData resizeSaliencyData(SaliencyData data, cv::Size smallImageSize, cv::Size largImageSize){
  float widthRatio = (float)largImageSize.width/smallImageSize.width;
  float hieghtRatio = (float)largImageSize.height/smallImageSize.height;
  // cout << widthRatio<< "////" << hieghtRatio <<endl;
  SaliencyData newScalar;
  newScalar.centerOfTarget.x = data.centerOfTarget.x * widthRatio;
  newScalar.centerOfTarget.y = data.centerOfTarget.y * hieghtRatio;
  newScalar.boundingBox.x = (float)data.boundingBox.x * widthRatio;
  newScalar.boundingBox.y = (float)data.boundingBox.y * hieghtRatio;
  newScalar.boundingBox.height = (float)data.boundingBox.height * hieghtRatio;
  newScalar.boundingBox.width = (float)data.boundingBox.width * widthRatio;
  newScalar.probabilityOfTomamto = (float)data.probabilityOfTomamto;
  // cout << (float)data.boundingBox.height << "  "<<(float)data.boundingBox.width <<endl;
  // cout << (float)newScalar.boundingBox.height << "  " << (float)newScalar.boundingBox.width <<endl;
  return newScalar;
}

class FocalOfAttentionBasedWatershed{

public:

    FocalOfAttentionBasedWatershed(bool _DebugActive){

        DebugActive = _DebugActive;
        windowsNameS = "image";
        windowsNameWatershed = "waterShedOutput";

        cv::namedWindow(windowsNameS, cv::WINDOW_NORMAL);
        cv::resizeWindow(windowsNameS, 720,500);
        cv::moveWindow(windowsNameS, 1000,30);

        // cv::namedWindow(windowsNameWatershed, cv::WINDOW_NORMAL);
        // cv::resizeWindow(windowsNameWatershed, 720,500);
        // cv::moveWindow(windowsNameWatershed, 1000,30);

        MostSalientPoint = cv::Point(-1, -1);
        updateWatershed = false;
        firstloop = true;
        maximumValueInSaliency = 255;
        samePointCount = 0;
        samePoint = cv::Point(0,0);

    }

    static void onMouseCallback( int event, int x, int y, int flags, void* ) {
        if( event == EVENT_LBUTTONDOWN ){
            cv::Point prevPt = Point(x,y);
//            updatemarkerMask(prevPt);

        }
    }

    void restart(){
        markerMask = Scalar::all(0);
        inputSaliency.copyTo(img);
//        showImageAuto( "image", img );
    }

    ////////////////
    /// \brief updatemarkerMask This function use to update themarkermask by taking the most salient point in the saliency map and draw it in the markerMask to pass it to tje contours
    /// \param centerPoint
    ///
    void updatemarkerMask(cv::Point centerPoint){
        circle(markerMask, centerPoint, 2,Scalar::all(255), -1);
        circle(img, centerPoint, 2,Scalar::all(255), -1);
        updateWatershed = true;
        if (DebugActive){
//            cv::imshow("markerMask", markerMask);q
        }

    }

    /////////////////////
    /// \brief drawFOA this to track the most salient feature in the image and update the main salincy map
    /// \return
    ///
    vector<SaliencyData> drawFOA(){
        int i, j, compCount = 0;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(markerMask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
//        if( contours.empty() )
//            return -1;
        Mat markers(markerMask.size(), CV_32S);
        markers = Scalar::all(0);
        int idx = 0;
        for( ; idx >= 0; idx = hierarchy[idx][0], compCount++ ){
            drawContours(markers, contours, idx, Scalar::all(compCount+1), -1, 8, hierarchy, INT_MAX);
        }

//        if( compCount == 0 )
//            return -1;
        vector<Vec3b> colorTab;
        for( i = 0; i < compCount; i++ )
        {
            int b = theRNG().uniform(0, 255);
            int g = theRNG().uniform(0, 255);
            int r = theRNG().uniform(0, 255);
            colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
        }
        double t = (double)getTickCount();
        cv::watershed( saliencyMapRGB, markers );
        t = (double)getTickCount() - t;
        if(DebugActive){
            printf( "execution time = %gms\n", t*1000./getTickFrequency() );
        }

        Mat wshed(markers.size(), CV_8UC3);
        // paint the watershed image
        for( i = 0; i < markers.rows; i++ )
            for( j = 0; j < markers.cols; j++ )
            {
                int index = markers.at<int>(i,j);
                if( index == -1 )
                    wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
                else if( index <= 0 || index > compCount )
                    wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
                else
                    wshed.at<Vec3b>(i,j) = colorTab[index - 1];
            }
        wshed = wshed*0.5 + img*0.5;
        // cv::imshow("windowsNameWatershed_1", wshed );

        cv::waitKey(1);
        vector<SaliencyData> data = FOABasedContours( markers, img, &inputSaliency, compCount);

        return data;
    }

    ////////////////////////////
    /// \brief FOABasedContours this function to compute the proparites of each object
    /// and draw on the saliency map to keep tracking the objects
    /// \param markers
    /// \param img0
    /// \param saliencyImage
    /// \param compCount
    vector<SaliencyData> FOABasedContours(cv::Mat markers, cv::Mat img0, cv::Mat *saliencyImage, int compCount){
        vector<SaliencyData> tempData;
        Mat originalImageCopy1;
        img0.copyTo(originalImageCopy1);
        for (int seed = 1; seed <= compCount; ++seed)
          {
            Mat1b mask1 = (markers == seed);
            // Now you have the mask, you can compute your statistics
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            findContours(mask1, contours, hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE);
            for( int i = 0; i< contours.size(); i=hierarchy[i][0] ) // iterate through each contour.
                {
                  Rect r= boundingRect(contours[i]);


                  if(hierarchy[i][2]<0 && r.width < img0.rows - 50){
                      //rectangle(originalImageCopy1,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,0,255),3,8,0);
//                      drawContours(originalImageCopy1, contours, i, Scalar(0,0,255), -1, 8);
                      //Draw on the Saliency map black
                      drawContours(*saliencyImage, contours, i, Scalar(0,0,0), -1, 8);
                      drawContours(*saliencyImage, contours, i, Scalar(0,0,0), 10, 8);
                      if (DebugActive){
                          std::cout << "Seed=" << seed << " Object " << i << std::endl;
                      }


                      // Compute the required information for the target and add it to the vectorcv::Point2f center;
                      float radius;
                      cv::Point2f center;
                      cv::minEnclosingCircle( contours[i], center, radius);

                      // Check if it is tomato or not
                      cv::Vec3b bgrPixel = originalImageCopy1.at<cv::Vec3b>(center.y, center.x);
                      // check the pixel if it belong to the tomato or not
                      float probability = -0.00258638* bgrPixel[0] + -0.00602994*bgrPixel[1] + 0.00700861*bgrPixel[2] + 0.49576749971493594;
                      if(probability > 0.60){
                          SaliencyData temp_;
                          temp_.boundingBox = r;
                          temp_.centerOfTarget = center;
                          temp_.probabilityOfTomamto = 1.0;
                          tempData.push_back(temp_);
                          drawContours(originalImageCopy1, contours, i, Scalar(0,255,255), -1, 8);
                          cv::circle(originalImageCopy1, center, 5, cv::Scalar(0,255,0),-1);
                          cv::rectangle(originalImageCopy1, cv::Rect(r) ,cv::Scalar(0,255,0));
                      }else{
//                          drawContours(originalImageCopy1, contours, i, Scalar(0,255,0), 2, 8);
                      }

                }else if(r.width < img0.rows - 50){
                      drawContours(*saliencyImage, contours, i, Scalar(0,0,0), -1, 8);
                      drawContours(*saliencyImage, contours, i, Scalar(0,0,0), 10, 8);
                  }
            }
        }
        cv::imshow(windowsNameS, originalImageCopy1);
        // cv::imshow(windowsNameWatershed, *saliencyImage);
        cv::waitKey(1);
        return tempData;
    }


    ///////////////////////////
    /// \brief mainLoop
    /// \param saliencyMap
    /// \param RGBimage
    void mainLoop(cv::Mat saliencyMap, cv::Mat RGBimage){

        saliencyMap.copyTo(inputSaliency);
        cvtColor(saliencyMap, saliencyMapRGB, CV_GRAY2BGR);
        RGBimage.copyTo(img);
        inputSaliency.copyTo(markerMask);
        markerMask = Scalar::all(0);
        vector<SaliencyData> saliencyData;
        while(1){
            char c = (char)waitKey(1);
            if( c == 27 || c =='q' )
                break;
            if( c == 'r' )
            {
                restart();
            }

            double minVal = 0;
            double maxVal = 0;
            Point maxLoc, minLoc;
            minMaxLoc(inputSaliency, &minVal, &maxVal, &minLoc, &maxLoc);

            if((maxVal/maximumValueInSaliency) > 0.5f){
                // if this is the first point select the min value then draw the max value
                if(firstloop){
                    updatemarkerMask(minLoc);
                    updatemarkerMask(maxLoc);
                    maximumValueInSaliency = maxVal;
                    firstloop = false;
                }else{
                    updatemarkerMask(maxLoc);
                }

                if(updateWatershed){
                    saliencyData = drawFOA();
                    char ikey = cv::waitKey(0);
                    if (ikey== 'q'){
                        break;
                    }
                }
            }else{
                //return the list and break
                cout <<"Total target Found: " << saliencyData.size() <<endl;
                break;
            }
        }// End of while loop
    }


    std::vector<SaliencyData> ComputeFOA(cv::Mat saliencyMap, cv::Mat RGBimage){

        saliencyMap.copyTo(inputSaliency);
        cvtColor(saliencyMap, saliencyMapRGB, CV_GRAY2BGR);
        RGBimage.copyTo(img);
        inputSaliency.copyTo(markerMask);
        markerMask = Scalar::all(0);
        std::vector<SaliencyData> saliencyData;

        markerMask = bhFindLocalMaximum(inputSaliency, 7);
        saliencyData = drawFOA();
        // cout<< "Totoal Object Found: " << saliencyData.size() << endl;
        char ikey = cv::waitKey(1);
        return saliencyData;

    }
    //////////////////////
    /// \brief FOABasedConnectComponent This function used connectComponent to compute the saliency information
    /// \param markers
    /// \param img0
    /// \param compCount
    ///
    void FOABasedConnectComponent(cv::Mat markers, cv::Mat img0, int compCount){
        Mat originalImageCopy;
        img0.copyTo(originalImageCopy);
        for (int seed = 1; seed <= compCount; ++seed)
          {
              Mat1b mask = (markers == seed);
              // Now you have the mask, you can compute your statistics
              Mat labels;
              Mat stats;
              Mat centroids;
              cv::connectedComponentsWithStats(mask, labels, stats, centroids);

              //std::cout << labels << std::endl;
              //std::cout << "stats.size()=" << labels.size() << std::endl;
    //                  std::cout << centroids << std::endl;

              for(int i=0; i<stats.rows; i++)
              {
                int x = stats.at<int>(Point(0, i));
                int y = stats.at<int>(Point(1, i));
                int w = stats.at<int>(Point(2, i));
                int h = stats.at<int>(Point(3, i));
                double area = stats.at<int>(Point(4, i));
                std::cout << "Seed=" << seed << " Object " << i <<" x=" << x << " y=" << y << " w=" << w << " h=" << h << " Area="<< area <<std::endl;

                Scalar color(255,0,0);
                int extra = 0;
                if(w < 200){
                    Rect rect(x-extra,y-extra,w+extra,h+extra);
                    cv::rectangle(originalImageCopy, rect, color);
                }

              }
          }
        cout<<"///////////////////////////////" << endl;
    }

    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    std::vector<cv::Point> bhContoursCenter(const std::vector<std::vector<cv::Point>>& contours,bool centerOfMass,int contourIdx=-1){
    std::vector<cv::Point> result;
    if (contourIdx > -1)
    {
        if (centerOfMass)
        {
            cv::Moments m = cv::moments(contours[contourIdx],true);
            result.push_back( Point(m.m10/m.m00, m.m01/m.m00));
        }
        else
        {
            cv::Rect rct = cv::boundingRect(contours[contourIdx]);
            result.push_back( cv::Point(rct.x + rct.width / 2 ,rct.y + rct.height / 2));
        }
    }
    else
    {
        if (centerOfMass)
        {
            for (int i=0; i < contours.size();i++)
            {
                cv::Moments m = cv::moments(contours[i],true);
                result.push_back( cv::Point(m.m10/m.m00, m.m01/m.m00));

            }
        }
        else
        {

            for (int i=0; i < contours.size(); i++)
            {
                cv::Rect rct = cv::boundingRect(contours[i]);
                result.push_back(Point(rct.x + rct.width / 2 ,rct.y + rct.height / 2));
            }
        }
    }

    return result;
}


    cv::Mat bhFindLocalMaximum(cv::InputArray _src, int neighbor=2){

        cv::Mat markerMask;
        _src.copyTo(markerMask);
        markerMask = cv::Scalar::all(0);

        cv::Mat src = _src.getMat();

        cv::Mat peak_img = src.clone();
        cv::dilate(peak_img,peak_img,Mat(),Point(-1,-1),neighbor);
        peak_img = peak_img - src;



        cv::Mat flat_img ;
        cv::erode(src,flat_img,Mat(),Point(-1,-1),neighbor);
        flat_img = src - flat_img;


        cv::threshold(peak_img,peak_img,0,255,CV_THRESH_BINARY);
        cv::threshold(flat_img,flat_img,0,255,CV_THRESH_BINARY);
        cv::bitwise_not(flat_img,flat_img);

        peak_img.setTo(Scalar::all(255),flat_img);
        cv::bitwise_not(peak_img,peak_img);


        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(peak_img,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
        cv::drawContours(src, contours, -1, cv::Scalar::all(0), 5);
        cv::drawContours(markerMask, contours, -1, cv::Scalar::all(255), 5);
        // cv::imshow("test",src );
        return markerMask;
    }

    cv::Mat FOASingleTarget(cv::Mat saliencyMapImage, cv::Mat BGRImage, int templateSize = 100){
      double minVal = 0;
      double maxVal = 0;
      Point maxLoc, minLoc;
      minMaxLoc(saliencyMapImage, &minVal, &maxVal, &minLoc, &maxLoc);
      cv::Point2f newCenter = recomputeTheCenter(maxLoc, saliencyMapImage.size(), BGRImage.size());
      cv::Rect cropRect = returnRectanguleTemplate(newCenter, templateSize, BGRImage.size());
      cv::Mat CroppedImage = BGRImage(cropRect);
      // cout << maxLoc << endl;
      return CroppedImage;

    }


    cv::Point2f recomputeTheCenter(cv::Point2f center, cv::Size smallImageSize, cv::Size largImageSize){
      cv::Point2f newCenter;
      newCenter.x = center.x * largImageSize.width/smallImageSize.width;
      newCenter.y = center.y * largImageSize.height/smallImageSize.height;

      return newCenter;
    }

    bool compareTargets(cv::Mat newTarget, cv::Mat oldTarget){

      cv::Mat difference;
      cv::absdiff(newTarget, oldTarget, difference);
      float result = cv::countNonZero(difference) / (newTarget.cols*newTarget.rows);
    }

    float computeDifferences(cv::Mat img1, cv::Mat img2){
        float diff = 0;
        for (int y = 0; y < img1.size().height; y++)
        {
            for (int x = 0; x < img1.size().width; x++)
            {
                diff += (float)abs(img1.at<uchar>(y, x) - img2.at<uchar>(y, x)) / 255;
            }
        }
        diff = diff / (img1.size().width * img1.size().height);
        // cout <<"diff: "<<  100 * diff << "%" << endl;

        return diff;
    }


private:
    cv::Mat img;
    cv::Mat markerMask;
    cv::Mat imgGray;
    cv::Mat inputSaliency, saliencyMapRGB;

    float maximumValueInSaliency;

    cv::Point MostSalientPoint;
    bool updateWatershed;
    bool firstloop;
    bool DebugActive;
    string windowsNameS, windowsNameWatershed;

    int samePointCount;
    cv::Point samePoint;

};
