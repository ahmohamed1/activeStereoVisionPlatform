#include <iostream>

#include "opencv2/opencv_modules.hpp"

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::cuda;
using namespace cv::xfeatures2d;

// It searches for the right position, orientation and scale of the object in the scene based on the good_matches.
void localizeInImage(const std::vector<DMatch>& good_matches,
        const std::vector<KeyPoint>& keypoints_object,
        const std::vector<KeyPoint>& keypoints_scene, const Mat& img_object,
        const Mat& img_matches)
{
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for (int i = 0; i < good_matches.size(); i++) {
        //-- Get the keypoints from the good matches
        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    try {
        Mat H = findHomography(obj, scene, RANSAC);
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(img_object.cols, 0);
        obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
        obj_corners[3] = cvPoint(0, img_object.rows);
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform(obj_corners, scene_corners, H);
        Point2f center = (scene_corners[0]+scene_corners[1]+scene_corners[2]+scene_corners[3])/4;
//        cout << center <<endl;

        // Draw lines between the corners (the mapped object in the scene - image_2 )
        line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0),
                scene_corners[1] + Point2f(img_object.cols, 0),
                Scalar(255, 0, 0), 4);
        line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0),
                scene_corners[2] + Point2f(img_object.cols, 0),
                Scalar(255, 0, 0), 4);
        line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0),
                scene_corners[3] + Point2f(img_object.cols, 0),
                Scalar(255, 0, 0), 4);
        line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0),
                scene_corners[0] + Point2f(img_object.cols, 0),
                Scalar(255, 0, 0), 4);
    } catch (Exception& e) {}
}

//void processWithGpu(string objectInputFile, string sceneInputFile, string outputFile, int minHessian = 100)
bool processWithGpu(Mat img, Mat obj, int minHessian = 100)
{
//    printf("GPU::Processing object: %s and scene: %s ...\n", objectInputFile.c_str(), sceneInputFile.c_str());

    // Load the image from the disk
    Mat img_object, img_scene ;
    cv::cvtColor(obj,img_object,cv::COLOR_BGR2GRAY);
    cv::cvtColor(img,img_scene,cv::COLOR_BGR2GRAY);
    if( !img_object.data || !img_scene.data ) {
        std::cout<< "Error reading images." << std::endl;
        return false;
    }

    // Copy the image into GPU memory
    cuda::GpuMat img_object_Gpu( img_object );
    cuda::GpuMat img_scene_Gpu( img_scene );

    cuda::GpuMat keypoints_scene_Gpu, keypoints_object_Gpu; // keypoints
    cuda::GpuMat descriptors_scene_Gpu, descriptors_object_Gpu; // descriptors (features)

    //-- Steps 1 + 2, detect the keypoints and compute descriptors, both in one method
    cuda::SURF_CUDA surf( minHessian );
    surf( img_object_Gpu, cuda::GpuMat(), keypoints_object_Gpu, descriptors_object_Gpu );
    surf( img_scene_Gpu, cuda::GpuMat(), keypoints_scene_Gpu, descriptors_scene_Gpu );
    //cout << "FOUND " << keypoints_object_Gpu.cols << " keypoints on object image" << endl;
    //cout << "Found " << keypoints_scene_Gpu.cols << " keypoints on scene image" << endl;

    //-- Step 3: Matching descriptor vectors using BruteForceMatcher
    Ptr< cuda::DescriptorMatcher > matcher = cuda::DescriptorMatcher::createBFMatcher();
    vector< vector< DMatch> > matches;
    matcher->knnMatch(descriptors_object_Gpu, descriptors_scene_Gpu, matches, 2);

    // Downloading results  Gpu -> Cpu
    vector< KeyPoint > keypoints_scene, keypoints_object;
    //vector< float> descriptors_scene, descriptors_object;
    surf.downloadKeypoints(keypoints_scene_Gpu, keypoints_scene);
    surf.downloadKeypoints(keypoints_object_Gpu, keypoints_object);



    //-- Step 4: Select only goot matches
    //vector<Point2f> obj, scene;
    std::vector< DMatch > good_matches;
    for (int k = 0; k < std::min(keypoints_object.size()-1, matches.size()); k++)
    {
        if ( (matches[k][0].distance < 0.6*(matches[k][1].distance)) &&
                ((int)matches[k].size() <= 2 && (int)matches[k].size()>0) )
        {
            // take the first result only if its distance is smaller than 0.6*second_best_dist
            // that means this descriptor is ignored if the second distance is bigger or of similar
            good_matches.push_back(matches[k][0]);
        }
    }

    //-- Step 5: Draw lines between the good matching points
    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
            good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
            vector<char>(), DrawMatchesFlags::DEFAULT );

    //-- Step 6: Localize the object inside the scene image with a square
    localizeInImage( good_matches, keypoints_object, keypoints_scene, img_object, img_matches );

    //-- Step 7: Show/save matches
    cv::namedWindow("Good Matches & Object detection",cv::WINDOW_NORMAL);
    cv::resizeWindow("Good Matches & Object detection", 900,600);
    imshow("Good Matches & Object detection", img_matches);
    char ikey = waitKey('0');
//    imwrite(outputFile, img_matches);
    if (ikey == 'q'){
    //-- Step 8: Release objects from the GPU memory
    surf.releaseMemory();
    matcher.release();
    img_object_Gpu.release();
    img_scene_Gpu.release();
    cout<<"Close GPU tracking"<<endl;
    return false;
    }
    return true;
}



bool ORBFullGpu(const cv::Mat &scene_image, const cv::Mat object_image) {
    //Upload from host memory to gpu device memeory
    cv::cuda::GpuMat scene_image_gpu(scene_image), object_image_gpu(object_image);
    cv::cuda::GpuMat scene_image_gray_gpu, object_image_gray_gpu;

    //Convert RGB to grayscale as gpu detectAndCompute only allow grayscale GpuMat
    cv::cuda::cvtColor(scene_image_gpu, scene_image_gray_gpu, CV_BGR2GRAY);
    cv::cuda::cvtColor(object_image_gpu, object_image_gray_gpu, CV_BGR2GRAY);

    //Create a GPU ORB feature object
    //blurForDescriptor=true seems to give better results
    //http://answers.opencv.org/question/10835/orb_gpu-not-as-good-as-orbcpu/
    cv::Ptr<cv::cuda::ORB> orb = cv::cuda::ORB::create(500, 1.2f, 8, 31, 0, 2, 0, 31, 20, true);

    cv::cuda::GpuMat keypoints_scene_Gpu, descriptors_scene_Gpu;
    //Detect ORB keypoints and extract descriptors on train image (box.png)
    orb->detectAndComputeAsync(scene_image_gray_gpu, cv::cuda::GpuMat(), keypoints_scene_Gpu, descriptors_scene_Gpu);
    std::vector<cv::KeyPoint> keypoints_scene;
    //Convert from CUDA object to std::vector<cv::KeyPoint>
    orb->convert(keypoints_scene_Gpu, keypoints_scene);
    // std::cout << "keypoints_scene=" << keypoints_scene.size() << " ; descriptors_scene_Gpu=" << descriptors_scene_Gpu.rows
    //     << "x" << descriptors_scene_Gpu.cols << std::endl;

    std::vector<cv::KeyPoint> keypoints_object;
    cv::cuda::GpuMat descriptors_object_Gpu;
    //Detect ORB keypoints and extract descriptors on query image (box_in_scene.png)
    //The conversion from internal data to std::vector<cv::KeyPoint> is done implicitly in detectAndCompute()
    orb->detectAndCompute(object_image_gray_gpu, cv::cuda::GpuMat(), keypoints_object, descriptors_object_Gpu);
    // std::cout << "keypoints_object=" << keypoints_object.size() << " ; descriptors_object_Gpu=" << descriptors_object_Gpu.rows
    //     << "x" << descriptors_object_Gpu.cols << std::endl;

    //Create a GPU brute-force matcher with Hamming distance as we use a binary descriptor (ORB)
    cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);

    std::vector<std::vector<cv::DMatch> > knn_matches;
    //Match each query descriptor to a train descriptor
    matcher->knnMatch(descriptors_object_Gpu, descriptors_scene_Gpu, knn_matches, 2);
    // std::cout << "knn_matches=" << knn_matches.size() << std::endl;

    std::vector<cv::DMatch> matches;
    //Filter the matches using the ratio test
    for(std::vector<std::vector<cv::DMatch> >::const_iterator it = knn_matches.begin(); it != knn_matches.end(); ++it) {
        if(it->size() > 1 && (*it)[0].distance/(*it)[1].distance < 0.8) {
            matches.push_back((*it)[0]);
        }
    }

    cv::Mat imgRes;
    //Display and save the image with matches
    cv::drawMatches(object_image, keypoints_object, scene_image, keypoints_scene, matches, imgRes);
    //-- Step 6: Localize the object inside the scene image with a square
    localizeInImage( matches, keypoints_object, keypoints_scene, object_image, imgRes );

    cv::namedWindow("example_with_full_gpu",cv::WINDOW_NORMAL);
    cv::resizeWindow("example_with_full_gpu", 900,600);
    cv::imshow("example_with_full_gpu", imgRes);
    char ikey = waitKey('0');
//    imwrite(outputFile, img_matches);
    if (ikey == 'q'){
    //-- Step 8: Release objects from the GPU memory
    matcher.release();
    scene_image_gpu.release();
    scene_image_gray_gpu.release();
    object_image_gpu.release();
    object_image_gray_gpu.release();
    cout<<"Close GPU tracking"<<endl;
    return false;
    }
    return true;
}

bool ORBGpuMatching(const cv::Mat &scene_image, const cv::Mat object_image) {
    //Create a CPU ORB feature object
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, 0, 31, 20);

    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat descriptors1;
    //Detect ORB keypoints and extract descriptors on train image (box.png)
    orb->detectAndCompute(scene_image, cv::Mat(), keypoints_scene, descriptors1);
    // std::cout << "keypoints_scene=" << keypoints_scene.size() << " ; descriptors1=" << descriptors1.rows
    //     << "x" << descriptors1.cols << std::endl;

    std::vector<cv::KeyPoint> keypoints_object;
    cv::Mat descriptors2;
    //Detect ORB keypoints and extract descriptors on query image (box_in_scene.png)
    orb->detectAndCompute(object_image, cv::Mat(), keypoints_object, descriptors2);
    // std::cout << "keypoints_object=" << keypoints_object.size() << " ; descriptors2=" << descriptors2.rows
    //     << "x" << descriptors2.cols << std::endl;

    //Create a GPU brute-force matcher with Hamming distance as we use a binary descriptor (ORB)
    cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);

    //Upload from host memory to gpu device memeory
    cv::cuda::GpuMat descriptors_scene_Gpu(descriptors1), descriptors_object_Gpu;
    //Upload from host memory to gpu device memeory (another way to do it)
    descriptors_object_Gpu.upload(descriptors2);

    std::vector<std::vector<cv::DMatch> > knn_matches;
    //Match each query descriptor to a train descriptor
    matcher->knnMatch(descriptors_object_Gpu, descriptors_scene_Gpu, knn_matches, 2);
    // std::cout << "knn_matches=" << knn_matches.size() << std::endl;

    std::vector<cv::DMatch> matches;
    //Filter the matches using the ratio test
    for(std::vector<std::vector<cv::DMatch> >::const_iterator it = knn_matches.begin(); it != knn_matches.end(); ++it) {
        if(it->size() > 1 && (*it)[0].distance/(*it)[1].distance < 0.8) {
            matches.push_back((*it)[0]);
        }
    }

    cv::Mat imgRes;
    //Display and save the image with matches
    cv::drawMatches(object_image, keypoints_object, scene_image, keypoints_scene, matches, imgRes);
    //-- Step 6: Localize the object inside the scene image with a square
    localizeInImage( matches, keypoints_object, keypoints_scene, object_image, imgRes );
    cv::namedWindow("ORBGpuMatching",cv::WINDOW_NORMAL);
    cv::resizeWindow("ORBGpuMatching", 900,600);
    cv::imshow("ORBGpuMatching", imgRes);
    char ikey = waitKey('0');
//    imwrite(outputFile, img_matches);
    if (ikey == 'q'){
    //-- Step 8: Release objects from the GPU memory
    matcher.release();
    cout<<"Close GPU tracking"<<endl;
    return false;
    }
    return true;
}
