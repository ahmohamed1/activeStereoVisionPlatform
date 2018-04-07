#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
// CUDA structures and methods
#include <opencv2/core/cuda.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/cudafilters.hpp>

using namespace cv;


int main(int argc,char** argv)
{
    ros::init(argc,argv,"RectifyImages");
    ros::NodeHandle nh;

    Mat src = imread("/home/abdulla/dev/workshop/gpu_test/spQSY.jpg");
    Mat dst;
    if (!src.data) exit(1);

    cuda::GpuMat d_src;
    d_src.upload(src);
    cuda::GpuMat d_dst;

    // (1) OpenCVビルド情報を表示する
    cout << cv::getBuildInformation() << std::endl;

    // (2) CUDA
    cuda::printCudaDeviceInfo(cv::cuda::getDevice());

    cuda::cvtColor(d_src,d_dst,CV_BGR2GRAY);
    cuda::bilateralFilter(d_dst, d_dst,10, 80,80);

    d_dst.download(dst);

    imshow("img", dst);
    waitKey();

}

