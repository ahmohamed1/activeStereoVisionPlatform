
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/* Constants declaration */
const double WEIGHT_INTENSITY = 0.20f;
const double WEIGHT_COLOR = 0.50f;
const double WEIGHT_ORIENTATION = 0.30f;
const double SCALE_GAUSS_PYRAMID = 1.7782794100389228012254211951927; // = 100^0.125
const int DEFAULT_STEP_LOCAL = 8;


class IttiSaliencyMap
{
public:
	// default constructor
    IttiSaliencyMap(int temp);
	// method for generating output saliency map
    cv::Mat ComputeSaliencyMap(cv::Mat src);
	// matrix of saliency map
	cv::Mat salMap;

private:
	// matrixes for default color and intensity channels
	cv::Mat R, G, B, I;

	// matrixes for gabor kernels
	cv::Mat GaborKernel0;
	cv::Mat GaborKernel45;
	cv::Mat GaborKernel90;
	cv::Mat GaborKernel135;

	// splitting color channels
	void SMExtractRGBI(cv::Mat inputImage);
	// extracting feature maps
	void IFMGetFM(cv::Mat dst[6]);
	void CFMGetFM(cv::Mat RGFM[6], cv::Mat BYFM[6]);
	void OFMGetFM(cv::Mat dst[24]);
	// extracting conspicuity maps
	cv::Mat ICMGetCM(cv::Mat IFM[6], cv::Size size);
	cv::Mat CCMGetCM(cv::Mat CFM_RG[6], cv::Mat CFM_BY[6], cv::Size size);
	cv::Mat OCMGetCM(cv::Mat OFM[24], cv::Size size);

	void FMGaussianPyrCSD(cv::Mat src, cv::Mat dst[6]);
	void FMCreateGaussianPyr(cv::Mat src, cv::Mat dst[9]);
	void FMCenterSurroundDiff(cv::Mat GaussianMap[9], cv::Mat dst[6]);
	// normalization
	void normalizeFeatureMaps(cv::Mat FM[6], cv::Mat NFM[6], int width, int height, int num_maps);
	cv::Mat SMNormalization(cv::Mat src); // Itti normalization
	cv::Mat SMRangeNormalize(cv::Mat src); // dynamic range normalization
	double SMAvgLocalMax(cv::Mat src);
};

// constants for gabor kernels
static double GaborKernel_0[9][9] = {
	{ 1.85212E-06, 1.28181E-05, -0.000350433, -0.000136537, 0.002010422, -0.000136537, -0.000350433, 1.28181E-05, 1.85212E-06 },
	{ 2.80209E-05, 0.000193926, -0.005301717, -0.002065674, 0.030415784, -0.002065674, -0.005301717, 0.000193926, 2.80209E-05 },
	{ 0.000195076, 0.001350077, -0.036909595, -0.014380852, 0.211749204, -0.014380852, -0.036909595, 0.001350077, 0.000195076 },
	{ 0.00062494, 0.004325061, -0.118242318, -0.046070008, 0.678352526, -0.046070008, -0.118242318, 0.004325061, 0.00062494 },
	{ 0.000921261, 0.006375831, -0.174308068, -0.067914552, 1, -0.067914552, -0.174308068, 0.006375831, 0.000921261 },
	{ 0.00062494, 0.004325061, -0.118242318, -0.046070008, 0.678352526, -0.046070008, -0.118242318, 0.004325061, 0.00062494 },
	{ 0.000195076, 0.001350077, -0.036909595, -0.014380852, 0.211749204, -0.014380852, -0.036909595, 0.001350077, 0.000195076 },
	{ 2.80209E-05, 0.000193926, -0.005301717, -0.002065674, 0.030415784, -0.002065674, -0.005301717, 0.000193926, 2.80209E-05 },
	{ 1.85212E-06, 1.28181E-05, -0.000350433, -0.000136537, 0.002010422, -0.000136537, -0.000350433, 1.28181E-05, 1.85212E-06 }
};

static double GaborKernel_45[9][9] = {
	{ 4.0418E-06, 2.2532E-05, -0.000279806, -0.001028923, 3.79931E-05, 0.000744712, 0.000132863, -9.04408E-06, -1.01551E-06 },
	{ 2.2532E-05, 0.00092512, 0.002373205, -0.013561362, -0.0229477, 0.000389916, 0.003516954 , 0.000288732, -9.04408E-06 },
	{ -0.000279806, 0.002373205, 0.044837725, 0.052928748, -0.139178011, -0.108372072, 0.000847346 , 0.003516954, 0.000132863 },
	{ -0.001028923, -0.013561362, 0.052928748, 0.46016215, 0.249959607, -0.302454279, -0.108372072, 0.000389916, 0.000744712 },
	{ 3.79931E-05, -0.0229477, -0.139178011, 0.249959607, 1, 0.249959607, -0.139178011, -0.0229477, 3.79931E-05 },
	{ 0.000744712, 0.000389916, -0.108372072, -0.302454279, 0.249959607, 0.46016215, 0.052928748, -0.013561362, -0.001028923 },
	{ 0.000132863, 0.003516954, 0.000847346, -0.108372072, -0.139178011, 0.052928748, 0.044837725, 0.002373205, -0.000279806 },
	{ -9.04408E-06, 0.000288732, 0.003516954, 0.000389916, -0.0229477, -0.013561362, 0.002373205, 0.00092512, 2.2532E-05 },
	{ -1.01551E-06, -9.04408E-06, 0.000132863, 0.000744712, 3.79931E-05, -0.001028923, -0.000279806, 2.2532E-05, 4.0418E-06 }
};

static double GaborKernel_90[9][9] = {
	{ 1.85212E-06, 2.80209E-05, 0.000195076, 0.00062494, 0.000921261, 0.00062494, 0.000195076, 2.80209E-05, 1.85212E-06 },
	{ 1.28181E-05, 0.000193926, 0.001350077, 0.004325061, 0.006375831, 0.004325061, 0.001350077, 0.000193926, 1.28181E-05 },
	{ -0.000350433, -0.005301717, -0.036909595, -0.118242318, -0.174308068, -0.118242318, -0.036909595, -0.005301717, -0.000350433 },
	{ -0.000136537, -0.002065674, -0.014380852, -0.046070008, -0.067914552, -0.046070008, -0.014380852, -0.002065674, -0.000136537 },
	{ 0.002010422, 0.030415784, 0.211749204, 0.678352526, 1, 0.678352526, 0.211749204, 0.030415784, 0.002010422 },
	{ -0.000136537, -0.002065674, -0.014380852, -0.046070008, -0.067914552, -0.046070008, -0.014380852, -0.002065674, -0.000136537 },
	{ -0.000350433, -0.005301717, -0.036909595, -0.118242318, -0.174308068, -0.118242318, -0.036909595, -0.005301717, -0.000350433 },
	{ 1.28181E-05, 0.000193926, 0.001350077, 0.004325061, 0.006375831, 0.004325061, 0.001350077, 0.000193926, 1.28181E-05 },
	{ 1.85212E-06, 2.80209E-05, 0.000195076, 0.00062494, 0.000921261, 0.00062494, 0.000195076, 2.80209E-05, 1.85212E-06 }
};

static double GaborKernel_135[9][9] = {
	{ -1.01551E-06, -9.04408E-06, 0.000132863, 0.000744712, 3.79931E-05, -0.001028923, -0.000279806, 2.2532E-05, 4.0418E-06 },
	{ -9.04408E-06, 0.000288732, 0.003516954, 0.000389916, -0.0229477, -0.013561362, 0.002373205, 0.00092512, 2.2532E-05 },
	{ 0.000132863, 0.003516954, 0.000847346, -0.108372072, -0.139178011, 0.052928748, 0.044837725, 0.002373205, -0.000279806 },
	{ 0.000744712, 0.000389916, -0.108372072, -0.302454279, 0.249959607, 0.46016215, 0.052928748, -0.013561362, -0.001028923 },
	{ 3.79931E-05, -0.0229477, -0.139178011, 0.249959607, 1, 0.249959607, -0.139178011, -0.0229477, 3.79931E-05 },
	{ -0.001028923, -0.013561362, 0.052928748, 0.46016215, 0.249959607 , -0.302454279, -0.108372072, 0.000389916, 0.000744712 },
	{ -0.000279806, 0.002373205, 0.044837725, 0.052928748, -0.139178011, -0.108372072, 0.000847346, 0.003516954, 0.000132863 },
	{ 2.2532E-05, 0.00092512, 0.002373205, -0.013561362, -0.0229477, 0.000389916, 0.003516954, 0.000288732, -9.04408E-06 },
	{ 4.0418E-06, 2.2532E-05, -0.000279806, -0.001028923, 3.79931E-05 , 0.000744712, 0.000132863, -9.04408E-06, -1.01551E-06 }
};
