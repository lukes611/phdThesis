#include <iostream>
#include <string>
#include <vector>
#include <stack>
#include <functional>

#include "code/basics/locv3.h"
#include "code/basics/R3.h"
#include "code/basics/llCamera.h"
#include "code/basics/LTimer.h"
#include "code/basics/Pixel3DSet.h"
#include "code/basics/VMatF.h"
#include "code/phd/measurements.h"
#include "code/phd/Licp.h"
#include "code/script/LScript.h"
#include "code/phd/fmRansac.h"
//#include "code/pc/TheVolumePhaseCorrelator.h"
//#include "code/phd/Lpcr.h"
//#include "code/phd/experiments.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
//using namespace ll_experiments;
using namespace ll_siobj;




#include "code/basics/BitReaderWriter.h"

class L3DFeat{
public:
    R3 p, n; float s;
    L3DFeat(R3 p, R3 n, float s)
    {
        this->p=p;
        this->n=n;
        this->s=s;
    }
    L3DFeat(const L3DFeat & input)
    {
        copyIn(input);
    }
    L3DFeat & operator = (const L3DFeat & input)
    {
        copyIn(input);
        return *this;
    }
    void copyIn(const L3DFeat & input)
    {
        p = input.p;
        n = input.n;
        s = input.s;
    }
};

namespace ll_Sift
{

	typedef struct _llSiftFeat {
		int x, y, z;
		double scale;
		double octave;
		R3 normal;
		float angle1, angle2;
		double featureVector[2];
	} llSiftFeat;

	double gaussian(double x, double y, double R)
	{
		return (1.0 / (2.0 * 3.14159265359 * R*R)) * exp(-(x*x + y*y) / (2.0 * R*R));
	}

	double gaussian(double d, double R)
	{
		return (1.0 / (2.0 * 3.14159265359 * R*R)) * exp(-(d) / (2.0 * R*R));
	}

	double gaussianDifference(double d, double R, double K)
	{
		return gaussian(d, K*R) - gaussian(d, R);
	}

	Mat getGaussianImage(Size s, double R)
	{
		Mat ret = Mat::zeros(s, CV_32FC1);
		for (int y = 0; y < s.height; y++)
		{
			for (int x = 0; x < s.width; x++)
			{
				ret.at<float>(y, x) = (float)gaussian(ll_distance(x, y, s.width/2, s.height/2), R);
			}
		}
		return ret.clone();
	}

	Mat getGaussianDifferenceImage(Size s, double R, double K)
	{
		Mat ret = Mat::zeros(s, CV_32FC1);
		for (int y = 0; y < s.height; y++)
		{
			for (int x = 0; x < s.width; x++)
			{
				ret.at<float>(y, x) = (float)gaussianDifference(ll_distance(x, y, s.width / 2, s.height / 2), R, K);
			}
		}
		return ret.clone();
	}

	bool isTrueFeature(int x, int y, Mat & m)
	{
		float dxx = m.at<float>(y, x + 1) - m.at<float>(y, x - 1);
		float dyy = m.at<float>(y + 1, x) - m.at<float>(y - 1, x);
		float dxy = m.at<float>(y + 1, x + 1) - m.at<float>(y - 1, x - 1);

		dxx /= 2.f;
		dyy /= 2.f;
		dxy /= sqrt(8.0f);

		float tr = dxx + dyy;
		float det = dxx*dyy - dxy*dxy;
		
		float measure = (tr*tr) / det;
		float r = 10.0f;
		float r1 = r + 1.0f;
		//cout << measure << endl;
		
		return measure <= (r1*r1) / r;
	}

	float orientation(int x, int y, Mat & m)
	{
		float v =
			(m.at<float>(y + 1, x) - m.at<float>(y - 1, x)) /
			(m.at<float>(y, x + 1) - m.at<float>(y, x - 1));
		return atan(v);
	}



	vector<llSiftFeat> findFeatures(Mat & input)
	{
		vector<llSiftFeat> ret;
		double K = sqrt(2.0);
		Size kernelSize(7,7);
		int numLevels = 5;

		double R = K * 0.5;
		Mat g = getGaussianImage(kernelSize, R);
		Mat d = getGaussianDifferenceImage(kernelSize, R, K);
		Mat prevG; filter2D(input, prevG, input.depth(), g);
		Mat prevD; filter2D(input, prevD, input.depth(), d);


		R = R * K;
		g = getGaussianImage(kernelSize, R);
		d = getGaussianDifferenceImage(kernelSize, R, K);
		Mat currentG; filter2D(input, currentG, input.depth(), g);
		Mat currentD; filter2D(input, currentD, input.depth(), d);
		
		for (int i = 2; i <= numLevels; i++)
		{
			R = R * K;
			g = getGaussianImage(kernelSize, R);
			d = getGaussianDifferenceImage(kernelSize, R, K);
			Mat nextG; filter2D(input, nextG, input.depth(), g);
			Mat nextD; filter2D(input, nextD, input.depth(), d);
			

			//for each pixel, if feature vector: record
			for (int y = 2; y < input.size().height - 2; y++)
			{
				for (int x = 2; x < input.size().width - 2; x++)
				{
					float dv = currentD.at<float>(y, x);
					int lt = 0, gt = 0;
					for (int j = -1; j <= 1; j++)
					{
						for (int i = -1; i <= 1; i++)
						{
							if (i != 0 && j != 0)
							{
								if (dv > currentD.at<float>(y + j, x + i)) lt++;
								if (dv < currentD.at<float>(y + j, x + i)) gt++;
							}
							if (dv > prevD.at<float>(y + j, x + i)) lt++;
							if (dv < prevD.at<float>(y + j, x + i)) gt++;

							if (dv > nextD.at<float>(y + j, x + i)) lt++;
							if (dv < nextD.at<float>(y + j, x + i)) gt++;
						}
					}

					if (lt == 0 || gt == 0)
					{
						if (!isTrueFeature(x, y, currentG)) continue;
						
						llSiftFeat feature; feature.x = x; feature.y = y;
						feature.octave = 1.0;
						feature.scale = R/K;
						feature.angle1 = orientation(x, y, currentG) * 57.3f;
						R3::GetUnitPointFromAngle(feature.angle1, feature.normal.x, feature.normal.y);

						//compute feature vector
						//by: 
						
						ret.push_back(feature);
					}
				}
			}

			//setup for next iteration
			prevG = currentG;
			currentG = nextG;

			prevD = currentD;
			currentD = nextD;
		}
		return ret;
	}

}

int main(int argc, char * * argv)
{
   
	for (int i = 0; i <= 360; i += 2)
	{
		Mat lenna = imread("c:/lcppdata/ims/lena.png", CV_LOAD_IMAGE_GRAYSCALE);
		ll_UCF1_to_32F1(lenna);
		double D = rand() / (double)RAND_MAX;
		D *= 30.0;
		ll_transform_image(lenna, lenna, i, 1.0, D, 0.0);
		vector<ll_Sift::llSiftFeat> fl = ll_Sift::findFeatures(lenna);

		for (int i = 0; i < fl.size(); i++)
		{
			cv::circle(lenna, Point(fl[i].x, fl[i].y), 2, Scalar(1.0));
		}

		cout << "num feats: " << fl.size() << endl;

		imshow("l", lenna);
		waitKey();
	}


	return 0;
}




