#include "LSift.h"

using namespace cv;

namespace LukeLincoln
{
    double SiftFeature2D::difference(const SiftFeature2D & o)
    {
        return cv::sum(abs(featureVector - o.featureVector))[0];
    }

    double gaussian(double x, double y, double sigma)
	{
        double sigmaSquared = sigma * sigma;
		return (1.0 / (2.0 * 3.14159265359 * sigmaSquared)) * exp(-(x*x + y*y) / (2.0 * sigmaSquared));
	}

	double gaussian(double d, double sigma)
	{
        double sigmaSquared = sigma * sigma;
		return (1.0 / (2.0 * 3.14159265359 * sigmaSquared)) * exp(-(d) / (2.0 * sigmaSquared));
	}

    double gaussianDifference(double distanceFromOrigin, double sigma, double K)
	{
		return gaussian(distanceFromOrigin, K*sigma) - gaussian(distanceFromOrigin, sigma);
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

    void partialDerivatives1(Mat & im, int x, int y, float & dx, float & dy)
    {
        dx = im.at<float>(y, x + 1) - im.at<float>(y, x - 1);
        dy = im.at<float>(y + 1, x) - im.at<float>(y - 1, x);
    }

    void partialDerivatives2(Mat & im, int x, int y, float & dx, float & dy)
    {
        dx = (im.at<float>(y-1, x + 1) - im.at<float>(y-1, x - 1))
            +
            2.0f*(im.at<float>(y, x + 1) - 2.0f*im.at<float>(y, x - 1))
            +
            (im.at<float>(y+1, x + 1) - im.at<float>(y+1, x - 1))
        ;
        dy = (im.at<float>(y + 1, x-1) - im.at<float>(y - 1, x-1))
            +
            2.0f*(im.at<float>(y + 1, x) - 2.0f*im.at<float>(y - 1, x))
            +
            (im.at<float>(y + 1, x+1) - im.at<float>(y - 1, x+1));
        //dx /= 4.0f, dy /= 4.0f;
    }


}
