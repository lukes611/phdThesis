#pragma once

/*

implementation of 2d sift and 3d sift
author : luke lincoln @ lukes611@gmail.com

requires: locv, R3, ll_pix3d, LMat



*/
#include "../basics/locv3.h"


namespace LukeLincoln
{
    class SiftFeature2D {
    public:
		int x, y;
		double scale;
		double octave;
		float angle;
		cv::Mat featureVector;

        double difference(const SiftFeature2D & o);

	};

	//gaussian functions:
	double gaussian(double x, double y, double sigma);
    double gaussian(double distanceFromOrigin, double sigma);
    double gaussianDifference(double distanceFromOrigin, double sigma, double scalar);

    //gaussian image generators
    cv::Mat getGaussianImage(cv::Size s, double sigma);
    cv::Mat getGaussianDifferenceImage(cv::Size s, double sigma, double scalar);

}
