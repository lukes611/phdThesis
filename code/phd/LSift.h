#pragma once

/*

implementation of 2d sift and 3d sift
author : luke lincoln @ lukes611@gmail.com

requires: locv, R3, ll_pix3d, LMat



*/
#include "../basics/locv3.h"
#include <vector>


namespace LukeLincoln
{
    class SiftFeature2D {
    public:
		int x, y;
		double scale;
		int octave;
		float angle;
		cv::Mat featureVector;

        double difference(const SiftFeature2D & o);

        cv::Point2f truePoint();
        float trueRad();
	};

	//gaussian functions:
	double gaussian(double x, double y, double sigma);
    double gaussian(double distanceFromOrigin, double sigma);
    double gaussianDifference(double distanceFromOrigin, double sigma, double scalar);

    //gaussian image generators
    cv::Mat getGaussianImage(cv::Size s, double sigma);
    cv::Mat getGaussianDifferenceImage(cv::Size s, double sigma, double scalar);

    //derivative functions
    void partialDerivatives1(cv::Mat & im, int x, int y, float & dx, float & dy);
    void partialDerivatives2(cv::Mat & im, int x, int y, float & dx, float & dy);
    void partialDerivatives3(cv::Mat & im, int x, int y, float & dx, float & dy, float & dxy);

    //compute orientations and magnitudes
    float computeOrientation(int x, int y, cv::Mat & m);
    float computeMagnitude(int x, int y, cv::Mat & m);


    //is corner
    bool isCorner(int x, int y, cv::Mat & m);
    bool isCorner2(int x, int y, cv::Mat & m);

    //compute orientations
    std::vector<float> computeOrientations(SiftFeature2D & feature, cv::Mat & image);

    //compute feature signature
    void computeFeatureVectors(SiftFeature2D & ret, cv::Mat & im);

    //locate features:
    void findFeatures(std::vector<SiftFeature2D> & ret, int octave, cv::Mat & input);
    std::vector<SiftFeature2D> findFeatures(cv::Mat & input, int octave = 0);

    std::vector<SiftFeature2D> findMultiScaleFeatures(cv::Mat & input, int numOctaves);
}
