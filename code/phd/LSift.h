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

        double sad(const SiftFeature2D & o);
		double correlation(const SiftFeature2D & o);

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

	//perform matching
	void computeMatches(std::vector<SiftFeature2D> & fvs1, std::vector<SiftFeature2D> & fvs2, std::vector<cv::Point2i> & p1, std::vector<cv::Point2i> & p2, bool sort = true, int limit = -1);
	cv::Point2f operator*(cv::Mat M, const cv::Point2f& p);

	//high level functions:
	void lukes_sift(cv::Mat & im1, cv::Mat & im2, std::vector<cv::Point2i> & p1, std::vector<cv::Point2i> & p2, bool sort = false, int top = -1);//performs sift
	cv::Mat featureVisualization(std::vector<SiftFeature2D> & f, cv::Mat im);
	cv::Mat lukes_siftRegister(cv::Mat & im1, cv::Mat & im2, bool sort = true, int getTopXFeatures = 50, double allowedError = 1.2);

	//tests
	void testFeatures(cv::Mat & im, double R, double S, cv::Point2d T);
	void testMatches(cv::Mat & im, double R, double S, cv::Point2d T, bool sort = true, int top = 3);

	
}
