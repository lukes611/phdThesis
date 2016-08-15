#pragma once

/*

implementation of the iterative closest point algorithm
author : luke lincoln @ lukes611@gmail.com

requires: locv, R3

*/

//a n^2 complexity search for the closest point for each point in src to dst,
//returns an array of mappings src[i] mapped to dst[array[i]]
// ipc wiki: https://en.wikipedia.org/wiki/Iterative_closest_point
// kind of good: http://www.mrpt.org/Iterative_Closest_Point_(ICP)_and_other_matching_algorithms
// //optimal resource: http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
// : least squares: https://en.wikipedia.org/wiki/Linear_least_squares_(mathematics)


#include "../basics/locv3.h"
#include "../basics/R3.h"
#include "../basics/Pixel3DSet.h"

namespace Licp
{

	/*
		takes a list of R3 points : [x1,y1,z1,1], [x2,y2,z2,1], [x3,y3,z3,1] ... [xn,yn,zn,1]
		and returns a matrix where each R3 point is a row:

		[x1, y1, z1, 1]
		[x2, y2, z2, 1]
		[x3, y3, z3, 1]
		 .    .  .   .
		 .    .  .   .
		 .    .  .   .
		[xn, yn, zn, 1]
	*/
	Mat asMatRows(vector<ll_R3::R3> & inp);
	Mat asMatRows(ll_pix3d::Pixel3DSet & inp);

	//for each point in src: find the corresponding nearest neighbour in dst
	//returns a float representing the average distance from each closest point from src to dst
	//returns ptpairs: where src[i] matches with dst[ptpairs[i]] as output of the matching
	//dists represents the distance between src[i] and its match dst[ptpairs[i]]
	float knn(Mat & dst, Mat & src, vector<int> & ptpairs, vector<float> & dists = vector<float>());


	//finds the closest point for each point in src to a point in dst
	//returns a float representing the average distance from each closest point from src to dst
	//returns indexes: where src[i] matches with dst[indexes[i]] as output of the matching
	//distances represents the distance between src[i] and its match dst[ptpairs[i]]
	//algorithm : O(n^2) basic search
	double closestPoints(vector<ll_R3::R3> & src, vector<ll_R3::R3> & dst, vector<int> & indexes, vector<double> & distances);

	//finds the closest point for each point in src to a point in dst speedily
	//returns a float representing the average distance from each closest point from src to dst
	//returns indexes: where src[i] matches with dst[indexes[i]] as output of the matching
	//distances represents the distance between src[i] and its match dst[ptpairs[i]]
	//algorithm : O(n log n) k-d tree based algorithm
	double closestPointsf(vector<ll_R3::R3> & src, vector<ll_R3::R3> & dst, vector<int> & indexes, vector<double> & distances);
	double closestPointsf(ll_pix3d::Pixel3DSet & src, ll_pix3d::Pixel3DSet & dst, vector<int> & indexes, vector<double> & distances);

	//performs least squares to find the transform from all the points in src to all the points in dst
	//here: src[i] is matched with dst[indexes[i]] for a given index i, 
	Mat leastSquaresTransform(vector<ll_R3::R3> & src, vector<ll_R3::R3> & dst, vector<int> & indexes);


	//implementation of the iterative cloesest point algorithm
	//does not remove outliers
	Mat icp(	vector<ll_R3::R3> & src,		//source points
				vector<ll_R3::R3> & dst,		//destination points
				vector<ll_R3::R3> & out,		//registered points
				double & error_out,		//error between registered points and destination points
				double & time,			//the amount of time in seconds required to run this algorithm
				int & iterations,		//the number of iterations used
				double minError = -1.0, //the minimum error we want to achieve
				int maxIterations = -1  //the maximum number of iterations
			);

	Mat icp(	ll_pix3d::Pixel3DSet & src,		//source points
				ll_pix3d::Pixel3DSet & dst,		//destination points
				ll_pix3d::Pixel3DSet & out,		//registered points
				double & error_out,		//error between registered points and destination points
				double & time,			//the amount of time in seconds required to run this algorithm
				int & iterations,		//the number of iterations used
				double minError = -1.0, //the minimum error we want to achieve
				int maxIterations = -1  //the maximum number of iterations
			);

	//implementation of the iterative cloesest point algorithm
	//remove's outliers
	Mat icp_outlierRemoval(	vector<ll_R3::R3> & src,			//source points
				vector<ll_R3::R3> & dst,						//destination points
				vector<ll_R3::R3> & out,						//registered points
				double & error_out,								//error between registered points and destination points
				double & time,									//the amount of time in seconds required to run this algorithm
				int & iterations,								//the number of iterations used
				double outlierRemovalTimesMean,					//a value: matches are not outliers if: |match| <= ortm * |average|
				double minError = -1.0,							//the minimum error we want to achieve
				int maxIterations = -1							//the maximum number of iterations
			);
	Mat icp_outlierRemoval(	ll_pix3d::Pixel3DSet & src,			//source points
				ll_pix3d::Pixel3DSet & dst,						//destination points
				ll_pix3d::Pixel3DSet & out,						//registered points
				double & error_out,								//error between registered points and destination points
				double & time,									//the amount of time in seconds required to run this algorithm
				int & iterations,								//the number of iterations used
				double outlierRemovalTimesMean,					//a value: matches are not outliers if: |match| <= ortm * |average|
				double minError = -1.0,							//the minimum error we want to achieve
				int maxIterations = -1							//the maximum number of iterations
			);

}