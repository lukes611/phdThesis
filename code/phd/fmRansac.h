#pragma once

/*

implementation of the iterative closest point algorithm
author : luke lincoln @ lukes611@gmail.com

requires: locv, R3, ll_pix3d

*/

#include "../basics/locv3.h"
#include "../basics/R3.h"
#include "../basics/Pixel3DSet.h"



namespace ll_fmrsc
{

	//performs feature matching using "algorithm" on objects 1 and 2 and returns the corresponding 2d point matches
	//it filters these by valid depth too
	int featureMatch(string algorithm,  //algorithm
					ll_pix3d::Pix3D & object1, //the first object
					ll_pix3d::Pix3D & object2, //the second object
					vector<Point2i> & p1, //a list of points in which p1[i] matches with p2[i]
					vector<Point2i> & p2,
					bool sort = false, //whether to sort the features by their match strength
					int top = -1); //only use the top "top" matches

	bool registerPix3D(string fm_algorithm, ll_pix3d::Pix3D & object1, ll_pix3d::Pix3D & object2, Mat & matrix, double & seconds, bool sort = false, int top = -1);
}