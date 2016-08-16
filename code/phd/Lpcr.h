#pragma once

/*

implementation of phase correlation based registration (pca is also included here
author : luke lincoln @ lukes611@gmail.com

requires: locv, R3, ll_pix3d, volumePhaseCorrelator, gpu (cuda)

*/
#include "..\basics\locv3.h"
#include "..\basics\Pixel3DSet.h"

namespace ll_pc
{

	//performs phase correlation and returns the registration matrix as 4x4 mat
	Mat pc_register(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, bool isScaled = true, int volumeSize = 256);

	//same as above but uses pca with pc, can do 3 axes of rotation
	Mat pc_register_pca(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, bool isScaled = true, int volumeSize = 256);

	//does pc_register_pca a number of times to get a better registration
	Mat pc_register_pca_i(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, int count = 2, int volumeSize = 256);
}

namespace ll_pca
{
	//does pca only
	Mat register_pca(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, int volumeSize = 256);
}