#pragma once

/*

implementation of phase correlation based registration (pca is also included here
author : luke lincoln @ lukes611@gmail.com

requires: locv, R3, ll_pix3d, volumePhaseCorrelator, gpu (cuda)

*/
#include "../basics/locv3.h"
#include "../basics/Pixel3DSet.h"
#include "experiments.h"

#if defined(HASCUDA) || defined(HASFFTW)

namespace ll_pc
{

	//performs phase correlation and returns the registration matrix as 4x4 mat
	cv::Mat pc_register(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, bool isScaled = true, int volumeSize = 256);

	//same as above but uses pca with pc, can do 3 axes of rotation
	cv::Mat pc_register_pca(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, bool isScaled = true, int volumeSize = 256);

	//does pc_register_pca a number of times to get a better registration
	cv::Mat pc_register_pca_i(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, int count = 2, int volumeSize = 256);

	//uses pca, pc and icp to find the transform
	cv::Mat pc_pca_icp(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, bool isScaled = true, int volumeSize = 256);
}

#endif



namespace ll_pca
{
	//does pca only
	cv::Mat register_pca(ll_pix3d::Pixel3DSet & object1, ll_pix3d::Pixel3DSet & object2, double & seconds, int volumeSize = 256);
}
