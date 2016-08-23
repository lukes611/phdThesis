#include "Lpcr.h"
#include "../basics/LTimer.h"
#include "../basics/VMatF.h"
#include "../pc/TheVolumePhaseCorrelator.h"

using namespace ll_pix3d;
using namespace cv;
using namespace std;


namespace ll_pc
{

Mat pc_register(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, bool isScaled, int volumeSize)
{
	LTimer t; t.start();
	VMat VA(volumeSize, object1, 0.0f, true);
	VMat VB(volumeSize, object2, 0.0f, true);
	Mat ret = ll_volume_gpu::phase_correlate_rst(VA, VB);
	t.stop();
	seconds = t.getSeconds();
	return ret.clone();
}

Mat pc_register_pca(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, bool isScaled, int volumeSize)
{
	LTimer t; t.start();
	VMat VA(volumeSize, object1, 0.0f, isScaled);
	VMat VB(volumeSize, object2, 0.0f, isScaled);
	Mat ret = ll_volume_gpu::pca_phase_correlate_rst(VA, VB);
	t.stop();
	seconds = t.getSeconds();
	return ret.clone();
}

Mat pc_register_pca_i(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, int count, int volumeSize)
{
	Mat ret = Mat::eye(Size(4,4), CV_32FC1);
	Pixel3DSet src = object1;
	seconds = 0.0;
	for(int i = 0; i < count; i++)
	{
		double secs = 0.0;
		Mat _m = pc_register_pca(src, object2, secs, true, volumeSize);	
		seconds += secs;
		src.transform_set(_m);
		ret *= _m;
	}
	return ret.clone();
}

}

namespace ll_pca
{

Mat register_pca(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, int volumeSize)
{
	LTimer t; t.start();
	VMat VA(volumeSize, object1, 0.0f, true);
	VMat VB(volumeSize, object2, 0.0f, true);
	Mat ret = VMat::pca(VA, VB);
	t.stop();
	seconds = t.getSeconds();
	return ret.clone();
}

}

