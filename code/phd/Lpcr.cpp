#include "Lpcr.h"
#include "../basics/LTimer.h"
#include "../basics/VMatF.h"
#include "../pc/TheVolumePhaseCorrelator.h"
#include "../phd/Licp.h"

using namespace ll_pix3d;
using namespace cv;
using namespace std;

#ifdef HASCUDA

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

Mat pc_pca_icp(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, bool isScaled, int volumeSize)
{
	LTimer t; t.start();
	
	Mat m1 = pc_register_pca(object1, object2, seconds, isScaled, volumeSize);
	Pixel3DSet tmp = object1;
	tmp.transform_set(m1);

	Pixel3DSet out;
	int iters;
	double errorOut;
	double s2;
	Mat m2 = Licp::icp(tmp, object2, out, errorOut, s2, iters, 1.0, 150);
	seconds += s2;

	t.stop();
	seconds = t.getSeconds();
	Mat ret = m2 * m1;
	return ret.clone();
}

}

#endif

namespace ll_pca
{

Mat register_pca(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, int volumeSize)
{
	LTimer t; t.start();
	Mat ret = ll_algorithms::ll_pca_3d::LPCA::pca_register(object1, object2);
	t.stop();
	seconds = t.getSeconds();
	return ret.clone();
}

}

