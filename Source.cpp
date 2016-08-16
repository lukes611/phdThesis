#include <iostream>
#include <string>
#include <vector>

#include "code\basics\locv3.h"
#include "code\basics\R3.h"
#include "code\basics\llCamera.h"
#include "code\basics\VMatF.h"
#include "code\basics\LTimer.h"
#include "code\phd\Licp.h"
#include "code\phd\measurements.h"
#include "code\phd\fmRansac.h"
#include "code\pc\TheVolumePhaseCorrelator.h"

using namespace std;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
//using namespace cv;

//proto-type for experiments: VMat pc(VMat a, VMat b, double & time, Mat & transform, double & mse);
//to-do:= pc, ipc, 

/*
read in 1 x Pixel3DSets
mutate points slightly
get knn working with 4d data

*/



Mat pc_register(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, bool isScaled = true, int volumeSize = 256)
{
	LTimer t; t.start();
	VMat VA(volumeSize, object1, 0.0f, true);
	VMat VB(volumeSize, object2, 0.0f, true);
	Mat ret = ll_volume_gpu::phase_correlate_rst(VA, VB);
	t.stop();
	seconds = t.getSeconds();
	return ret.clone();
}


Mat pc_register_pca(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, bool isScaled = true, int volumeSize = 256)
{
	LTimer t; t.start();
	VMat VA(volumeSize, object1, 0.0f, isScaled);
	VMat VB(volumeSize, object2, 0.0f, isScaled);
	Mat ret = ll_volume_gpu::pca_phase_correlate_rst(VA, VB);
	t.stop();
	seconds = t.getSeconds();
	return ret.clone();
}

Mat register_pca(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, int volumeSize = 256)
{
	LTimer t; t.start();
	VMat VA(volumeSize, object1, 0.0f, true);
	VMat VB(volumeSize, object2, 0.0f, true);
	Mat ret = VMat::pca(VA, VB);
	t.stop();
	seconds = t.getSeconds();
	return ret.clone();
}

Mat pc_register_pca_i(Pixel3DSet & object1, Pixel3DSet & object2, double & seconds, int count = 2, int volumeSize = 256)
{
	Mat ret = Mat::eye(Size(4,4), CV_32FC1);
	Pixel3DSet src = object1;
	seconds = 0.0;
	for(int i = 0; i < count; i++)
	{
		double secs = 0.0;
		Mat _m = pc_register_pca(src, object2, secs, volumeSize);	
		seconds += secs;
		src.transform_set(_m);
		ret *= _m;
	}
	return ret.clone();
}




int main(int argc, char * * argv)
{
	CapturePixel3DSet cap("Apartment.Texture.rotate", 8);

	Pixel3DSet p1, p2, p3;

	cap.read_frame(p1, 0);

	p1.normalize(256);
	cap.read_frame(p2, 5);
	
	p2 = p1.clone();
	p3 = p1.clone();

	p2.transform_set(0.0f, 30.0f, 30.0f, 1.0f, 5.0f, 10.0f, -5.0f, R3(256/2,256/2,256/2));
	double s;
	Mat m = register_pca(p1, p2, s, 256);
	p1.transform_set(m);

	cout << "seconds: " << s << endl;
	//Mat m2 = pc_registerPCA(p1, p2);

	//p1.transform_set(m2);

	
	Pixel3DSet t1 = p3 + p2;
	Pixel3DSet t2 = p1 + p2;
	Mat a, b;
	Pix3D A = render(t1, 256, 500.0f);
	Pix3D B = render(t2, 256, 500.0f);
	
	A.colorImage(a);
	B.colorImage(b);
	
	imshow("before", a);
	imshow("after", b);

	waitKey(30);

	cout << "before/after avg: " << avg(p3, p2) << " / " << avg(p1, p2) << endl;
	//cout << "before/after % m: " << percentMatch(p3, p2, 2.0f) << " / " << percentMatch(p1, p2, 2.0f) << endl;

	waitKey();
	return 0;
}
