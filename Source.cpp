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



Mat pc_register(Pixel3DSet & object1, Pixel3DSet & object2, int volumeSize = 256)
{
	VMat VA(volumeSize, object1, 0.0f, true);
	VMat VB(volumeSize, object2, 0.0f, true);
	//Mat ret = ll_volume_gpu::phase_correlate_rst(VA, VB);
	Mat ret; float r, s; R3 t;
	ll_volume_gpu::phase_correlate_rst(VA, VB, r, s, t);
	cout << Point2f(r,s) << " | " << t << endl;
	ret = VMat::transformation_matrix(volumeSize, 0.0f, r, 0.0f, s, t.x, t.y, t.z);
	return ret.clone();
}


Mat pc_registerPCA(Pixel3DSet & object1, Pixel3DSet & object2, int volumeSize = 256)
{
	VMat VA(volumeSize, object1, 0.0f, true);
	VMat VB(volumeSize, object2, 0.0f, true);
	Mat ret = ll_volume_gpu::pca_phase_correlate_rst(VA, VB);
	return ret.clone();
}

Mat _PCA(Pixel3DSet & object1, Pixel3DSet & object2, int volumeSize = 256)
{
	VMat VA(volumeSize, object1, 0.0f, true);
	VMat VB(volumeSize, object2, 0.0f, true);
	Mat ret = VMat::pca(VA, VB);
	return ret.clone();
}




int main(int argc, char * * argv)
{
	CapturePixel3DSet cap("Apartment.Texture.rotate", 8);

	Pixel3DSet p1, p2, p3;

	cap.read_frame(p1, 0);

	//p1.normalize(256);
	cap.read_frame(p2, 5);
	
	//p2 = p1.clone();
	p3 = p1.clone();

	//p2.transform_set(0.0f, 30.0f, 30.0f, 1.0f, 5.0f, 10.0f, -5.0f, R3(256/2,256/2,256/2));

	Mat m = pc_registerPCA(p1, p2, 256);
	p1.transform_set(m);

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
