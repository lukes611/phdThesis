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
#include "code\phd\Lpcr.h"

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











int main(int argc, char * * argv)
{
	CapturePixel3DSet cap("Apartment.Texture.rotate", 8);

	Pixel3DSet p1, p2, p3;

	Pix3D p1_, p2_;
	cap.read_frame(p1_, 0);
	cap.read_frame(p2_, 6);
	
	p1 = Pixel3DSet(p1_);
	p2 = Pixel3DSet(p2_);


	//p2 = p1.clone();
	p3 = p1.clone();

	//p2.transform_set(0.0f, 30.0f, 0.0f, 1.0f, 5.0f, 10.0f, -5.0f, R3(256/2,256/2,256/2));
	double s;
	//Mat m = ll_pc::pc_register(p1, p2, s);
	int its; double eo; Pixel3DSet o;
	//Mat m = Licp::icp(p1, p2, o, eo, s, its);
	Mat m;ll_fmrsc::registerPix3D("surf", p1_, p2_, m, s);
	m.convertTo(m, CV_32FC1);
	p1.transform_set(m);

	cout << "seconds: " << s << endl;
	//Mat m2 = pc_registerPCA(p1, p2);

	//p1.transform_set(m2);

	
	Pixel3DSet t1 = p3 + p2;
	Pixel3DSet t2 = p1 + p2;
	Mat a, b;
	cout << "rendering\n";
	Pix3D A = render(t1, 256, 500.0f); cout << "r1";
	Pix3D B = render(t2, 256, 500.0f); cout << "r2\n";
	
	A.colorImage(a);
	B.colorImage(b);
	
	imshow("before", a);
	imshow("after", b);

	waitKey(90);
	cout << p1.size() << " : " << p2.size() << " : " << p3.size() << endl;
	cout << "computing avg: " << endl;
	cout << "before/after avg: " << avg(p3, p2);

	

	cout << " / " << avg(p1, p2) << endl;
	//cout << "before/after % m: " << percentMatch(p3, p2, 2.0f) << " / " << percentMatch(p1, p2, 2.0f) << endl;

	waitKey();
	return 0;
}
