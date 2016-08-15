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
	CapturePixel3DSet cap("Apartment.Texture.rotate", 16);

	Pix3D p1, p2;

	cap.read_frame(p1, 8);

	cap.read_frame(p2, 5);

	cout << cap.size() << endl;
	Mat matrix;
	cout << ll_fmrsc::registerPix3D("surf", p1, p2, matrix, true, 80) << endl;
	matrix.convertTo(matrix, CV_32FC1);
	Pixel3DSet A(p1), B(p2);
	Pixel3DSet C = A + B;
	SIObj(C.points).saveOBJ("c:/lcppdata/obj/bad.obj");
	A.transform_set(matrix);
	A += B;

	SIObj(A.points).saveOBJ("c:/lcppdata/obj/reg.obj");
	//A.normalize(256);

	Pix3D X = render(A, 384, 500.0f);
	Mat x;
	X.colorImage(x);
	imshow("x", x);
	waitKey();



	return 0;
}
