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

using namespace std;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
//using namespace cv;

//proto-type for experiments: VMat pc(VMat a, VMat b, double & time, Mat & transform, double & mse);
//to-do:= pc, ipc, 

/*
read in 1 x Pixel3DSets
mutate points slightly
get knn working with 4d data

*/






int pointMatch(string algorithm, Pix3D & object1, Pix3D & object2, vector<Point2i> & p1, vector<Point2i> & p2, bool sort = false, int top = -1)
{
	p1.clear(); p2.clear();
	vector<Point2i> pts1, pts2;
	Mat colA, colB, vdA, vdB;
	object1.colorImage(colA); object2.colorImage(colB);
	object1.vdImage(vdA); object2.vdImage(vdB);
	if(algorithm == "sift")
		ll_sift(colA, colB, pts1, pts2, sort, top);
	else //surf
		ll_surf(colA, colB, pts1, pts2, sort, top);
	for(int i = 0; i < pts1.size() && i < pts2.size(); i++)
	{
		if(vdA.at<unsigned char>(pts1[i]) == 0xFF && vdB.at<unsigned char>(pts2[i]))
		{
			p1.push_back(pts1[i]);
			p2.push_back(pts2[i]);
		}
	}
	return p1.size();
}



int main(int argc, char * * argv)
{
	CapturePixel3DSet cap("Apartment.Texture.rotate", 10);

	Pix3D p1, p2;

	cap.read(p1);

	cap.read_frame(p2, 20);

	cout << cap.size() << endl;

	Mat a, b;

	p1.colorImage(a); p2.colorImage(b);

	imshow("a" , a);
	imshow("b" , b);

	vector<Point2i> points1, points2;
	pointMatch("surf", p1, p2, points1, points2, true, 50);

	Mat z = ll_view_matches(a, b, points1, points2);

	//get point-matches for pix3d
	//filter these by vd
	//

	imshow("z", z);

	waitKey();
	
	return 0;
}
