#include <string>
#include <iostream>
#include "code/basics/SIObj.h"
#include "code/basics/locv3.h"
#include "code/basics/llCamera.h"
#include "code/phd/experiments.h"
#include "code/basics/VMatF.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_siobj;



int main(){


	string fn = string(LCPPDATA_DIR) + string("/obj/") + "bunny_simplified2.obj";

	SIObj obIn;
	obIn.open_obj(fn);
	obIn.normalize(256.0f);
	vector<R3> points;
	for (int i = 0; i < obIn._triangles.size(); i++) obIn.getTriangle(i).rasterize(points);
	

	VMat ob(256, points, 0.0f);

	Mat im = ob.luke_dimension_reduce_2a(0.005f, 2, Size(256, 256));
	ll_normalize(im);
	//cout << ll_type(im.type()) << endl;
	//imshow("im1", im);
	//waitKey();

	ll_32F1_to_UCF1(im);
	ll_transform_image(im, im, 180.0, 1.0, 0, 0);

	imwrite(
"C:/Users/s2807774/Documents/Visual Studio 2015/Projects/PhD project 16/Ocv3 Basic with ll_ libraries/thesis/images/methodology/FVR/zaxis.png", im);

	

	cout << obIn._points.size() << endl;


    return 0;
}

