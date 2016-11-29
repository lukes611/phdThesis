#include <iostream>
#include <string>
#include <vector>
#include <stack>
#include <functional>

#include "code/basics/locv3.h"
#include "code/basics/R3.h"
#include "code/basics/llCamera.h"
#include "code/basics/LTimer.h"
#include "code/basics/Pixel3DSet.h"
#include "code/basics/VMatF.h"
#include "code/phd/measurements.h"
#include "code/phd/Licp.h"
#include "code/script/LScript.h"
#include "code/phd/fmRansac.h"
#include "code/phd/LSift.h"
//#include "code/pc/TheVolumePhaseCorrelator.h"
//#include "code/phd/Lpcr.h"
//#include "code/phd/experiments.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
//using namespace ll_experiments;
using namespace ll_siobj;
using namespace LukeLincoln;



#include "code/basics/BitReaderWriter.h"



int main(int argc, char * * argv)
{

    

    double R = 25.0, S = 1.0;
    Point2d T(0.0, 0.0);

    Mat A, B, a, b;

    A = imread("c:/lcppdata/ims/lena.png");


    cvtColor(A, a, cv::COLOR_BGR2GRAY); ll_UCF1_to_32F1(a);

	testFeatures(a, R, S, T);
	testMatches(a, R, S, T, true, 50);

    ll_transform_image(a, b, R, S, T.x, T.y);
    ll_transform_image(A, B, R, S, T.x, T.y);

	Mat M = LukeLincoln::lukes_siftRegister(a, b, true, 50, 3.0);
	ll_transform_image(a, a, M);
	imshow("reg", a + b); waitKey();


	return 0;
}




