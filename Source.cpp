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
	SIObj ob;
    ob.open_obj("/home/luke/lcppdata/obj/bunny_simplified2.obj");


    VMat v2 = ob;
    VMat g = getGaussianImage(3, 3.0);

    LTimer t; t.start();
    v2.filter(g);
    t.stop(); cout << t.getSeconds() << endl;


	v2.save_obj("/home/luke/Desktop/a.obj", 256, 0.2f);


	return 0;
}




