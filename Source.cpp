#include <string>
#include <iostream>
#include "code/basics/SIObj.h"
#include "code/basics/locv3.h"
#include "code/basics/llCamera.h"
#include "code/phd/experiments.h"
#include "code/basics/VMatF.h"
#include "code/basics/locv_algorithms.h"
#include "code/pc/TheVolumePhaseCorrelator.h"
#include "code/phd/Lpcr.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_siobj;
using namespace ll_experiments;



int main(){


	ll_pix3d::CapturePixel3DSet reader = ll_experiments::openData("Apartment.Texture.rotate", 3);

	Pixel3DSet p1;
	Pixel3DSet p2;

	{
		Pix3D p;
		reader.read_frame(p, 0);
		p1 = p;
	}
	{
		Pix3D p;
		reader.read_frame(p, 10);
		p2 = p;
	}





	return 0;
}

