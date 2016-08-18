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
#include "code\script\LScript.h"

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

string askPython(string data)

*/







void viewVideo(string name, bool viewColor = true, bool viewDepth = false, bool viewVD = false, int wks = -1)
{
	CapturePixel3DSet video (name, 80);
	Pix3D p;
	Mat color, depth, vd;
	for(int i = 0; i < video.size(); i++)
	{
		video.read(p);
		
		if(viewColor)
		{
			p.colorImage(color);
			imshow("color", color);
		}

		if(viewDepth)
		{
			p.depthImage(depth);
			imshow("depth", depth);
		}

		if(viewVD)
		{
			p.vdImage(vd);
			imshow("vd",vd);
		}
		
		if(viewColor || viewDepth || viewVD)
		{
			if (wks > 0) waitKey(wks);
			else waitKey();
		}
	}
}



int main(int argc, char * * argv)
{
	//cout << ll_script::askPython("ls.py") << endl;

	string d = "C:/lcppdata/pix3dc/films";

	vector<string> files = ll_split(ll_script::askPython("ls.py -d " + d), ',');


	for(int j = 0; j < files.size(); j++)
	{
		cout << files[j] << endl;
		viewVideo(files[j], true, true, false, 30);

	}
	
	return 0;
}
