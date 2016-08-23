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
#include "code\phd\experiments.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
using namespace ll_experiments;
//using namespace cv;

//proto-type for experiments: VMat pc(VMat a, VMat b, double & time, Mat & transform, double & mse);
//to-do:= pc, ipc, 

/*
read in 1 x Pixel3DSets
mutate points slightly
get knn working with 4d data

string askPython(string data)

PixCompressor api:

range representation __from __to
from -256 + 512


*/


#include "code\basics\BitReaderWriter.h"



void exp1(string name, vector<int> frames)
{
	CapturePixel3DSet video(name, 10);

	Pixel3DSet output;

	Mat accMatrix = Mat::eye(Size(4,4), CV_32FC1);
	Pix3D frame1, frame2;
	Pixel3DSet a, b;

	//add first frame to the output
	video.read_frame(frame1, frames[0]);
	output += Pixel3DSet(frame1);

	for(int _i = 1; _i < frames.size(); _i++)
	{
		int currentIndex = frames[_i];
		//get match _m from i+1 to i
		double seconds = 0.0, error = 0.0; int iters = 0;
		Pixel3DSet _;

		video.read_frame(frame2, currentIndex);

		a = frame1; b = frame2;

		Mat _m = Mat::eye(Size(4,4) , CV_32FC1);

		cout << "matching " << currentIndex << " with " << frames[_i-1] << endl;
		//cout << "worked " << ll_fmrsc::registerPix3D("surf", frame2, frame1, _m, seconds, true, 150) << endl;;
		//_m = ll_pc::pc_register_pca_i(b, a, seconds);
		//_m.convertTo(_m, CV_32FC1);
		_m = Licp::icp(b, a, _, error, seconds, iters);
		
		//_m.convertTo(_m, CV_32FC1);
		cout << "error: " << error << endl;
		

		//b.transform_set(_m);

		//optionally measure the error here

		
		//m = m * _m : either is fine
		accMatrix = accMatrix * _m;
		//accMatrix = _m * accMatrix;
		//multiply i by m and add to output
		//cout << nm.size() << " -> " << ll_type(nm.type()) << endl;
		
		b.transform_set(accMatrix);

		output += b;
		frame1 = frame2;
	}
	cout << "saving" << endl;
	//output.reduce(256);
	//SIObj(output.points).saveOBJ("C:/Users/luke/Desktop/result2.obj");
	LLPointers::setPtr("object", &output);
	ll_experiments::viewPixel3DSet();

}




int add(int a){
	if(LLPointers::has("b")) return a + LLPointers::get<int>("b");
	return a;
}



int main(int argc, char * * argv)
{
	exp1("Apartment.Texture.rotate", ll_experiments::rng(40));
	
	return 0;
}
