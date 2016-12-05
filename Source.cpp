#include <iostream>
#include <string>
#include <vector>

//#define USINGGPU

#include "code/basics/locv3.h"


#include "code/basics/R3.h"
#include "code/basics/llCamera.h"
#include "code/basics/VMatF.h"
#include "code/basics/LTimer.h"
#include "code/phd/Licp.h"
#include "code/phd/measurements.h"
#include "code/phd/fmRansac.h"
#include "code/pc/TheVolumePhaseCorrelator.h"
#ifdef USINGGPU
#include "code/phd/Lpcr.h"
#endif
#include "code/script/LScript.h"
#include "code/phd/experiments.h"
#include "code/phd/LSift.h"

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

plan: experiment api:
algorithm name
data name
vector<int> frameIndexes

write:
input:
algorithm
data
description
vector<indexes> frames
error added
output:
alg, data, description, error added
errors, seconds per experiment


writes error to csv format file: data name, algorithm errorx, error y, error z times. description

V1.0
filename:
data_name.algorithm_name
version number,
data name
description
frame number
frame number 2
amount of error added
errors...




*/


#include "code/basics/BitReaderWriter.h"





void exp1(string name, vector<int> frames)
{
	//CapturePixel3DSet video(name, 10);
    CapturePixel3DSet video = CapturePixel3DSet::openCustom("/home/luke/lcppdata/pix3dc/films", name, 1);

	LukeLincoln::LVol<Vec3b> output(64, 64, 64, Vec3b(0,0,0));

    output(20,20,20) = Vec3b(255, 0,0);

	Mat accMatrix = Mat::eye(Size(4, 4), CV_32FC1);
	Pix3D frame1, frame2;
	Pixel3DSet a, b;

	//add first frame to the output
	video.read_frame(frame1, frames[0]);
	{
        Pixel3DSet _frame1(frame1);
        output.add(_frame1.points, _frame1.colors);
	}


	for (int _i = 1; _i < frames.size(); _i++)
	{
		int currentIndex = frames[_i];
		//get match _m from i+1 to i
		double seconds = 0.0, error = 0.0; int iters = 0;
		Pixel3DSet _;

		video.read_frame(frame2, currentIndex);

		a = frame1; b = frame2;

		Mat _m = Mat::eye(Size(4, 4), CV_32FC1);

		cout << "matching " << currentIndex << " with " << frames[_i - 1] << " ";
		//cout << "worked " << ll_fmrsc::registerPix3D("surf", frame2, frame1, _m, seconds, true, 50) << endl;
		//_m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
		//_m = ll_pc::pc_register_pca_i(b, a, seconds);
		//_m = ll_pc::pc_register(b, a, seconds);
		//_m = ll_pc::pc_register_pca(b, a, seconds, true, 256);
		//_m.convertTo(_m, CV_32FC1);
		_m = Licp::icp(b, a, _, error, seconds, iters);

		//_m = ll_pca::register_pca(b, a, seconds, 256);

		//_m.convertTo(_m, CV_32FC1);
		//cout << "error: " << error << endl;


		//b.transform_set(_m);

		//optionally measure the error here

		//if (error >= 1.0) continue;
		//m = m * _m : either is fine
		accMatrix = accMatrix * _m;
		//accMatrix = _m * accMatrix;
		//multiply i by m and add to output
		//cout << nm.size() << " -> " << ll_type(nm.type()) << endl;
		//double hde, msee, pme;

		b.transform_set(accMatrix);

		double hde, msee, pme;
		LTimer ___; ___.start();
		ll_measure::error_metrics(a, b, hde, msee, pme);
		___.stop(); cout << "measuring error takes" << ___.getSeconds() << endl;

		cout << hde << " " << msee << " " << pme << endl;


		cout << "adding " << output.add(b.points, b.colors) << " out of " << b.points.size() << endl;;

		frame1 = frame2;
	}
	//cout << "saving" << endl;

	Pixel3DSet obj = LukeLincoln::makePixel3DSet(output);
	obj.save_obj("/home/luke/Desktop/one.obj");

	#ifdef USINGGPU
	LLPointers::setPtr("object", &output);
    ll_experiments::viewPixel3DSet();
    #endif


}

void saveV10(string data_name, string alg_name, string desc, int frame1, int frame2, float errorAdded, float seconds, float mse, float pm, float hd)
{
    //save output to file
    cout << "saving..." << endl;
    #ifdef _WIN32
    string outDirName = "";
    #else
    string outDirName = "/home/luke/gitProjects/phdThesis/experiments";
    #endif

    stringstream outFn; outFn << outDirName << "/" << data_name << "." << alg_name << ".csv";
    string header = "data name, algorithm name, description, frame-index 1, frame-index 2, error-added, seconds, mse, percent match, hausdorff distance";
    string fileName = outFn.str();
    stringstream outData;
    outData <<
    data_name << "," <<
    alg_name << "," <<
    desc << "," <<
    frame1 << "," <<
    frame2 << "," <<
    errorAdded << "," <<
    seconds << "," <<
    mse << "," <<
    pm << "," <<
    hd;

    appendData(fileName, header, outData.str());



}

void quantitativeExperiment10(string algorithm_name,
	string data_name,
	string description,
	vector<int> frames,
	float error_added)
{
	CapturePixel3DSet video(data_name, 1);
	Pix3D frame1, frame2;
	Pixel3DSet a, b;

	//vector<double> times, errors, mses, pmes; //times, errors, mse errors, percent matches

											  //add first frame to the output
	video.read_frame(frame1, frames[0]);

	for (int _i = 1; _i < frames.size(); _i++)
	{
		int currentIndex = frames[_i];
		//get match _m from i+1 to i
		double seconds = 0.0;
		double hde, msee, pme;
		int iters = 0;
		Pixel3DSet _;

		video.read_frame(frame2, currentIndex);

		a = frame1; b = frame2;

		Mat _m = Mat::eye(Size(4, 4), CV_32FC1);

		//algorithms here:
		if (algorithm_name == "none") {
		}
#ifdef USINGGPU
		else if (algorithm_name == "pc") {
			_m = ll_pc::pc_register(b, a, seconds);
		}

		else if (algorithm_name == "pc2") {
			_m = ll_pc::pc_register_pca_i(b, a, seconds);
		}
#endif
		else if (algorithm_name == "icp") {
			_m = Licp::icp(b, a, _, hde, seconds, iters);
		}

		else if (algorithm_name == "icp2") {
			_m = Licp::icp_outlierRemoval(b, a, _, hde, seconds, iters, 10.0);
		}

		else if (algorithm_name == "fm") {
			ll_fmrsc::registerPix3D("surf", frame2, frame1, _m, seconds, true, 150);
		}

		//:end

		//compute the errors
		b.transform_set(_m);
		ll_measure::error_metrics(a, b, hde, msee, pme);

		//append data to lists
		//pmes.push_back(pme);
		//mses.push_back(msee);
		//errors.push_back(hde);
		//times.push_back(seconds);
		saveV10(data_name, algorithm_name, description, frames[_i], frames[_i-1], error_added, seconds, msee, pme, hde);


		frame1 = frame2;
	}




}



int main(int argc, char * * argv)
{
	exp1("Apartment.Texture.rotate", ll_experiments::rng(0, 4, 2));







	return 0;
}
