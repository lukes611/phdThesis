#include <iostream>
#include <string>
#include <vector>



#include "code/basics/locv3.h"
#include "code/phd/experiments.h"

#include "code/basics/R3.h"
#include "code/basics/llCamera.h"
#include "code/basics/VMatF.h"
#include "code/basics/LTimer.h"
#include "code/phd/Licp.h"
#include "code/phd/measurements.h"
#include "code/phd/fmRansac.h"
#include "code/pc/TheVolumePhaseCorrelator.h"
#include "code/phd/Lpcr.h"
#include "code/script/LScript.h"

#include "code/phd/LSift.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
using namespace ll_experiments;

/*

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


void test(string name, Point3d rotation, float scale, Point3d translation, double noiseRange = 0.0, bool view = false)
{
	cout << "**************\n";
	cout << "rotation: " << rotation << ", scale: " << scale << ", translation: " << translation << endl;

	CapturePixel3DSet video = ll_experiments::openData(name, 1);
	Mat M = VMat::transformation_matrix(256, rotation.x, rotation.y, rotation.z, scale, translation.x, translation.y, translation.z);

	//output structure
	LukeLincoln::LVol<Vec3b> output(64, 64, 64, Vec3b(0, 0, 0));
	//read in frame
	Pix3D frame1, frame2; video.read_frame(frame1, 0);
	Mat SCALAR_MAT = VMat::transformation_matrix(256, 0.0f, 0.0f, 0.0f, 384.0f / 256.0f, 0.0f, 0.0f, 0.0f);
	for (int i = 0; i < frame1.count; i++)
	{
		Point3f p = LukeLincoln::operator*(SCALAR_MAT, Point3f(frame1.points[i].x, frame1.points[i].y, frame1.points[i].z));
		frame1.points[i] = R3(p.x, p.y, p.z);
	}
	frame2 = frame1;
	//transform second frame
	for (int i = 0; i < frame2.count; i++)
	{
		Point3f p = LukeLincoln::operator*(M, Point3f(frame2.points[i].x, frame2.points[i].y, frame2.points[i].z));
		frame2.points[i] = R3(p.x, p.y, p.z);
	}

	vector<string> algorithms;
	//algorithms.push_back("none");
	//algorithms.push_back("fm");
	//algorithms.push_back("fm3d");
	//algorithms.push_back("icp");
	algorithms.push_back("pc");
	algorithms.push_back("pc2");
	algorithms.push_back("pc3");
	//algorithms.push_back("pca");



	//for each algorithm
	for (int i = 0; i < algorithms.size(); i++)
	{
		//register
		Mat M2 = Mat::eye(Size(4, 4), CV_32FC1);

		double snr1, snr2, snrOut;

		Pix3D frame1Noise = ll_experiments::getNoisedVersion(frame1, noiseRange, snr1);
		Pix3D frame2Noise = ll_experiments::getNoisedVersion(frame2, noiseRange, snr2);
		snrOut = (snr1 + snr2) * 0.5f;

		Pixel3DSet f1 = frame1Noise;
		Pixel3DSet f2 = frame2Noise;

		

		double seconds;
		if (algorithms[i] == "fm")
			ll_fmrsc::registerPix3D("", frame1Noise, frame2Noise, M2, seconds, true, 50);
		else if (algorithms[i] == "icp")
		{
			Pixel3DSet out;
			int iters;
			double errorOut;
			M2 = Licp::icp(f1, f2, out, errorOut, seconds, iters, 0.2, 150);
		}
		else if (algorithms[i] == "fm3d")
		{
			M2 = LukeLincoln::sift3DRegister(f1, f2, seconds, true, 256);
		}
		else if (algorithms[i] == "pca")
		{
			M2 = ll_pca::register_pca(f1, f2, seconds, 256);
		}
#ifdef HASCUDA
		else if(algorithms[i] == "pc")
		{
			M2 = ll_pc::pc_register(f1, f2, seconds);
		}else if(algorithms[i] == "pc2")
		{
			M2 = ll_pc::pc_register_pca(f1, f2, seconds);
		}else if(algorithms[i] == "pc3")
		{
			M2 = ll_pc::pc_pca_icp(f1, f2, seconds);
		}
#endif


		//measure error
		double count = 0.0, numMatches = 0.0;
		for (int i = 0; i < frame1.count; i++)
		{
			Point3f _ = LukeLincoln::operator*(M2, Point3f(frame1.points[i].x, frame1.points[i].y, frame1.points[i].z));
			R3 p(_.x, _.y, _.z);
			float D = p.dist(frame2.points[i]);
			if (D < 3.0f) numMatches += 1.0;
			count += 1.0;
		}

		double percentMatch = 100.0 * (numMatches / count);
		stringstream line; line << algorithms[i] << ", " << rotation.x << ", " << rotation.y << ", " << rotation.z << ", " <<
			scale << ", " << translation.x << ", " << translation.y << ", " << translation.z << ", " << 
			noiseRange << ", " << snrOut << ", "
			<< percentMatch << ", " << seconds;

		//print error
		cout << "algorithm[" << algorithms[i] << "] had " << percentMatch << "% matching rate" << endl;
		ll_experiments::appendData(string(EXPS_DIR) + string("/") + name + string(".tests.v2.csv"),
			string("algorithm, rx, ry, rz, scale, tx, ty, tz, noiseRange, SNR, percent match, seconds"),
			line.str());

#ifdef HASGL
		if(algorithms[i] == "="){
			f1.transform_set(M2);
			f1 += f2;
			LLPointers::setPtr("object", &f1);
			ll_experiments::viewPixel3DSet();
		}
#endif
	}










}

void testSetPix3d(string name)
{
<<<<<<< HEAD
	double nr = 3.0;
	for(int Y = 330; Y < 360; Y+=10){
=======
	double nr = 2.0;
	for(int Y = 0; Y < 360; Y+=10){
>>>>>>> 3c9fa0ed7bbcf9756ff6f761ac182de0b57513b1
		test(name, Point3d(0.0, Y, 0.0), 1.0f, Point3d(0.0, 0.0, 0.0), nr);

	}
	for(int x = 0; x < 160; x+=10)
		test(name, Point3d(0.0, 0.0, 0.0), 1.0f, Point3d(x, 0.0, 0.0), nr);
	for(double S = 0.9; S <= 1.2; S += 0.5)
		test(name, Point3d(0.0, 0.0, 0.0), S, Point3d(0.0, 0.0, 0.0), nr);

	for(int y = 0; y < 30; y+=10)
	{
		for(int x = 0; x < 30; x+=10)
			for(int z = 0; z < 30; z += 10)
				test(name, Point3d(x, y, z), 1.0f, Point3d(10.0, 5.0, 2.0), nr);
	}

}


void exp1(string name, vector<int> frames)
{

	CapturePixel3DSet video = ll_experiments::openData(name, 1);

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
		_m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
		//_m = ll_pc::pc_register_pca_i(b, a, seconds);
		//_m = ll_pc::pc_register(b, a, seconds);
		//_m = ll_pc::pc_register_pca(b, a, seconds, true, 256);
		//_m.convertTo(_m, CV_32FC1);
		//_m = Licp::icp(b, a, _, error, seconds, iters);

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
	obj.save_obj(string(DESKTOP_DIR) + "/one.obj");

	#ifdef HASGL
	//LLPointers::setPtr("object", &obj);
    //ll_experiments::viewPixel3DSet();
    #endif


}

void saveV10(string data_name, string alg_name, string desc, int frame1, int frame2, float errorAdded, float seconds, float mse, float pm, float hd)
{
    //save output to file
    cout << "saving..." << endl;
    string outDirName = EXPS_DIR;

    stringstream outFn; outFn << outDirName << "/" << data_name << ".v1.0.csv";
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

    cout << outData.str() << endl;

    appendData(fileName, header, outData.str());



}

void quantitativeExperiment10(string algorithm_name,
	string data_name,
	string description,
	vector<int> frames,
	float error_added)
{
	#ifdef _WIN32
	CapturePixel3DSet video(data_name, 1);
#else
    CapturePixel3DSet video = CapturePixel3DSet::openCustom("/home/luke/lcppdata/pix3dc/films", data_name, 1);
#endif

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

		cout << algorithm_name << " : " << _i << " / " << frames.size() << endl;

		video.read_frame(frame2, currentIndex);

		a = frame1; b = frame2;

		Mat _m = Mat::eye(Size(4, 4), CV_32FC1);

		//algorithms here:
		if (algorithm_name == "none") {
		}
#ifdef HASCUDA
		else if (algorithm_name == "pc") {
			_m = ll_pc::pc_register(b, a, seconds);
		}

		else if (algorithm_name == "pc2") {
			_m = ll_pc::pc_register_pca_i(b, a, seconds);
		}
		else if (algorithm_name == "pc3") {
			_m = ll_pc::pc_pca_icp(b, a, seconds);
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
		}else if(algorithm_name == "fm3d"){
		    _m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
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

    string namesList[20] = {

        "Apartment.Texture.rotate",
        "Apartment.Texture.rotateXAxis",
        "Boxes.Texture.arbitrarycamera",
        "Boxes.Texture.rotate",
        "Boxes.Texture.zoomOut",
        "Desk.Texture.Translation",
        "IndoorSpace.tc.translation",
        "Kitchen.littleTexture.pan",
        "Kitchen.littleTexture.zoom",
        "OfficeDesk.Texture.rotationLift",
        //"office.move1cm",
        "Office.Texture.blindSpotRotation",
        "Office.TexturedItems.Translation",
        "Office.Texture.rotation",
        "Office.Texture.rotationXAxis",
        "Office.Texture.Translation",
        "Outside.NoTexture.rotation",
        "Outside.NoTexture.translation",
        "Outside.TextureConfusion.rotation",
        "Outside.TextureConfusion.Translation",
        "PlantsOutdoors.tc.rotation"
    };

    //string fn = "PlantsOutdoors.tc.rotation";
    int start = 4, to = 30, inc = 1;
    vector<int> inds = ll_experiments::rng(start, to, inc);


	//save images: 4 per video file
<<<<<<< HEAD
	/*for(int videoI = 0; videoI < 20; videoI++)
	{
		CapturePixel3DSet video = openData(namesList[videoI], 1);
		for(int i = 0; i < 4; i++)
		{
			int j = 4 + i * 6;
			Pix3D frame;
			video.read_frame(frame, j);
			Mat image;
			frame.colorImage(image);
			//imshow("image", image);
			//waitKey(100);
			stringstream file_name;
			file_name << "C:/Users/luke/Documents/Visual Studio 2012/Projects/PhD 16 Sem2/PhD 16 Sem2/thesis/images/experiments/"
				<< "test_data/" << namesList[videoI] << "." << i << ".png";
			cv::resize(image, image, Size(640/2, 480/2));
			cv::imwrite(file_name.str(), image);
			
			//cout << file_name.str() << endl;
		}
	}*/
=======
	
>>>>>>> 3c9fa0ed7bbcf9756ff6f761ac182de0b57513b1

	testSetPix3d(namesList[0]);
	//test("Apartment.Texture.rotate", Point3d(5.0f, 2.0f, 0.0f), 1.0f, Point3d(0.0, 1.0, 8.0));

	//for(int i = 13; i < 20; i++)
	{
	//string fn = namesList[i];
	//exp1("Apartment.Texture.rotate", ll_experiments::rng(15, 20, 1));
	
    //quantitativeExperiment10("none", fn, "regular", inds,0.0f);
    //quantitativeExperiment10("fm", fn, "regular", inds,0.0f);
    //quantitativeExperiment10("fm3d", fn, "regular", inds,0.0f);
    //quantitativeExperiment10("icp", fn, "regular", inds,0.0f);
    //quantitativeExperiment10("icp2", fn, "regular", inds,0.0f);
    //quantitativeExperiment10("pc", fn, "regular", inds,0.0f);
    //quantitativeExperiment10("pc2", fn, "regular", inds,0.0f);
    //quantitativeExperiment10("pca", fn, "regular", inds,0.0f);
	//quantitativeExperiment10("pc3", fn, "regular", inds,0.0f);

    }




	return 0;
}
