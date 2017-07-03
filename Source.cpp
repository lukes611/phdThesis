#include "code/phd/experiments.h"
#ifdef HASGL
#include "code/basics/ll_gl.h"
#endif
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


v2.0
rules: save name of file as [data-name].[versionNo].csv

headers:
algorithm name, frame-index-1, frame-index-2, seconds, error (hausdorff-distance)

toJSON function ->
{
    results : [
        {
            dataset : {
                algorithm : [
                    name : string,
                    errors : array,
                    seconds : array
                ]
            }
        }
    ]
}

*/
#define HASFFTW


#include "code/basics/BitReaderWriter.h"






//single experiment, only tests one algorithm at a time
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


//saves to version 2.0 data file
void saveV20(string data_name, string alg_name, int frame1, int frame2, float seconds, float hd)
{
    //save output to file
    cout << "saving v2.0..." << endl;
    string outDirName = EXPS_DIR;
    //save name of file as [data-name].[versionNo].csv
    stringstream outFn; outFn << outDirName << "/" << data_name << ".v2.csv";
    //header: algorithm name, frame-index-1, frame-index-2, seconds, error (hausdorff-distance)
    string header = "alg,frame1,frame2,seconds,hd";
    string fileName = outFn.str();
    stringstream outData;
    outData <<
    alg_name << "," <<
    frame1 << "," <<
    frame2 << "," <<
    seconds << "," <<
    hd;

    cout << outData.str() << endl;

    appendData(fileName, header, outData.str());



}


void quantitativeExperiment20(string algorithm_name, string data_name, vector<int> frames)
{
    CapturePixel3DSet video = ll_experiments::openData(data_name, 1);

	Pix3D frame1, frame2;
	Pixel3DSet a, b;

	video.read_frame(frame1, frames[0]);

	for (int _i = 1; _i < frames.size(); _i++)
	{
		int currentIndex = frames[_i];
		//get match _m from i+1 to i
		double seconds = 0.0;
		double hde;
		int iters = 0;
		Pixel3DSet _;


		cout << algorithm_name << ", completed " << _i << " of " << frames.size() << endl;

		video.read_frame(frame2, currentIndex);

		a = frame1; b = frame2;

		Mat _m = Mat::eye(Size(4, 4), CV_32FC1);

		//algorithms here:
		if (algorithm_name == "none") {
            //do-nothing
		}
#if defined(HASCUDA) || defined(HASFFTW)
		else if (algorithm_name == "FVR") {
			_m = ll_pc::pc_register(b, a, seconds);
		}

		else if (algorithm_name == "FVR3D") {
			_m = ll_pc::pc_register_pca_i(b, a, seconds);
		}
		else if (algorithm_name == "FVR3D-2") {
			_m = ll_pc::pc_pca_icp(b, a, seconds);
		}else if (algorithm_name == "FFVR"){
            _m = ll_pc::ffvr(b, a, seconds);
		}
#endif
        else if(algorithm_name == "PCA"){
            _m = ll_pca::register_pca(b, a, seconds);
        }else if (algorithm_name == "ICP") {
			_m = Licp::icp(b, a, _, hde, seconds, iters);
		}
		else if (algorithm_name == "FM2D") {
			ll_fmrsc::registerPix3D("surf", frame2, frame1, _m, seconds, true, 150);
		}else if(algorithm_name == "FM3D"){
		    _m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
		}



		//:end

		//compute the errors
		b.transform_set(_m);
		//hde = ll_measure::hausdorff(a, b);
		double msee, pme;
		hde = 21.0;
		ll_measure::error_metrics(a, b, hde, msee, pme);

		//void saveV20(string data_name, string alg_name, int frame1, int frame2, float seconds, float hd)
		saveV20(data_name, algorithm_name, frames[_i], frames[_i-1], seconds, hde);


		frame1 = frame2;
	}




}


//saves to kitti data file
void saveKitti(string data_name, string alg_name, int frame1, int frame2, float seconds, float hd)
{
    //save output to file
    cout << "saving v2.0..." << endl;
    string outDirName = EXPS_DIR;
    //save name of file as [data-name].[versionNo].csv
    stringstream outFn; outFn << outDirName << "/" << data_name << ".kitti.v2.csv";
    //header: algorithm name, frame-index-1, frame-index-2, seconds, error (hausdorff-distance)
    string header = "alg,frame1,frame2,seconds,hd";
    string fileName = outFn.str();
    stringstream outData;
    outData <<
    alg_name << "," <<
    frame1 << "," <<
    frame2 << "," <<
    seconds << "," <<
    hd;

    cout << outData.str() << endl;

    appendData(fileName, header, outData.str());



}



void quantitativeExperimentKitti10(string algorithm_name, string data_name, vector<int> frames)
{
    kitti::KittiPix3dSet frame1, frame2;
	Pixel3DSet a, b;

    frame1 = kitti::open(data_name, frames[0]);

	for (int _i = 1; _i < frames.size(); _i++)
	{
		int currentIndex = frames[_i];
		//get match _m from i+1 to i
		double seconds = 0.0;
		double hde;
		int iters = 0;
		Pixel3DSet _;


		cout << algorithm_name << ", completed " << _i << " of " << frames.size() << endl;

		frame2 = kitti::open(data_name, currentIndex);


		a = frame1.getPoints(); b = frame2.getPoints();

		Mat _m = Mat::eye(Size(4, 4), CV_32FC1);

		//algorithms here:
		if (algorithm_name == "none") {
            //do-nothing
		}
#if defined(HASCUDA) || defined(HASFFTW)
		else if (algorithm_name == "FVR") {
			_m = ll_pc::pc_register(b, a, seconds);
		}

		else if (algorithm_name == "FVR3D") {
			_m = ll_pc::pc_register_pca_i(b, a, seconds);
		}
		else if (algorithm_name == "FVR3D-2") {
			_m = ll_pc::pc_pca_icp(b, a, seconds);
		}else if (algorithm_name == "FFVR"){
            _m = ll_pc::ffvr(b, a, seconds);
		}
#endif
        else if(algorithm_name == "PCA"){
            _m = ll_pca::register_pca(b, a, seconds);
        }else if (algorithm_name == "ICP") {
			_m = Licp::icp(b, a, _, hde, seconds, iters);
		}
		else if (algorithm_name == "FM2D") {
			ll_fmrsc::registerPix3D("surf", frame2, frame1, _m, seconds, true, -1);
		}else if(algorithm_name == "FM3D"){
		    _m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
		}



		//:end

		//compute the errors
		b.transform_set(_m);
		//hde = ll_measure::hausdorff(a, b);
		double msee, pme;
		hde = 21.0;
		ll_measure::error_metrics(a, b, hde, msee, pme);

		//void saveV20(string data_name, string alg_name, int frame1, int frame2, float seconds, float hd)
		saveKitti(data_name, algorithm_name, frames[_i], frames[_i-1], seconds, hde);


		frame1 = frame2;
	}




}


string kittiData = "2011_09_26_drive_0001_sync";



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

	//testSetPix3d(namesList[0]);
	//test("Apartment.Texture.rotate", Point3d(5.0f, 2.0f, 0.0f), 1.0f, Point3d(0.0, 1.0, 8.0));

	//for(int i = 13; i < 20; i++)
	{
	string fn = namesList[6];
	//exp1("Apartment.Texture.rotate", ll_experiments::rng(15, 20, 1));

    quantitativeExperiment20("none", fn, inds);
    quantitativeExperiment20("FM2D", fn, inds);
    quantitativeExperiment20("FM3D", fn, inds);
    quantitativeExperiment20("ICP", fn, inds);
    quantitativeExperiment20("PCA", fn, inds);
    quantitativeExperiment20("FVR", fn, inds);
    quantitativeExperiment20("FVR3D", fn, inds);
    quantitativeExperiment20("FVR3D-2", fn, inds);
	quantitativeExperiment20("FFVR", fn, inds);


    }

    /*
	to-do:
		2. create experiments 3.0 []
            feature matching 2d []
            feature matching 3d []
            fvr []
            fvr3d []
            fvr3d-2 []
            ffvr []
            icp []
            PCA []

		3. run experiments 3.0
		4. setup the other test experiments
		5. add to the thesis
		6. finalize thesis experiments section
		7. re-write the section also
			add the r-table required
		8. re-write intro.conclusion.methodology (somewhat)


	*/

	inds = ll_experiments::rng(0, 107, 1);
    //quantitativeExperimentKitti10("none", kittiData, inds);
    //quantitativeExperimentKitti10("FM2D", kittiData, inds);
    //quantitativeExperimentKitti10("FM3D", kittiData, inds);
    //quantitativeExperimentKitti10("ICP", kittiData, inds);
    //quantitativeExperimentKitti10("PCA", kittiData, inds);
    //quantitativeExperimentKitti10("FVR", kittiData, inds);
    //quantitativeExperimentKitti10("FVR3D", kittiData, inds);
    //quantitativeExperimentKitti10("FVR3D-2", kittiData, inds);
	//quantitativeExperimentKitti10("FFVR", kittiData, inds);



	return 0;
}
