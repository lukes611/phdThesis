#include "LPhDHelper.h"
#include "../basics/llCamera.h"
using namespace ll_cam;


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

/**
 * @desc - added on 12.9.2017
 */
void qualitativeExperiment(string algorithm_name, string data_name, vector<int> frames)
{
    CapturePixel3DSet video = ll_experiments::openData(data_name, 1);


	Pix3D frame1, frame2;
	Pixel3DSet a, b;

	LukeLincoln::LVol<Vec3b> output(512, 512, 512, Vec3b(0,0,0));
	cout << output.resolution << " was res" << endl;
	output.resolution = 0.5;

    //output(20,20,20) = Vec3b(255, 0,0);

	Mat accMatrix = Mat::eye(Size(4, 4), CV_32FC1);
	

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


		accMatrix = accMatrix * _m;
		b.transform_set(accMatrix);
		

		cout << "adding " << output.add(b.points, b.colors) << " out of " << b.points.size() << endl;;

		frame1 = frame2;
	}


	Pixel3DSet obj = LukeLincoln::makePixel3DSet(output);
	//obj.save_obj(string(DESKTOP_DIR) + "/one.obj");
	Fps_cam cam(R3(-25.6504, 70, -39.8557) , 54.000000, 98.000000);

	#ifdef HASGL
	LLPointers::setPtr("object", &obj);
    ll_experiments::viewPixel3DSet(cam);
    #endif


}
