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
	//algorithms.push_back("pc");
	//algorithms.push_back("pc2");
	//algorithms.push_back("pc3");
	//algorithms.push_back("pca");
    algorithms.push_back("ffvr");


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
#if defined(HASCUDA) || defined(HASFFTW)
		else if(algorithms[i] == "pc")
		{
			M2 = ll_pc::pc_register(f1, f2, seconds);
		}else if(algorithms[i] == "pc2")
		{
			//M2 = ll_pc::pc_register_pca(f1, f2, seconds);
			ll_pc::pc_register_pca_i(f1, f2, seconds);
		}else if(algorithms[i] == "pc3")
		{
			M2 = ll_pc::pc_pca_icp(f1, f2, seconds);
		}else if(algorithms[i] == "ffvr")
		{
			M2 = ll_pc::ffvr(f1, f2, seconds);
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
		ll_experiments::appendData(string(EXPS_DIR) + string("/") + name + string(".") + algorithms[i] + string(".") + string(".tests.v3.csv"),
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


//does noise experiments
void testSetPix3d(string name)
{
	double nr = 0.0;
	//for(int Y = 0; Y < 360; Y+=10){
	//	test(name, Point3d(0.0, Y, 0.0), 1.0f, Point3d(0.0, 0.0, 0.0), nr);
	//}
	//for(int x = 0; x < 160; x+=10)
	//	test(name, Point3d(0.0, 0.0, 0.0), 1.0f, Point3d(x, 0.0, 0.0), nr);
	//for(double S = 0.9; S <= 1.2; S += 0.5)
	//	test(name, Point3d(0.0, 0.0, 0.0), S, Point3d(0.0, 0.0, 0.0), nr);
	for(int i = 0; i < 3; i++)
	for(int y = 0; y < 200; y+=30)
	{
		for(int x = 0; x < 200; x+=30)
		{
			//for(int z = 0; z < 30; z += 10)
			test(name, Point3d(x, y, 0.0f), 1.0f, Point3d(0.0, 0.0, 0.0), (double)i);
		}
	}

}


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


int countt = 1;
double pas = 0.0;
LTimer tmr;
Mat velo2cam, primary;
Pixel3DSet obj;
Mat imi;

void unproject(Pixel3DSet & o)
{
	Pixel3DSet tmp = o.clone();
	o = Pixel3DSet();
	tmp.transform_set(velo2cam);
	for (int i = 0; i < tmp.size(); i++)
	{
		if (tmp[i].z >= 0.0f)
		{
			Vec3b white(255, 255, 255);
			R3 t = tmp[i];
			t = R3(t.x, t.y, t.z);
			Pixel3DSet::transform_point(primary, t);
			t /= 1000.0;
			o.push_back(t, white);
		}

		//o[i].z = 0.0f;
		//o[i] /= 1000.0;
	}
	//cout << o.size() << endl;
}


R3 kittiProject(R3 input, Mat & matrix)
{
    Mat tmp = Mat::zeros(Size(1, 4), CV_32FC1);
    tmp.at<float>(0,0) = input.x;
    tmp.at<float>(1,0) = input.y;
    tmp.at<float>(2,0) = input.z;
    tmp.at<float>(3,0) = 1.0f;
	//cout << "in " << tmp << endl;
    tmp = matrix * tmp;
	//cout << "out " << tmp << endl;
	//system("pause");
	return R3(tmp.at<float>(0, 0), tmp.at<float>(1, 0), tmp.at<float>(2, 0) * tmp.at<float>(2, 0)) / tmp.at<float>(2, 0);
}

void unprojectNew(Pixel3DSet & o, Mat m)
{
    for(int i = 0; i < o.size(); i++)
        o[i] = kittiProject(o[i], m);
}

void unprojectNew3(Pixel3DSet & in, Mat m)
{
	Pixel3DSet o = in.clone();
	in = Pixel3DSet();
	Vec3b white(255, 255, 255);
	for (int i = 0; i < o.size(); i++)
	{
		//if (o[i].y > 5) continue;
		//if (o[i].x < -46.0) continue;
		o[i] = kittiProject(o[i], m);
		int col_idx = round(64.0 * 5.0 / o[i].y);
		//o[i].z = 0.0f;
		if(o[i].x >= 0.0f && o[i].x < 1242.0f && o[i].y > 0.0f && o[i].y < 375.0f)
			in.push_back(o[i], white);
	}
}

void unprojectNew2(Pixel3DSet & o, Mat & m)
{
	Pixel3DSet tmp = o.clone();
	o = Pixel3DSet();
	tmp.transform_set(velo2cam);
	for (int i = 0; i < tmp.size(); i++)
	{
		if (tmp[i].z >= 0.0f)
		{
			Vec3b white(255, 255, 255);
			R3 t = kittiProject(tmp[i], m);
			o.push_back(t, white);
			o[o.size()-1] /= 1000.0;
		}

		//o[i].z = 0.0f;
		
	}
	//cout << o.size() << endl;
}


//todo
/*
view video of pixel3dsets
see if I can align the pixels with the depth data


*/

#ifdef HASGL

//[570, 230, -1050] , angle_y: 88.000000, angle_x: 90.000000
Fps_cam camera(R3(570, 230, -1050), 88.0f, 90.0f);



void display()
{


	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	ll_gl::default_viewing(camera);

	ll_gl::default_lighting();
	ll_gl::turn_off_lights();


	glBegin(GL_POINTS);
	for (int i = 0; i < obj.points.size(); i++)
	{
		R3 col = obj.color_as_r3(i) / 255.0f;
		swap(col.x, col.z);
		ll_gl::set_color(col);
		ll_gl::glR3(obj.points[i]);
	}
	glEnd();


	glutSwapBuffers();
	glutPostRedisplay();
}

void kbf(unsigned char key, int x, int y)
{
	camera.keyboard(key, x, y);
	if (key == '5')
	{
		cout << camera.to_string() << endl;
	}
}
void mouseF(int x, int y)
{
	camera.mouse(x, y);
}

void mouse(int button, int state, int x, int y)
{
	ll_gl::camera_mouse_click(camera, button, state, x, y);
}

void pasteInData(Pixel3DSet & p, Mat & im)
{
	cvtColor(im, im, CV_GRAY2RGB);
	R3 mn, mx;
	//p.min_max_R3(mn, mx);
	//cout << mx << endl << mn << endl;
	//cout << "*************";
	//cout << endl;
	//system("pause");
	for (int i = 0; i < p.size(); i++)
	{
		R3 point = p[i];
		if (point.z < 0.0) continue;
		Point2i p2d(point.x, point.y);
		unsigned char color = 256.0f * (point.z / 80.0f);
		Vec3b outColor = ll_getColoredPixelFromGrayscale(255 - color);
		circle(im, p2d, 1, outColor, -1);
	}
}

void idle()
{
	//return;
	tmr.stop();
	double passed = tmr.getSeconds();
	if (pas > 0.03)
	{
		string directory = string(LCPPDATA_DIR) + string("/kitti/2011_09_26_drive_0001_sync/velodyne_points/data/");
		string imDirectory = string(LCPPDATA_DIR) + string("/kitti/2011_09_26_drive_0001_sync/image_00/data/");
		obj = kitti::read(directory, countt);
		//obj.transform_set(primary);
		unprojectNew3(obj, primary);
		countt++;
		countt %= 107;
		pas = 0;
		imi = kitti::readImage(imDirectory, countt);
		pasteInData(obj, imi);
		imshow("win-i", imi);
		waitKey(30);
		
	}
	pas += passed;

	tmr.reset();
	tmr.start();
}
#endif

using namespace ll_experiments;

void printProper(string name, Mat m)
{
	cout << "*******************" << name << "*******************" << endl << endl;

	for (int y = 0; y < m.rows; y++)
	{
		for (int x = 0; x < m.cols; x++)
		{
			cout << m.at<float>(y, x) << "\t\t\t";
		}
		cout << endl;
	}

	cout << "\n\n****************************" << endl << endl;
}

int main()
{
	string directory = string(LCPPDATA_DIR) + string("/kitti/2011_09_26_drive_0001_sync/velodyne_points/data/");
	string imDirectory = string(LCPPDATA_DIR) + string("/kitti/2011_09_26_drive_0001_sync/image_00/data/");

	velo2cam = ll_experiments::kitti::velo2Cam(string(LCPPDATA_DIR) + string("/kitti/2011_09_26_drive_0001_sync/"));
	//cout << velo2cam << endl;
	Mat R, P;
	ll_experiments::kitti::cam2cam(string(LCPPDATA_DIR) + string("/kitti/2011_09_26_drive_0001_sync/"), R, P, 0);

	namedWindow("win-i");

	P.at<float>(3, 3) = 0.0f;

	printProper("R", R);
	printProper("P", P);

	printProper("P*R", P*R);
	//return 5;
	

	primary = P*R*velo2cam;
	/*primary = Mat::eye(Size(4, 4), CV_32FC1);
	for (int y = 0; y < 3; y++)
	{
		for (int x = 0; x < 4; x++)
		{
			primary.at<float>(y, x) = tmp.at<float>(y, x);
		}
	}*/
	printProper("primary", primary);
	//
	cout << "primary-size: "<< primary.size() << endl;

	//primary1 = P;
	//Mat primary = Mat::eye(Size(4, 4), CV_32FC1);
	//for (int y = 0; y < 3; y++)
	//	for (int x = 0; x < 4; x++)
	//		primary.at<float>(y, x) = primary1.at<float>(y, x);
	//cout << primary1.size() << endl;
	//cout << primary1.size() << endl;
	//primary = primary.inv();
    //cout << "sizes: " << R.size() << " was R: P=" << P.size() << endl;
	//cout << "R:\n\n" << R << "\n\nP:\n\n" << P << endl;

	cout << "\n\n" << primary << endl << endl;

	
	//cout << "P\n" << P << endl;
	//cout << P.size() << endl << R.size() << endl;
	//cout << "out: " << endl << (P * R) << endl;
	//for (int i = 0; i < 1; i++) {
		//cout << filename(i) << endl;
	//return 0;

		obj = kitti::read(directory, 0);
		cout << obj.size() << endl;



        unprojectNew3(obj, primary);
		
		imi = kitti::readImage(imDirectory, 0);

		imshow("win-i", imi);
		waitKey(30);

		
		//unproject(obj);
		//obj.transform_set(P);



	//}
		R3 _mn, _mx;
		obj.min_max_R3(_mn, _mx);

		cout << "min: " << _mn << endl
			<< "max: " << _mx << endl;

		//system("pause");

		//return 0;
#ifdef HASGL
		tmr.start();


		ll_gl::default_glut_main("lukes phd project", 640, 480);

		glutDisplayFunc(display);

		glutKeyboardFunc(kbf);

		glutMotionFunc(mouseF);
		glutMouseFunc(mouse);
		glutIdleFunc(idle);
		glutMainLoop();
#endif
	return 0;
}


int main2(int argc, char * * argv)
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
	string fn = namesList[3];
	//exp1("Apartment.Texture.rotate", ll_experiments::rng(15, 20, 1));

    //quantitativeExperiment20("none", fn, inds);
    //quantitativeExperiment20("FM2D", fn, inds);
    //quantitativeExperiment20("FM3D", fn, inds);
    //quantitativeExperiment20("ICP", fn, inds);
    //quantitativeExperiment20("PCA", fn, inds);
    //quantitativeExperiment20("FVR", fn, inds);
    quantitativeExperiment20("FVR3D", fn, inds);
    quantitativeExperiment20("FVR3D-2", fn, inds);
	//quantitativeExperiment20("FFVR", fn, inds);


    }


	return 0;
}
