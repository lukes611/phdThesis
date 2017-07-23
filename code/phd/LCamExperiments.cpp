#include "LCamExperiments.h"
#include "experiments.h"
#include "../phd/fmRansac.h"
#include "../phd/Lpcr.h"
#include "../phd/Licp.h"
#include "../phd/LSift.h"
#include "../phd/measurements.h"
using namespace ll_pix3d;
using namespace cv;
using namespace ll_R3;
using namespace ll_experiments;
using namespace std;
/*


#ifdef HASGL

#include "../basics/ll_gl.h"
#include "../basics/llCamera.h"
using namespace ll_gl;
using namespace ll_cam;




//save camera experiments
//saves to version 2.0 data file
void saveCameraExperiments(string data_name, string alg_name, string distance, double noiseRange, double SNR, float trans, double rotation)
{
	//save output to file
	cout << "saving cameraExperiments.v2.0..." << endl;
	string outDirName = string(EXPS_DIR) + "/camera-tests/";
	//save name of file as [data-name].[versionNo].csv
	stringstream outFn; outFn << outDirName << "/" << data_name << ".v2.csv";
	//header: algorithm-name,amount,translation-x,rotation,noise-level,SNR
	string header = "alg,distance,translation,rotation,noise,SNR";
	string fileName = outFn.str();
	stringstream outData;
	outData <<
		alg_name << "," <<
		distance << "," <<
		trans << "," <<
		rotation << "," <<
		noiseRange << "," <<
		SNR;

	cout << outData.str() << endl;

	appendData(fileName, header, outData.str());



}


void currate(Pix3D a, Pix3D b, string dataset, string distance)
{
	R3 translation;
	double rotation = 0.0f;
	bool usingCamera = false;

	//set-up initial values
	ll_gl::default_glut_main("translation_curration", 640, 480);
	Fps_cam * camera = new Fps_cam(R3(40, 40, -60), 90.0f, 90.0f);
	LLPointers::setPtr("currate::camera", camera);

	//add-items
	LLPointers::setPtr("currate::translation", &translation);
	LLPointers::setPtr("currate::rotation", &rotation);
	LLPointers::setPtr("currate::usingCamera", &usingCamera);
	LLPointers::setPtr("currate::distance", &distance);
	LLPointers::setPtr("currate::dataset", &dataset);

	//add objects
	Pixel3DSet obj1Original = a;
	Pixel3DSet obj2 = b;
	Pixel3DSet obj1 = a;
	LLPointers::setPtr("currate::obj1Original", &obj1Original);
	LLPointers::setPtr("currate::obj1", &obj1);
	LLPointers::setPtr("currate::obj2", &obj2);


	glutDisplayFunc([]()->void
	{


		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("currate::camera");
		ll_gl::default_viewing(*camera);

		ll_gl::default_lighting();
		ll_gl::turn_off_lights();

		if (LLPointers::has("currate::obj1"))
		{
			Pixel3DSet * obj = LLPointers::getPtr<Pixel3DSet>("currate::obj1");
			glBegin(GL_POINTS);
			for (int i = 0; i < obj->points.size(); i++)
			{
				R3 col = obj->color_as_r3(i) / 255.0f;
				swap(col.x, col.z);
				ll_gl::set_color(col);
				ll_gl::glR3(obj->points[i]);
			}
			glEnd();
		}

		if (LLPointers::has("currate::obj2"))
		{
			Pixel3DSet * obj = LLPointers::getPtr<Pixel3DSet>("currate::obj2");
			glBegin(GL_POINTS);
			for (int i = 0; i < obj->points.size(); i++)
			{
				R3 col = obj->color_as_r3(i) / 255.0f;
				swap(col.x, col.z);
				ll_gl::set_color(col);
				R3 p = obj->points[i];
				ll_gl::glR3(p);
			}
			glEnd();
		}

		glutSwapBuffers();
		glutPostRedisplay();
	});

	glutKeyboardFunc([](unsigned char key, int x, int y)->void {
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("currate::camera");
		bool usingCamera = *LLPointers::getPtr<bool>("currate::usingCamera");

		if (usingCamera)
		{
			camera->keyboard(key, x, y);
			if (key == '5')
			{
				cout << camera->to_string() << endl;
			}
		}
		else
		{
			double * rotation = LLPointers::getPtr<double>("currate::rotation");
			R3 * translation = LLPointers::getPtr<R3>("currate::translation");

			if (key == '1' || key == '2' || key == '5' || key == '9' || key == '0')
			{


				double rinc = 0.01;
				float tinc = 0.01f;

				switch (key)
				{
				case '1': translation->x -= tinc; break;
				case '2': translation->x += tinc; break;
				case '9': *rotation -= rinc; break;
				case '0': *rotation += rinc; break;
				case '5': *rotation = 0.0; *translation = R3(0,0,0);
				}

				Pixel3DSet * obj1 = LLPointers::getPtr<Pixel3DSet>("currate::obj1");
				Pixel3DSet * obj1Original = LLPointers::getPtr<Pixel3DSet>("currate::obj1Original");

				*obj1 = *obj1Original;
				obj1->transform_set(0.0f, *rotation, 0.0f, 1.0f, translation->x, 0.0f, 0.0f, R3());
			}

			if (key == 's')
			{
				string dataset = *LLPointers::getPtr<string>("currate::dataset");
				string distance = *LLPointers::getPtr<string>("currate::distance");
				cout << "saving dataset:" << dataset << ", with distance:" << distance << endl;

				saveCameraExperiments(dataset, "ground-truth", distance, 0.0, std::numeric_limits<double>::infinity(), translation->x, *rotation);

			}



		}
		if (key == 'u')
		{
			double * rotation = LLPointers::getPtr<double>("currate::rotation");
			R3 * translation = LLPointers::getPtr<R3>("currate::translation");
			*LLPointers::getPtr<bool>("currate::usingCamera") = !usingCamera;
			if (usingCamera) cout << "not using camera" << endl;
			else
			{
				cout << "switched to camera " << endl;
				cout << "*********************************\n";
				cout << "translation: " << translation->x << endl;
				cout << "rotation: " << *rotation << endl;
				cout << "*********************************\n";
			}
		}
	});
	glutReshapeFunc([](int wi, int he)->void {

	});
	glutMotionFunc([](int x, int y)->void {
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("currate::camera");
		camera->mouse(x, y);
	});
	glutMouseFunc([](int button, int state, int x, int y)->void {
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("currate::camera");
		ll_gl::camera_mouse_click(*camera, button, state, x, y);
	});
	glutMainLoop();


}

void currate(string datasetname, int index1, int index2, string distance)
{
	CapturePixel3DSet frames = openData(datasetname, 4);
	Pix3D a, b;
	frames.read_frame(a, index1);
	frames.read_frame(b, index2);

	currate(a, b, datasetname, distance);
}




#endif

*/

Mat performReg(Pix3D frame1, Pix3D frame2, string algorithm_name)
{
    Pixel3DSet b = frame1;
    Pixel3DSet a = frame2;
    Mat _m = Mat::eye(Size(4, 4), CV_32FC1);
    double seconds;
    double hde;
    int iters = 0;
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
    }
    else if (algorithm_name == "FFVR") {
        _m = ll_pc::ffvr(b, a, seconds);
    }
#endif
    else if (algorithm_name == "PCA") {
        _m = ll_pca::register_pca(b, a, seconds);
    }
    else if (algorithm_name == "ICP") {
        Pixel3DSet _;
        _m = Licp::icp(b, a, _, hde, seconds, iters);
    }
    else if (algorithm_name == "FM2D") {
        ll_fmrsc::registerPix3D("surf", frame1, frame2, _m, seconds, true, 150);
    }
    else if (algorithm_name == "FM3D") {
        _m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
    }
    return _m.clone();
}

void saveCameraExperimentsV3(string data_name, string alg_name, string distance, double noiseRange, double SNR, float error)
{
	//save output to file
	cout << "saving cameraExperiments.v3.0..." << endl;
	string outDirName = string(EXPS_DIR) + "/camera-tests/";
	//save name of file as [data-name].[versionNo].csv
	stringstream outFn; outFn << outDirName << "/" << data_name << ".v2.csv";
	//header: algorithm-name,amount,translation-x,rotation,noise-level,SNR
	string header = "alg,distance,noise,SNR,error";
	string fileName = outFn.str();
	stringstream outData;
	outData <<
		alg_name << "," <<
		distance << "," <<
		noiseRange << "," <<
		SNR << "," <<
		error;

	cout <<
		alg_name << " | " <<
		distance << " | " <<
		noiseRange << " | " <<
		SNR << " | " <<
		error << endl;

	appendData(fileName, header, outData.str());



}

void lcamExpT(std::string dataset, std::string algorithm)
{
	CapturePixel3DSet frames = openData(dataset, 4);

	double noiseLevels[5] = {0.0, 0.1, 0.25, 0.5, 0.75};

	for (int j = 0; j < 5; j++)
	{

		double noiseRange = noiseLevels[j];
		string dists[3] = {"5cm", "10cm", "15cm"};
		for (int frameIndex = 1, i = 0; frameIndex <= 3; frameIndex++, i++)
		{
			Pix3D frame1, frame2;
			frames.read_frame(frame1, 0);
			frames.read_frame(frame2, frameIndex);
			double snr1 = 0.0, snr2 = 0.0;

			//add noise
			frame1 = ll_experiments::getNoisedVersion(frame1, noiseRange, snr1);
			frame2 = ll_experiments::getNoisedVersion(frame2, noiseRange, snr2);

			//set pixel3dsets
			Pixel3DSet a = frame1, b = frame2;

			//algorithms solving it:
			//algorithms here:
			string algorithm_name = algorithm;
			Mat _m = Mat::eye(Size(4, 4), CV_32FC1);
			double seconds;
			double hde;
			int iters = 0;
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
			}
			else if (algorithm_name == "FFVR") {
				_m = ll_pc::ffvr(b, a, seconds);
			}
#endif
			else if (algorithm_name == "PCA") {
				_m = ll_pca::register_pca(b, a, seconds);
			}
			else if (algorithm_name == "ICP") {
				Pixel3DSet _;
				_m = Licp::icp(b, a, _, hde, seconds, iters);
			}
			else if (algorithm_name == "FM2D") {
				ll_fmrsc::registerPix3D("surf", frame2, frame1, _m, seconds, true, 150);
			}
			else if (algorithm_name == "FM3D") {
				_m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
			}



			//end
			b.transform_set(_m);
			double msee, pme;
			hde = 21.0;
			ll_measure::error_metrics(a, b, hde, msee, pme);
            saveCameraExperimentsV3(dataset, algorithm, dists[i], noiseRange, (snr1 + snr2) / 2, msee);

		}
	}
}


void lcamExpR(std::string dataset, std::string algorithm)
{
	CapturePixel3DSet frames = openData(dataset, 4);

	double noiseLevels[4] = { 0.0, 0.1, 0.25, 0.3 };

	for (int j = 0; j < 4; j++)
	{

		double noiseRange = noiseLevels[j];
		string dists[3] = { "10deg", "20deg", "30deg" };
		for (int frameIndex = 1, i = 0; frameIndex <= 3; frameIndex++, i++)
		{
			Pix3D frame1, frame2;
			frames.read_frame(frame1, 0);
			frames.read_frame(frame2, frameIndex);
			double snr1 = 0.0, snr2 = 0.0;

			//add noise
			frame1 = ll_experiments::getNoisedVersion(frame1, noiseRange, snr1);
			frame2 = ll_experiments::getNoisedVersion(frame2, noiseRange, snr2);

			//set pixel3dsets
			Pixel3DSet a = frame2, b = frame1;

			//algorithms solving it:
			//algorithms here:
			string algorithm_name = algorithm;
			Mat _m = Mat::eye(Size(4, 4), CV_32FC1);
			double seconds;
			double hde;
			int iters = 0;
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
			}
			else if (algorithm_name == "FFVR") {
				_m = ll_pc::ffvr(b, a, seconds);
			}
#endif
			else if (algorithm_name == "PCA") {
				_m = ll_pca::register_pca(b, a, seconds);
			}
			else if (algorithm_name == "ICP") {
				Pixel3DSet _;
				_m = Licp::icp(b, a, _, hde, seconds, iters);
			}
			else if (algorithm_name == "FM2D") {
				ll_fmrsc::registerPix3D("surf", frame1, frame2, _m, seconds, true, 150);
			}
			else if (algorithm_name == "FM3D") {
				_m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
			}

			//end
			b.transform_set(_m);

            {
                double estRotation = 0.0;

                R3 p(0.0f, 0.0f, 0.0f);
                R3 v(0.0f, 0.0f, 1.0f);

                Pixel3DSet::transform_point(_m, p);
                Pixel3DSet::transform_point(_m, v);

                v -= p; //get vector
                v.normalize();

                estRotation = R3::getAngle(v.z, v.x);
                cout << dataset << " | " << algorithm << " | " << dists[i] << " | " <<  noiseRange << " | ";
                cout << "estimated rotation: " << estRotation << endl;



            }

			double msee, pme;
			hde = 21.0;
			ll_measure::error_metrics(a, b, hde, msee, pme);

			saveCameraExperimentsV3(dataset, algorithm, dists[i], noiseRange, (snr1 + snr2) / 2, msee);

		}
	}
}


void saveReg(std::string dataset, std::string algorithm, int f1, int f2)
{
    CapturePixel3DSet frames = openData(dataset, 4);
    Pix3D frame1, frame2;
    frames.read_frame(frame1, f1);
    frames.read_frame(frame2, f2);

    //set pixel3dsets
    Pixel3DSet a = frame2, b = frame1;
    Mat _m = performReg(frame1, frame2, algorithm);

    //end
    b.transform_set(_m);

    b.UNION(a);

    b.save_obj(string(DESKTOP_DIR) + string("/") + algorithm + string("-") + dataset + "-out.obj");

}
