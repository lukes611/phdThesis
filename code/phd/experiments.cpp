#include "experiments.h"
#include <fstream>
#include <ctime>
#include <string>
#include <cstdio>
#include <cstdlib>

using namespace std;
#include "../basics/Pixel3DSet.h"
#ifdef HASGL
#include "../basics/ll_gl.h"
#endif
#include "../basics/R3.h"
#include "../basics/llCamera.h"
#include "../basics/locv3.h"

using namespace ll_pix3d;
using namespace ll_R3;
using namespace ll_cam;
using namespace cv;

namespace ll_experiments
{

map<string, void *> * LLPointers::access(){
	static map<string, void *> mp;
	return &mp;
}

bool LLPointers::has(string key){
	map<string, void*> * ptr = LLPointers::access();
	return ptr->find(key) != ptr->end();
}

void viewVideo(string name, bool viewColor, bool viewDepth, bool viewVD, int wks)
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

vector<int> rng(int to){ return rng(0, to, 1);}
vector<int> rng(int from, int to)
{
	return rng(from, to, 1);
}
vector<int> rng(int from, int to, int inc)
{
	vector<int> ret;
	if(from < to && inc > 0)
		for(int i = from; i < to; i+=inc) ret.push_back(i);
	else if(from > to && inc < 0) for(int i = from; i > to; i-=inc);
	return ret;
}

bool fileIsEmpty(string fileName)
{
    ifstream file;
    file.open(fileName.c_str(), ios::in);

    if(!file.is_open()) return true;

    file.seekg(0, ios::end);
    int len = file.tellg();
    file.close();
    return len == 0;
}

void appendData(string fileName, string header, string data, bool includeNewLine)
{
    bool isEmpty = fileIsEmpty(fileName);
    ofstream file;
    file.open(fileName, ios::app);

    cout << "is empty : " << isEmpty << endl;
    if(isEmpty)
    {
        file << header; if(includeNewLine) file << "\n";
    }
    file << data; if(includeNewLine) file << "\n";
    file.close();
}

ll_pix3d::CapturePixel3DSet openData(string name, int numFrames)
{
	stringstream path;
	path << LCPPDATA_DIR << "/pix3dc/films";
	return CapturePixel3DSet::openCustom(path.str(), name, numFrames);
}

namespace kitti
{
	string getFileName(string directory, int index)
	{
		stringstream t;
		t << directory;
		int numDP = index == 0 ? 1 : log10(index) + 1;
		for (int i = 0; i < 10 - numDP; i++) t << "0";
		t << index << ".bin";
		return t.str();
	}
	Pixel3DSet read(string directoryName, int index, bool flip)
	{
		FILE * file = fopen(getFileName(directoryName, index).c_str(), "rb");
		Pixel3DSet ret;
		if (file)
		{
			while (!feof(file))
			{
				float data[4];
				fread(data, sizeof(float), 4, file);
				R3 newPoint(data[0], data[1], data[2]);
				Vec3b newColor(Vec3b(255, 255, 255));
				ret.push_back(newPoint, newColor);
			}
		}
		if (flip)
		{
			//ret.transform_set(90, 0, 0, 1, 0, 0, 0, R3());
		}
		return ret;
	}

	Mat velo2Cam(string directoryName)
	{
		Mat ret = Mat::eye(Size(4, 4), CV_32FC1);
		FILE * file = fopen((directoryName + "calib_velo_to_cam.txt").c_str(), "r");

		char buf[100];
		fscanf(file, "%s", buf);
		fscanf(file, "%s", buf);
		fscanf(file, "%s", buf);
		fscanf(file, "%s", buf);
		for (int y = 0; y < 3; y++)
		{
			for (int x = 0; x < 3; x++)
			{
				double tmp;
				fscanf(file, "%lf", &tmp);
				ret.at<float>(y, x) = tmp;
			}
		}
		fscanf(file, "%s", buf);
		for (int y = 0; y < 3; y++)
		{
			double tmp;
			fscanf(file, "%lf", &tmp);
			ret.at<float>(y, 3) = tmp;
		}
		fclose(file);
		return ret;
	}
	void cam2cam(std::string directoryName, cv::Mat & R_rect_0x, cv::Mat & P_rect_0x, int x)
	{
        R_rect_0x = Mat::eye(Size(4,4), CV_32FC1);
        P_rect_0x = Mat::eye(Size(4,3), CV_32FC1);
        string wholeFile = "";
        FILE * file = fopen((directoryName + "calib_cam_to_cam.txt").c_str(), "r");
        string rheader, pheader;
        {
            stringstream rh, ph;
            rh << "R_rect_0" << "0:";
            ph << "P_rect_0" << x << ":";
            rheader = rh.str();
            pheader = ph.str();
        }
        if(file)
        {
            while(!feof(file))
            {
                char buf[101];
                int nr = fread(buf, 1, 100, file);
                buf[nr] = 0;
                wholeFile += buf;

            }
            vector<string> lines = ll_split(wholeFile, '\n');
            for(int l = 0; l < lines.size(); l++)
            {
                vector<string> words = ll_split(lines[l], ' ');
                if(words[0] == rheader)
                {
                    for(int y = 0, i = 1; y < 3; y++)
                        for(int x = 0; x < 3; x++, i++)
                        {
                            double tmp;
                            sscanf(words[i].c_str(), "%lf", &tmp);
                            R_rect_0x.at<float>(y,x) = tmp;
                        }
                }else if(words[0] == pheader)
                {
                    for(int y = 0, i = 1; y < 3; y++)
                        for(int x = 0; x < 4; x++, i++)
                        {
                            double tmp;
                            sscanf(words[i].c_str(), "%lf", &tmp);
                            P_rect_0x.at<float>(y,x) = tmp;
                        }
                }
            }
        }else cout << "could not open cam2cam calibration file\n";

	}
}


#ifdef HASGL

void viewPixel3DSet()
{
	ll_gl::default_glut_main("lukes phd project", 640, 480);

	Fps_cam * camera = new Fps_cam(R3(40,40,-60), 90.0f, 90.0f);
	LLPointers::setPtr("camera", camera);

	glutDisplayFunc([]()->void
	{


		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("camera");
		ll_gl::default_viewing(*camera);

		ll_gl::default_lighting();
		ll_gl::turn_off_lights();

		if(LLPointers::has("object"))
		{
			Pixel3DSet * obj = LLPointers::getPtr<Pixel3DSet>("object");
			glBegin(GL_POINTS);
			for(int i = 0; i < obj->points.size(); i++)
			{
				R3 col = obj->color_as_r3(i) / 255.0f;
				swap(col.x, col.z);
				ll_gl::set_color(col);
				ll_gl::glR3(obj->points[i]);
			}
			glEnd();
		}

		glutSwapBuffers();
		glutPostRedisplay();
	});

	glutKeyboardFunc([](unsigned char key, int x, int y)->void{
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("camera");
		camera->keyboard(key, x,y);
		if(key == '5')
		{
			cout << camera->to_string() << endl;
		}
	});
	glutReshapeFunc([](int wi, int he)->void{

	});
	glutMotionFunc([](int x, int y)->void{
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("camera");
		camera->mouse(x,y);
	});
	glutMouseFunc([](int button, int state, int x, int y)->void{
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("camera");
		ll_gl::camera_mouse_click(*camera, button, state, x, y);
	});
	glutMainLoop();
}

#endif


NoiseGenerator::NoiseGenerator(bool seedAtBeginning)
{
	if (seedAtBeginning) seed();
}
NoiseGenerator::~NoiseGenerator()
{
	noise.clear();
	signal.clear();
}
double NoiseGenerator::randomNumber()
{
	return rand() / (double)RAND_MAX;
}
Point3f NoiseGenerator::randomPoint()
{
	return Point3f(randomNumber(), randomNumber(), randomNumber());
}
void NoiseGenerator::seed()
{
	srand(time(NULL));
}
double NoiseGenerator::randomNumber(double range)
{
	double h = range * 0.5f;
	return randomNumber() * range - h;
}
Point3f NoiseGenerator::randomPoint(double range)
{
	return Point3f(randomNumber(range), randomNumber(range), randomNumber(range));
}

Point3f NoiseGenerator::getNoise(Point3f signal, double range)
{
	Point3f newNoise = randomPoint(range);

	this->signal.push_back(signal);
	this->noise.push_back(newNoise);

	return newNoise;
}

R3 NoiseGenerator::getNoise(R3 signal, double range)
{
	Point3f _ = randomPoint(range);
	R3 newNoise(_.x, _.y, _.z);

	this->signal.push_back(Point3f(signal.x, signal.y, signal.z));
	this->noise.push_back(Point3f(newNoise.x, newNoise.y, newNoise.z));

	return newNoise;
}

Point3f NoiseGenerator::stdDev(vector<Point3f> & data)
{
	Point3f mn = mean(data);
	Point3f stdDev(0.0f, 0.0f, 0.0f);
	if (data.size() == 0) return stdDev;
	float scalar = 1.0f / (float)data.size();

	for (int i = 0; i < data.size(); i++)
	{
		Point3f p = data[i] - mn;
		p.x *= p.x;
		p.y *= p.y;
		p.z *=  p.z;
		stdDev += p;
	}

	stdDev.x = sqrt(stdDev.x * scalar);
	stdDev.y = sqrt(stdDev.y * scalar);
	stdDev.z = sqrt(stdDev.z * scalar);

	return stdDev;
}

Point3f NoiseGenerator::mean(vector<Point3f> & data)
{
	Point3f init(0.0f, 0.0f, 0.0f);
	if (data.size() == 0) return init;
	float scalar = 1.0f / (float)data.size();
	for (int i = 0; i < data.size(); i++)
	{
		init += data[i] * scalar;
	}



	return init;
}
Point3f NoiseGenerator::stdDevNoise()
{
	return stdDev(noise);
}
Point3f NoiseGenerator::stdDevSignal()
{
	return stdDev(signal);
}

void NoiseGenerator::getSNR(Point3f & out)
{
	Point3f ss = stdDevSignal(), ns = stdDevNoise();

	out.x = (ss.x * ss.x) / (ns.x * ns.x);
	out.y = (ss.y * ss.y) / (ns.y * ns.y);
	out.z = (ss.z * ss.z) / (ns.z * ns.z);

}
void NoiseGenerator::getSNR(double & out)
{
	Point3f op;
	getSNR(op);

	out = (op.x + op.y + op.z) / 3.0f;
}

ll_pix3d::Pixel3DSet getNoisedVersion(ll_pix3d::Pixel3DSet & input, double noiseRange, double & snrOut)
{
	Pixel3DSet cp = input;
	NoiseGenerator noise;
	for (int i = 0; i < cp.size(); i++)
	{
		cp.points[i] += noise.getNoise(input.points[i], noiseRange);
	}
	noise.getSNR(snrOut);
	return cp;
}
ll_pix3d::Pix3D getNoisedVersion(ll_pix3d::Pix3D & input, double noiseRange, double & snrOut)
{
	Pix3D cp = input;
	NoiseGenerator noise;
	for (int i = 0; i < cp.count; i++)
	{
		if (cp.validDepth && cp.validDepth[i])
		{
			cp.points[i] += noise.getNoise(input.points[i], noiseRange);
		}
	}
	noise.getSNR(snrOut);
	return cp;
}

}

