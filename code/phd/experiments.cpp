#include "experiments.h"

using namespace std;
#include "../basics/Pixel3DSet.h"
#include "../basics/ll_gl.h"
#include "../basics/R3.h"
#include "../basics/llCamera.h"

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

}

