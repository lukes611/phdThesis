#include "LCamExperiments.h"




#ifdef HASGL

#include "../basics/ll_gl.h"
#include "../basics/llCamera.h"
#include "experiments.h"

using namespace ll_pix3d;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_experiments;
using namespace std;
using namespace ll_gl;

void currate(Pix3D a, Pix3D b, string distance)
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
			if (key == '1' || key == '2' || key == '5' || key == '9' || key == '0')
			{
				double * rotation = LLPointers::getPtr<double>("currate::rotation");
				R3 * translation = LLPointers::getPtr<R3>("currate::translation");

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

	currate(a, b, distance);
}


#endif