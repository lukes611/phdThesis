#include <string>
#include <iostream>
#include "code/basics/SIObj.h"
#include "code/basics/locv3.h"
#include "code/basics/llCamera.h"
#include "code/phd/experiments.h"
#include "code/basics/VMatF.h"
#include "code/basics/ll_gl.h"
#include "code/basics/locv_algorithms.h"

#include "code/phd/Lpcr.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_siobj;
using namespace ll_experiments;



int main(){


	ll_pix3d::CapturePixel3DSet reader = ll_experiments::openData("Apartment.Texture.rotate", 3);

	Pixel3DSet p1;
	Pixel3DSet p2;

	{
		Pix3D p;
		reader.read_frame(p, 0);
		p1 = p;
	}
	{
		Pix3D p;
		reader.read_frame(p, 10);
		p2 = p;
	}
	
	
	

	ll_gl::default_glut_main("lukes phd project", 640, 480);

	Fps_cam * camera = new Fps_cam(R3(40, 40, -60), 90.0f, 90.0f);
	LLPointers::setPtr("camera", camera);
	//[-5.12343, 60, 4.23075] , angle_y: 62.000000, angle_x: 108.000000
	camera->angle_x = 108, camera->angle_y = 62;
	camera->location = R3(-5.12343, 60, 4.23075);

	//ll_algorithms::ll_pca_3d::LPCA pc(p.points, ll_algorithms::ll_pca_3d::LPCA::COMPUTE_2);
	//p.transform_set(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, pc.mean);
	//p.basicMinFilter();
	//p.positional_noise(2.0);
	//ll_algorithms::ll_pca_3d::LPCA pc2(p.points, ll_algorithms::ll_pca_3d::LPCA::COMPUTE_2);

	

	VMat p1v(256, p1, 0.0f, true);
	Mat xx = p1v.pca_correct_up();
	double ss;
	//Mat m = ll_pc::pc_register_pca(p1, p2, ss, true, 256);
	//p1.transform_set(m);
	//p1 += p2;
	//p1 = p1v.pixel3dset();
	//p1.transform_set(xx);
	//LLPointers::setPtr<ll_algorithms::ll_pca_3d::LPCA>("pca", &pc2);
	LLPointers::setPtr<Pixel3DSet>("object", &p1);

	


	glutDisplayFunc([]()->void
	{


		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("camera");
		ll_gl::default_viewing(*camera);

		ll_gl::default_lighting();
		ll_gl::turn_off_lights();

		if (LLPointers::has("object"))
		{
			Pixel3DSet * obj = LLPointers::getPtr<Pixel3DSet>("object");
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

		//draw the axes
		if (LLPointers::has("pca"))
		{
			ll_algorithms::ll_pca_3d::LPCA * pca = LLPointers::getPtr<ll_algorithms::ll_pca_3d::LPCA>("pca");
			R3 colors[] = {R3(255.0f, 0.0f, 0.0f), R3(0, 255, 0), R3(0,0,255)};
			for (int i = 0; i < 3; i++)
			{
				R3 a = pca->mean; R3 b = pca->eigenvecs[i];
				ll_gl::set_color(colors[i]);
				ll_gl::draw_line(a, a + b * (pca->eigenvals[i]*0.1 + 10));
			}
		}


		glutSwapBuffers();
		glutPostRedisplay();
	});

	glutKeyboardFunc([](unsigned char key, int x, int y)->void {
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("camera");
		camera->keyboard(key, x, y);
		if (key == '5')
		{
			cout << camera->to_string() << endl;
		}
	});
	glutReshapeFunc([](int wi, int he)->void {

	});
	glutMotionFunc([](int x, int y)->void {
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("camera");
		camera->mouse(x, y);
	});
	glutMouseFunc([](int button, int state, int x, int y)->void {
		Fps_cam * camera = LLPointers::getPtr<Fps_cam>("camera");
		ll_gl::camera_mouse_click(*camera, button, state, x, y);
	});
	glutMainLoop();




    return 0;
}

