#include <iostream>
#include <string>
#include <vector>

#include "code\basics\locv3.h"
#include "code\basics\R3.h"
#include "code\basics\llCamera.h"
#include "code\basics\VMatF.h"
#include "code\basics\LTimer.h"
#include "code\phd\Licp.h"
#include "code\phd\measurements.h"

using namespace std;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
//using namespace cv;

//proto-type for experiments: VMat pc(VMat a, VMat b, double & time, Mat & transform, double & mse);
//to-do:= pc, ipc, 

/*

pc [almost]
icp [almost]
fm [ - ]


*/

/*
	capture:
		input	:		SIObj, camera params
		output	:		Pix3D object

	camera params: focalDistance, angle

*/

//width and height is always: 640 x 480
//total count is 640 * 480
void capture(SIObj & objectIn, int volumeWidth, float focalDistance, int __i)
{
	unsigned int S = 640 * 480;
	float volW	= (float) volumeWidth;
	float hvw = volW * 0.5f; //half the volume width

	Mat vd = Mat::zeros(480, 640, CV_8UC1);

	//angle-y = 60.0f, angle-x = 45.0f
	float angleY = 60.0f, angleX = 40.0f;
	float hay = angleY * 0.5f;
	float hax = angleX * 0.5f;

	//cout << objectIn.maximal_vertex_dimensions() << " -1s\n";

	//now take the pixel locations and draw some color in them: meanwhile draw a 3d point in too, if no hit: just set validDepth to false
	R3 cameraLocation(hvw, hvw, -focalDistance);
	float planeWidth = tan(hay * ll_R3_C::ll_R3_deg2rad) * focalDistance * 2.0f;
	float planeHeight = tan(hax * ll_R3_C::ll_R3_deg2rad) * focalDistance * 2.0f;
	float hpw = planeWidth * 0.5f;
	float hph = planeHeight * 0.5f;

	cout << "plane-size: " << Point2f(planeWidth, planeHeight) << endl;
	
	R3 planeTopLeft = R3(hvw - hpw, hvw + hph, 0.0f);
	cout << "camera location: " << cameraLocation << endl;
	cout << "planeTopLeft: " << planeTopLeft << endl;
	float rayXInc = planeWidth / 640.0f;
	float rayYInc = -planeHeight / 480.0f;
	R3 q = planeTopLeft;

	cout << "incs: " << Point2f(rayXInc, rayYInc) << endl;
	vector<SI_FullTriangle> tris; vector<R3> normals;
	R3 camera_direction(0.0f, 0.0f, -1.0f);
	for(int i = 0; i < objectIn._triangles.size(); i++)
	{
		SI_FullTriangle triangle(objectIn.getTriangle(i));
		R3 normal = triangle.normal();
		float angle = acos(normal * camera_direction) * ll_R3_C::ll_R3_rad2deg;
		if(abs(angle) <= 90.0f)
		{
			tris.push_back(triangle);
			normals.push_back(normal);
		}
	}
	
	cout << "measuring : " << tris.size() << " no. triangles of " << objectIn._triangles.size() << endl;
	bool found = false, hit = false;
	R3 ray, ip;
	float d = 0.0f;
	int count = 0;
	for(int y = 0; y < 480; y++)
	{
		q.y = (float)y * rayYInc + planeTopLeft.y;
		for(int x = 0; x < 640; x++)
		{
			q.x = (float)x * rayXInc + planeTopLeft.x;
			ray = (q - cameraLocation).unit(); 
			found = false;
			for(int j = 0; j < tris.size(); j++)
			{
				hit = false;
				SI_FullTriangle T = tris[j];
				d = T.RayIntersectsTriangle(cameraLocation, ray, normals[j], ip, hit);
				if(hit)
				{
					count++;
					found = true;
					break;
				}
			}
			vd.at<unsigned char>(y,x) = found ? 0xFF : 0x00;
		}
	}
	cout << "%= " << (((double)count)  / (double)S) << endl;
	cout << "imshow...";
	stringstream sout;
	sout << "vd" << focalDistance;
	imshow(sout.str(), vd);
	cout << "-> done\n";
	//cout << objectIn.maximal_vertex_dimensions() << " 0s\n";
	waitKey(30);
}


int main(int argc, char * * argv)
{
	
	
	string cubes = "C:/lcppdata/obj/cubes.obj";
	string bunny = "C:/lcppdata/obj/bunny_simplified2.obj";
	string zombie = "C:/lcppdata/obj/zombie.obj";

	float Sz[] = {200.0f, 300.0f, 450.0f, 1024.0f};
	SIObj ob; ob.open_obj(cubes);
	ob.normalize(256.0f);
	
	for(int i = 0; i < 4; i++)
	{
		LTimer t; t.start();
		capture(ob, 256, Sz[i], i);
		t.stop();
		cout << "time: " << t.getSeconds() << endl;
		waitKey();
	}

	

	return 0;
}
