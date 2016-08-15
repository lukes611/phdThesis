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
	vector<SI_FullTriangle> tris; vector<R3> normals, C2Ps;
	vector<float> radiuses;
	vector<float> distances;
	R3 camera_direction(0.0f, 0.0f, -1.0f);
	for(int i = 0; i < objectIn._triangles.size(); i++)
	{
		SI_FullTriangle triangle(objectIn.getTriangle(i));
		R3 normal = triangle.normal();
		float angle = acos(normal * camera_direction) * ll_R3_C::ll_R3_rad2deg;
		if(abs(angle) <= 90.0f)
		{
			R3 center = (triangle.a + triangle.b + triangle.c) * (1.0f / 3.0f);
			float radius = R3(center.dist(triangle.a), center.dist(triangle.b), center.dist(triangle.c)).max();
			tris.push_back(triangle);
			normals.push_back(normal);
			C2Ps.push_back(center - cameraLocation);
			radiuses.push_back(radius);
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

				//check if ray intersects bounding sphere
				//C2P is the vector from the camera to the center
				R3 C2P = C2Ps[j];
				R3 projectedPoint = C2P.project(ray);
				if (projectedPoint.dist(C2P) > radiuses[j]) continue;

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

Pix3D capture(SIObj & objectIn, int volumeWidth, float focalDistance)
{
	unsigned int S = 640 * 480; //image sizes for Pix3D are 640x480
	float volW = (float)volumeWidth; //floating point version of volume width
	float hvw = volW * 0.5f; //half the volume width

	R3 lightPosition = R3(-1000.0f, 1000.0f, -1000.0f);
	float ambientLight = 100.0f;


	//init buffers for the Pix3D data
	bool * vd = new bool[S];
	R3 * points = new R3[S];
	Vec3b * colors = new Vec3b[S];

	//camera focal ranges in angles
	//angle-y = 60.0f, angle-x = 45.0f
	float angleY = 60.0f, angleX = 40.0f;
	//half versions
	float hay = angleY * 0.5f;
	float hax = angleX * 0.5f;

	

	
	R3 cameraLocation(hvw, hvw, -focalDistance); //the location of the camera

	//define the projection plane dimensions
	float planeWidth = tan(hay * ll_R3_C::ll_R3_deg2rad) * focalDistance * 2.0f;
	float planeHeight = tan(hax * ll_R3_C::ll_R3_deg2rad) * focalDistance * 2.0f;
	float hpw = planeWidth * 0.5f;
	float hph = planeHeight * 0.5f;

	
	//define the top left corner of the projection plane
	R3 planeTopLeft = R3(hvw - hpw, hvw + hph, 0.0f);
	

	float rayXInc = planeWidth / 640.0f;
	float rayYInc = -planeHeight / 480.0f;
	

	//pre-compute some data used for ray-tracing
	//tris : the triangles
	//normals : the normal vectors for the triangles
	//c2ps : the vector from the camera to the center of the triangles
	//radiuses : the radius' of the bounding spheres for each triangle
	vector<SI_FullTriangle> tris; vector<R3> normals, C2Ps;
	vector<float> radiuses;
	R3 camera_direction(0.0f, 0.0f, -1.0f);

	for (int i = 0; i < objectIn._triangles.size(); i++) //for each triangle
	{
		SI_FullTriangle triangle = objectIn.getTriangle(i);
		R3 normal = triangle.normal();
		float angle = acos(normal * camera_direction) * ll_R3_C::ll_R3_rad2deg;
		if (abs(angle) <= 90.0f) //perform back-face culling
		{
			R3 center = (triangle.a + triangle.b + triangle.c) * (1.0f / 3.0f);
			float radius = R3(center.dist(triangle.a), center.dist(triangle.b), center.dist(triangle.c)).max();
			tris.push_back(triangle);
			normals.push_back(normal);
			C2Ps.push_back(center - cameraLocation);
			radiuses.push_back(radius);
		}
	}
		
	bool found = false, hit = false;
	R3 ray, ip;
	float d = 0.0f, bestDistance;
	R3 intersectionPoint, normal;
	int __index = 0;

	R3 q = planeTopLeft;

	for (int y = 0; y < 480; y++, q.y += rayYInc)
	{
		q.x = planeTopLeft.x;
		for (int x = 0; x < 640; x++, __index++, q.x += rayXInc)
		{
			ray = (q - cameraLocation).unit();
			found = false;
			bestDistance = FLT_MAX;
			for (int j = 0; j < tris.size(); j++)
			{
				hit = false;

				//check if ray intersects bounding sphere
				//C2P is the vector from the camera to the center
				R3 C2P = C2Ps[j];
				R3 projectedPoint = C2P.project(ray);
				if (projectedPoint.dist(C2P) > radiuses[j]) continue;

				SI_FullTriangle T = tris[j];
				d = T.RayIntersectsTriangle(cameraLocation, ray, normals[j], ip, hit);
				if (hit)
				{
					if (d < bestDistance)
					{
						bestDistance = d;
						intersectionPoint = ip;
						normal = normals[j];
					}
					found = true;
				}
			}
			if (found)
			{
				points[__index] = intersectionPoint;
				R3 lightNormal = (lightPosition - intersectionPoint).unit();
				float angle = abs(acos(normal * lightNormal) * ll_R3_C::ll_R3_rad2deg);
				float _color = ambientLight;
				if (angle <= 90.0f)
				{
					float diffuse = 180.0f - angle;
					_color += diffuse / 180.0f * 150.0f;
				}
				_color = _color > 255.0f ? 255.0f : _color;
				colors[__index] = Vec3b((unsigned char)_color, (unsigned char)_color, (unsigned char)_color);
				
			}
			vd[__index] = found;
			
		}
	}
	Pix3D ret(S, points, colors, vd);
	delete [] points;
	delete [] colors;
	delete [] vd;
	//cout << "%= " << (((double)count) / (double)S) << endl;
	cout << "imshow...";
	//stringstream sout;
	//sout << "vd" << focalDistance;

	Mat _vd, dp, col;
	ret.depthImage(dp);
	ret.colorImage(col);
	ret.vdImage(_vd);
	imshow("x", _vd);
	imshow("y", dp);
	imshow("z", col);
	cout << "-> done\n";
	//cout << objectIn.maximal_vertex_dimensions() << " 0s\n";
	waitKey(30);
	return ret;
}



int main(int argc, char * * argv)
{
	
	
	string cubes = "C:/lcppdata/obj/cubes.obj";
	string bunny = "C:/lcppdata/obj/bunny_simplified.obj";
	string zombie = "C:/lcppdata/obj/zombie.obj";
	string tp = "C:/lcppdata/obj/teapot.obj";
	string rp = "C:/lcppdata/obj/raptor.obj";
	string bunnysi = "C:/lcppdata/obj/bunny.si";

	float Sz[] = {50.0f, 100.0f, 250.0f, 424.0f};
	SIObj ob; ob.open_obj(rp);
	
	//ob.toOrigin();

	//cout << ob.maximal_vertex_dimensions() << endl;

	/*ob.normalize(256.0f);
	ob.mutate_subtract((ob.maximal_vertex_dimensions() - ob.minimal_vertex_dimensions()) * 0.5f);
	ob.mutate_add(R3(128, 128, 128));*/
	
	//ob.toOrigin();
	//ob.mutate_add(R3(128.0f, 128.0f, 128.0f));

	//cout << ob.maximal_vertex_dimensions() << endl;
	ob.centerNormalize(256.0f);
	ob.compute_normals();
	for(int i = 0; i < 4; i++)
	{
		LTimer t; t.start();
		capture(ob, 256, Sz[i]);
		t.stop();
		cout << "time: " << t.getSeconds() << endl;
		waitKey();
	}

	

	return 0;
}
