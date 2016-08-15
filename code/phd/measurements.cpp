#include "measurements.h"
#include "Licp.h"

using namespace ll_R3;
using namespace Licp;
using namespace ll_pix3d;

namespace ll_measure
{

//v<R3> error metrics
double hausdorff(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b)
{
	vector<int> i1;
	vector<double> d1;
	double e1 = closestPointsf(a, b, i1, d1);
	double e2 = closestPointsf(b, a, i1, d1);
	return (e1 + e2) * 0.5;
}
double hausdorff(Pixel3DSet & a, Pixel3DSet & b)
{
	vector<int> i1;
	vector<double> d1;
	double e1 = closestPointsf(a, b, i1, d1);
	double e2 = closestPointsf(b, a, i1, d1);
	return (e1 + e2) * 0.5;
}
double avg(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b)
{
	return hausdorff(a, b);
}
double avg(Pixel3DSet & a, Pixel3DSet & b)
{
	return hausdorff(a, b);
}

double mse_1way(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b)
{
	vector<int> i1;
	vector<double> d1;
	double e1 = closestPointsf(a, b, i1, d1);
	double ret = 0.0;
	for(int i = 0; i < d1.size(); i++)
	{
		ret += d1[i] * d1[i];
	}
	return ret / (double)d1.size();
}
double mse_1way(Pixel3DSet & a, Pixel3DSet & b)
{
	vector<int> i1;
	vector<double> d1;
	double e1 = closestPointsf(a, b, i1, d1);
	double ret = 0.0;
	for(int i = 0; i < d1.size(); i++)
	{
		ret += d1[i] * d1[i];
	}
	return ret / (double)d1.size();
}
double mse(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b)
{
	return ( mse_1way(a,b) + mse_1way(b,a) ) * 0.5;
}
double mse(Pixel3DSet & a, Pixel3DSet & b)
{
	return ( mse_1way(a,b) + mse_1way(b,a) ) * 0.5;
}
double percentMatch(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b, double minError)
{
	return ( percentMatch_1way(a,b, minError) + percentMatch_1way(b,a, minError) ) * 0.5;
}
double percentMatch_1way(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b, double minError)
{
	vector<int> i1;
	vector<double> d1;
	double e1 = closestPointsf(a, b, i1, d1);

	double scalar = 1.0 / (double) a.size();
	
	double ret = 0.0;

	for(int i = 0; i < d1.size(); i++)
	{
		if(d1[i] <= minError) ret += scalar;
	}

	return ret;
}

double percentMatch(Pixel3DSet & a, Pixel3DSet & b, double minError)
{
	return ( percentMatch_1way(a,b, minError) + percentMatch_1way(b,a, minError) ) * 0.5;
}
double percentMatch_1way(Pixel3DSet & a, Pixel3DSet & b, double minError)
{
	vector<int> i1;
	vector<double> d1;
	double e1 = closestPointsf(a, b, i1, d1);

	double scalar = 1.0 / (double) a.size();
	
	double ret = 0.0;

	for(int i = 0; i < d1.size(); i++)
	{
		if(d1[i] <= minError) ret += scalar;
	}

	return ret;
}


//VMat error metrics
double avg(VMat & a, VMat & b, float threshold)
{
	int count = 0;
	double ret = 0.0;
	for(int z = 0; z < a.s; z++)
		for(int y = 0; y < a.s; y++)
			for(int x = 0; x < a.s; x++)
				if(a.at(x,y,z) > threshold || b.at(x,y,z) > threshold)
				{
					double d = abs(a.at(x,y,z) - b.at(x,y,z));
					ret += d;
					count++;
				}
	if(count <= 0) return -1.0;
	return ret / (double) count;
}
double mse(VMat & a, VMat & b, float threshold)
{
	int count = 0;
	double ret = 0.0;
	for(int z = 0; z < a.s; z++)
		for(int y = 0; y < a.s; y++)
			for(int x = 0; x < a.s; x++)
				if(a.at(x,y,z) > threshold || b.at(x,y,z) > threshold)
				{
					double d = a.at(x,y,z) - b.at(x,y,z);
					ret += d*d;
					count++;
				}
	if(count <= 0) return -1.0;
	return ret / (double) count;
}
double percentMatch(VMat & a, VMat & b, float threshold)
{
	int count = 0;
	for(int z = 0; z < a.s; z++)
		for(int y = 0; y < a.s; y++)
			for(int x = 0; x < a.s; x++)
			{
				double d = abs(a.at(x,y,z) - b.at(x,y,z));
				if(d <= threshold)
				{
					count++;
				}
			}
	return count / (double) a.s3;

}


Pix3D render(SIObj & objectIn, int volumeWidth, float focalDistance)
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
	
	return ret;
}

Pix3D render(Pixel3DSet & objectIn, int volumeWidth, float focalDistance)
{
	unsigned int S = 640 * 480; //image sizes for Pix3D are 640x480
	float volW = (float)volumeWidth; //floating point version of volume width
	float hvw = volW * 0.5f; //half the volume width

	//init buffers for the Pix3D data
	bool * vd = new bool[S];
	R3 * points = new R3[S];
	Vec3b * colors = new Vec3b[S];
	float * depths = new float[S];
	//initialize values depth map
	for(int i = 0; i < S; i++)
	{
		depths[i] = FLT_MAX;
		vd[i] = false;
	}

	//camera focal ranges in angles
	//angle-y = 60.0f, angle-x = 45.0f
	float angleY = 60.0f, angleX = 40.0f;
	//half versions
	float hay = angleY * 0.5f;
	float hax = angleX * 0.5f;

	int hw = 640 / 2;
	int hh = 480 / 2;

	
	R3 cameraLocation(hvw, hvw, -focalDistance); //the location of the camera

	

	//define the projection plane dimensions
	float planeWidth = tan(hay * ll_R3_C::ll_R3_deg2rad) * focalDistance * 2.0f;
	float planeHeight = tan(hax * ll_R3_C::ll_R3_deg2rad) * focalDistance * 2.0f;
	float hpw = planeWidth * 0.5f;
	float hph = planeHeight * 0.5f;

	
	//define the top left corner of the projection plane
	R3 planeTopLeft = R3(hvw - hpw, hvw + hph, 0.0f);
	R3 planeBottomRight = planeTopLeft + R3(planeWidth, -planeHeight, 0.0f);
	
	for(int i = 0; i < objectIn.size(); i++)
	{
		//project onto projection plane
		R3 ray = (objectIn[i] - cameraLocation).unit();
		ray *= focalDistance / ray.z;
		//check if it is within the bounds
		int x = round((ray.x - planeTopLeft.x) / planeWidth * 640.0f) + hw;
		int y = round((planeTopLeft.y - ray.y) / planeHeight * 480.0f) - hh;
		if(x>=0 && y>=0 && x<640 && y<480)
		{
			int index = y*640 + x;
			//compute the depth
			float depth = ray.dist(cameraLocation);
			//update point if necessary
			if(depth < depths[index])
			{
				depths[index] = depth;
				vd[index] = true;
				colors[index] = objectIn.colors[i];
				points[index] = objectIn[i];
			}
			
		}
	}
	

	Pix3D ret(S, points, colors, vd);
	delete [] points;
	delete [] colors;
	delete [] vd;
	delete [] depths;
	
	return ret;
}

}