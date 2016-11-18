/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description: 
		The .cpp file for VMatF, which is a floating point, single channel only 3D version of opencv's Mat object

	depends on: Pixel3DSet, R3, locv_algorithms
*/

#include "VMatF.h"
#include "locv_algorithms.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <functional>
#include <complex>
using namespace std;
using namespace cv;

VMat::VMat(int sIn)
{
	s = sIn;
	setup_init();
}
VMat::VMat(int sIn, vector<R3> l, float offValue, float onValue)
{
	s = sIn;
	setup_init();
	setAll(offValue);
	R3 mn = l[0];
	R3 mx = l[0];
	for(int i = 1; i < l.size(); i++)
	{
		if(l[i].x < mn.x) mn.x = l[i].x;
		if(l[i].y < mn.y) mn.y = l[i].y;
		if(l[i].z < mn.z) mn.z = l[i].z;
		
		if(l[i].x > mx.x) mx.x = l[i].x;
		if(l[i].y > mx.y) mx.y = l[i].y;
		if(l[i].z > mx.z) mx.z = l[i].z;
	}
	float fx = (mx.x > mx.y)? mx.x : mx.y;
	fx = (fx > mx.z)? fx : mx.z;
	float fn = (mn.x < mn.y)? mn.x : mn.y;
	fn = (fn < mn.z)? fn : mn.z;
	fx -= fn;
	for(int i = 0; i < l.size(); i++)
	{
		R3 c = l[i];
		c -= mn;
		c *= (((float)s) / fx);
		int xx = (int)round(c.x);
		int yy = (int)round(c.y);
		int zz = (int)round(c.z);
		if(inbounds(xx,yy,zz))
		{
			at(xx,yy,zz) = onValue;
		}
	}
}

#ifdef AVG_SAMPLING

VMat::VMat(int sIn, Pixel3DSet l, float offValue, bool l_is_scaled)
{
	s = sIn;
	setup_init();
	setAll(offValue);
	int * counts = new int[s3];
	for(int i = 0; i < s3; i++) counts[i] = 0;
	if(!l_is_scaled)
	{
		R3 mn, mx;
		l.min_max_R3(mn, mx);
		float fx = (mx.x > mx.y)? mx.x : mx.y;
		fx = (fx > mx.z)? fx : mx.z;
		float fn = (mn.x < mn.y)? mn.x : mn.y;
		fn = (fn < mn.z)? fn : mn.z;
		fx -= fn;
		for(int i = 0; i < l.size(); i++)
		{
			R3 c = l[i];
			c -= mn;
			c *= (((float)s) / fx);
			int xx = RR(c.x);
			int yy = RR(c.y);
			int zz = RR(c.z);
			if(inbounds(xx,yy,zz))
			{
				at(xx,yy,zz) += l.gsNPixel(i);
				counts[zz*s2 + yy*s + xx]++;
			}
		}
	}else
	{
		for(int i = 0; i < l.size(); i++)
		{
			R3 c = l[i];
			int xx = RR(c.x);
			int yy = RR(c.y);
			int zz = RR(c.z);
			if(inbounds(xx,yy,zz))
			{
				at(xx,yy,zz) += l.gsNPixel(i);
				counts[zz*s2 + yy*s + xx]++;
			}
		}
	}

	for(int i = 0; i < s3; i++)
	{
		if(counts[i] > 0)
		{
			data[i] /= (float)counts[i];
		}else
		{
			data[i] = 0.0f;
		}
	}
	delete [] counts;
}

#else

VMat::VMat(int sIn, Pixel3DSet l, float offValue, bool l_is_scaled)
{
	s = sIn;
	setup_init();
	setAll(offValue);
	if(!l_is_scaled)
	{
		R3 mn, mx;
		l.min_max_R3(mn, mx);
		float fx = (mx.x > mx.y)? mx.x : mx.y;
		fx = (fx > mx.z)? fx : mx.z;
		float fn = (mn.x < mn.y)? mn.x : mn.y;
		fn = (fn < mn.z)? fn : mn.z;
		fx -= fn;
		for(int i = 0; i < l.size(); i++)
		{
			R3 c = l[i];
			c -= mn;
			c *= (((float)s) / fx);
			int xx = (int)round(c.x);
			int yy = (int)round(c.y);
			int zz = (int)round(c.z);
			if(inbounds(xx,yy,zz))
			{
				at(xx,yy,zz) = l.gsNPixel(i);
			}
		}
	}else
	{
		for(int i = 0; i < l.size(); i++)
		{
			R3 c = l[i];
			int xx = (int)round(c.x);
			int yy = (int)round(c.y);
			int zz = (int)round(c.z);
			if(inbounds(xx,yy,zz))
			{
				at(xx,yy,zz) = l.gsNPixel(i);
			}
		}
	}
}

#endif

VMat::~VMat()
{
	if(s != 0)
		delete [] data;
	s = s2 = s3 = 0;
}
VMat::VMat(const VMat & cp)
{
	s = 0;
	copyInVM(cp);
}
VMat::VMat(string fn)
{
	s = 0;
	open(fn);
}
VMat & VMat::operator = (const VMat & cp)
{
	copyInVM(cp);
	return *this;
}
void VMat::copyInVM(const VMat & cp)
{
	free();
	s = cp.s;
	setup_init();
	for(int i = 0; i < s3; i++)
	{
		data[i] = cp.data[i];
	}
}
//sets up everything once s is set, assumes no memory is allocated
void VMat::setup_init()
{
	s2 = s * s;
	s3 = s2 * s;
	data = new float[s3];
}
void VMat::free()
{
	if(s != 0)
		delete [] data;
	s = s2 = s3 = 0;
}

void VMat::operator *= (float s)
{
	for(int i = 0; i < s3; i++)
	{
		data[i] *= s;
	}
}
void VMat::operator /= (float s)
{
	for(int i = 0; i < s3; i++)
	{
		data[i] /= s;
	}
}
void VMat::operator += (float s)
{
	for(int i = 0; i < s3; i++)
	{
		data[i] += s;
	}
}
void VMat::operator -= (float s)
{
	for(int i = 0; i < s3; i++)
	{
		data[i] -= s;
	}
}
float VMat::average_pixel()
{
	float sum = 0;
	for(int i = 0; i < s3; i++)
	{
		sum += data[i];
	}
	return sum / (float)s3;
}
double VMat::mean()
{
	double sum = 0.0;
	for(int i = 0; i < s3; i++)
	{
		sum += (double)data[i];
	}
	return sum / (double)s3;
}
double VMat::variance()
{
	return variance(mean());
}
double VMat::variance(double mean)
{
	float sum = 0.0f;
	for(int i = 0; i < s3; i++)
	{
		sum += (float)pow((double)data[i] - mean, 2.0);
	}
	return sum / ((double)(s3-1));
}
double VMat::stddev()
{
	return stddev(mean());
}
double VMat::stddev(double mean)
{
	return sqrt(variance(mean));
}
void VMat::noise(double mean, double stddev)
{
	srand((unsigned int)time(NULL));
	double sum = 0.0;
	double scalar = 1.0 / RAND_MAX;
	for(int i = 0; i < s3; i++)
	{
		sum = 0.0;
		for(int j = 0; j < 12; j++)
		{
			sum += scalar * (double)rand();
		}
		sum -= 6.0;
		sum *= stddev;
		sum += mean;
		data[i] = (float)sum;
	}
}
void VMat::noise2(double mean, double stddev)
{
	double R1 = 0.0, R2 = 0.0;
	double scalar = 1.0 / RAND_MAX;
	double bs = log(exp(1.0));
	for(int i = 0; i < s3; i++)
	{
		R1 = scalar * (double)rand();
		R2 = scalar * (double)rand();
		R1 = pow(-2.0 * (log(R1) / bs), 0.5) * cos(2.0 * M_PI * R2);
		data[i] = (float)((R1*stddev) + mean);
	}
}
void VMat::add_noise(VMat & ns)
{
	if(s != ns.s) return;
	for(int i = 0; i < s3; i++)
	{
		data[i] += ns.data[i];
	}
}
float VMat::average_pixel_non_zeros()
{
	float sum = 0;
	int counter = 0;
	for(int i = 0; i < s3; i++)
	{
		if(data[i] == 0.0f)
		{
			continue;
		}
		sum += data[i];
		counter++;
	}
	return sum / (float)counter;
}
inline bool VMat::inbounds(int x, int y, int z)
{
	return x>=0 && y>=0 && z>=0 && x<s && y<s && z<s;
}
inline Point3i VMat::round_point(R3 & r)
{
	return Point3i((int)round(r.x), (int)round(r.y), (int)round(r.z));
}
void VMat::setAll(float value)
{
	for(int i = 0; i < s3; i++)
	{
		data[i] = value;
	}
}
int VMat::count(float threshold)
{
	int rv = 0;
	for(int i = 0; i < s3; i++)
	{
		rv += (data[i] > threshold) ? 1 : 0;
	}
	return rv;
}
SIObj VMat::siobj(int block_wid, float threshold)
{
	int cnt = count(threshold);
	float block_widf = (float) block_wid;
	SIObj rv(cnt*8, cnt*12);
	int vcount = 0, tcount = 0;
	R3 m[8];
	R3 xa(s / (float)block_wid, 0.0f, 0.0f);
	R3 ya(0.0f, xa.x, 0.0f);
	R3 za(0.0f, 0.0f, xa.x);
	float zf,yf,xf;
	float inc = xa.x;
	int xi,yi,zi, x,y,z;
	//cout << "inc = " << inc << endl;
	for(z = 0, zf = 0.0f; z < block_wid; z++, zf += inc)
	{
		for(y = 0, yf = 0.0f; y < block_wid; y++, yf += inc)
		{
			for(x = 0, xf = 0.0f; x < block_wid; x++, xf += inc)
			{
				xi = (int)round(xf); yi = (int)round(yf); zi = (int)round(zf);
				if(!inbounds(xi,yi,zi) || at(xi,yi,zi) <= threshold) continue;
				m[0] = R3((float)x, (float)y, (float)z);
				m[1] = m[0] + xa;
				m[2] = m[1] + ya;
				m[3] = m[0] + ya;

				m[4] = m[0] + za;
				m[5] = m[1] + za;
				m[6] = m[2] + za;
				m[7] = m[3] + za;
				//front
				rv._points[vcount]   = m[0];
				rv._points[vcount+1] = m[1];
				rv._points[vcount+2] = m[2];
				rv._points[vcount+3] = m[3];
				rv._points[vcount+4] = m[4];
				rv._points[vcount+5] = m[5];
				rv._points[vcount+6] = m[6];
				rv._points[vcount+7] = m[7];


				rv._triangles[tcount].a = vcount+0;
				rv._triangles[tcount].b = vcount+1;
				rv._triangles[tcount++].c = vcount+2;
					
				rv._triangles[tcount].a = vcount+0;
				rv._triangles[tcount].b = vcount+2;
				rv._triangles[tcount++].c = vcount+3;
				//left
				rv._triangles[tcount].a = vcount+1;
				rv._triangles[tcount].b = vcount+5;
				rv._triangles[tcount++].c = vcount+6;
				rv._triangles[tcount].a = vcount+1;
				rv._triangles[tcount].b = vcount+6;
				rv._triangles[tcount++].c = vcount+2;
				//back
				rv._triangles[tcount].a = vcount+4;
				rv._triangles[tcount].b = vcount+7;
				rv._triangles[tcount++].c = vcount+6;
				rv._triangles[tcount].a = vcount+4;
				rv._triangles[tcount].b = vcount+6;
				rv._triangles[tcount++].c = vcount+5;
				//left
				rv._triangles[tcount].a = vcount+0;
				rv._triangles[tcount].b = vcount+3;
				rv._triangles[tcount++].c = vcount+7;
				rv._triangles[tcount].a = vcount+0;
				rv._triangles[tcount].b = vcount+7;
				rv._triangles[tcount++].c = vcount+4;
				//top
				rv._triangles[tcount].a = vcount+3;
				rv._triangles[tcount].b = vcount+2;
				rv._triangles[tcount++].c = vcount+6;
				rv._triangles[tcount].a = vcount+3;
				rv._triangles[tcount].b = vcount+6;
				rv._triangles[tcount++].c = vcount+7;
				//bottom
				rv._triangles[tcount].a = vcount+0;
				rv._triangles[tcount].b = vcount+6;
				rv._triangles[tcount++].c = vcount+5;
				rv._triangles[tcount].a = vcount+0;
				rv._triangles[tcount].b = vcount+5;
				rv._triangles[tcount++].c = vcount+1;
				vcount += 8;
			}
		}
	}
	return rv;
}
void VMat::save(string fname)
{
	FILE * fi = fopen(fname.c_str(), "w");
	fwrite(&s, sizeof(int), 1, fi);
	fwrite(data, sizeof(float), s3, fi);
	fclose(fi);
}
void VMat::open(string fname)
{
	free();
	FILE * fi = fopen(fname.c_str(), "r");
	fread(&s, sizeof(int), 1, fi);
	setup_init();
	fread(data, sizeof(float), s3, fi);
	fclose(fi);
}
Mat VMat::translation_matrix(float x, float y, float z)
{
	float md[16] = {
	1.0f, 0.0f, 0.0f, x,
	0.0f, 1.0f, 0.0f, y,
	0.0f, 0.0f, 1.0f, z,
	0.0f, 0.0f, 0.0f, 1.0f,
	};
	return Mat(4, 4, CV_32FC1, md).clone();
}
Mat VMat::rotation_matrix_x(float angle)
{
	float cs = cos(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
	float sn = sin(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
	float md[16] = {
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, cs, -sn, 0.0f,
	0.0f, sn, cs, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f,
	};
	return Mat(4, 4, CV_32FC1, md).clone();
}
Mat VMat::rotation_matrix_y(float angle)
{
	float cs = cos(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
	float sn = sin(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
	float md[16] = {
	cs, 0.0f, sn, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	-sn, 0.0f, cs, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f,
	};
	return Mat(4, 4, CV_32FC1, md).clone();
}
Mat VMat::rotation_matrix_z(float angle)
{
	float cs = cos(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
	float sn = sin(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
	float md[16] = {
	cs, -sn, 0.0f, 0.0f,
	sn, cs, 0.0f, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f,
	};
	return Mat(4, 4, CV_32FC1, md).clone();
}
Mat VMat::rotation_matrix(float angleX, float angleY, float angleZ)
{
	return rotation_matrix_z(angleZ) * rotation_matrix_y(angleY) * rotation_matrix_x(angleX);
}
Mat VMat::scale_matrix(float sx, float sy, float sz)
{
	float md[16] = {
	sx, 0.0f, 0.0f, 0.0f,
	0.0f, sy, 0.0f, 0.0f,
	0.0f, 0.0f, sz, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f,
	};
	return Mat(4, 4, CV_32FC1, md).clone();
}
inline Mat VMat::center_origin_matrix_inv(int s)
{
	float nhw = ((float)s) * -0.5f;
	return translation_matrix(nhw, nhw, nhw);
}
inline Mat VMat::center_origin_matrix(int s)
{
	float nhw = ((float)s) * 0.5f;
	return translation_matrix(nhw, nhw, nhw);
}
Mat VMat::rotation_matrix_center(int volume_width, float rx, float ry, float rz)
{
	return center_origin_matrix(volume_width) * rotation_matrix(rx, ry, rz) * center_origin_matrix_inv(volume_width);
}
Mat VMat::scale_matrix_center(int volume_width, float sc)
{
	return center_origin_matrix(volume_width) * scale_matrix(sc) * center_origin_matrix_inv(volume_width);
}
Mat VMat::scale_matrix_center(int volume_width, R3 sc)
{
	return center_origin_matrix(volume_width) * scale_matrix(sc.x, sc.y, sc.z) * center_origin_matrix_inv(volume_width);
}
Mat VMat::transformation_matrix(int volume_width, float rx, float ry, float rz, float sc, float tx, float ty, float tz)
{
	return translation_matrix(tx, ty, tz) * scale_matrix_center(volume_width, sc) * rotation_matrix_center(volume_width, rx, ry, rz);
}
Mat VMat::scale_matrix(float us)
{
	float md[16] = {
	us, 0.0f, 0.0f, 0.0f,
	0.0f, us, 0.0f, 0.0f,
	0.0f, 0.0f, us, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f,
	};
	return Mat(4, 4, CV_32FC1, md).clone();
}
VMat VMat::clone()
{
	return VMat(*this);
}
inline void VMat::transform_point(Mat & m, R3 & inp)
{
	R3 x = inp;
	inp.x = x.x*m.at<float>(0,0) + x.y*m.at<float>(0,1) + x.z*m.at<float>(0,2) + m.at<float>(0,3);
	inp.y = x.x*m.at<float>(1,0) + x.y*m.at<float>(1,1) + x.z*m.at<float>(1,2) + m.at<float>(1,3);
	inp.z = x.x*m.at<float>(2,0) + x.y*m.at<float>(2,1) + x.z*m.at<float>(2,2) + m.at<float>(2,3);
}
void VMat::transform_volume(Mat & m, float nothing)
{
	Mat invm = m.inv();
	VMat cp = clone();
	setAll(nothing);
	for(int z = 0; z < s; z++)
	{
		for(int y = 0; y < s; y++)
		{
			for(int x = 0; x < s; x++)
			{
				R3 pf((float)x, (float)y, (float)z);
				transform_point(invm, pf);
				at(x,y,z) = cp.at(pf);
			}
		}
	}
}
void VMat::transform_volume_forward(Mat & m, float nothing)
{
	VMat cp = clone();
	setAll(nothing);
	int xi, yi, zi;
	for(int z = 0; z < s; z++)
	{
		for(int y = 0; y < s; y++)
		{
			for(int x = 0; x < s; x++)
			{
				R3 pf((float)x, (float)y, (float)z);
				transform_point(m, pf);
				pf.round();
				if(inbounds((int)pf.x, (int)pf.y, (int)pf.z))
					at((int)pf.x,(int)pf.y,(int)pf.z) = cp.at(x,y,z);
			}
		}
	}
}
void VMat::transform_volume(float rx, float ry, float rz, float sc, float tx, float ty, float tz)
{
	Mat m = transformation_matrix(s, rx,ry,rz,sc,tx,ty,tz);
	transform_volume(m);
}
void VMat::transform_volume_forward(float rx, float ry, float rz, float sc, float tx, float ty, float tz)
{
	Mat m = transformation_matrix(s, rx,ry,rz,sc,tx,ty,tz);
	transform_volume_forward(m);
}
vector<R3> VMat::collect_points(float th)
{
	vector<R3> rv;
	for(int z = 0; z < s; z++)
	{
		for(int y = 0; y < s; y++)
		{
			for(int x = 0; x < s; x++)
			{
				if((*this)(x,y,z) > th)
				{
					rv.push_back(R3((float)x,(float)y,(float)z));
				}
			}
		}
	}
	return rv;
}
Pixel3DSet VMat::pixel3dset(float zero)
{
	Pixel3DSet rv;
	float mn, mx;
	this->max_min_loc(mn, mx);
	float dv = mx-mn;
	Vec3b col;
	unsigned char v_uc;
	for(int z = 0; z < s; z++)
	{
		for(int y = 0; y < s; y++)
		{
			for(int x = 0; x < s; x++)
			{
				float v = (*this)(x,y,z);
				v -= mn;
				v /= dv;
				if(v - zero > 0.01f)
				{
					v *= 255.0f;
					v_uc = (unsigned char) v;
					col[0] = v_uc;
					col[1] = v_uc;
					col[2] = v_uc;
					rv.push_back(R3((float)x,(float)y,(float)z), col);
				}
			}
		}
	}
	return rv;
}
Mat VMat::getRYSMat(int volume_width, float sca, float ry)
{
	ry /= 57.2957795f;
	float T = (float) (volume_width/2);
	float iT = -T;
	float n = sin(ry);
	float c = cos(ry);
	float mt[16] = {sca*c, 0.0f, sca*n, iT*sca*(c+n) + T,
	0.0f, sca, 0.0f, iT*sca + T,
	-n*sca, 0.0f, sca*c, iT*sca*(c-n) + T,
	0.0f, 0.0f, 0.0f, 1.0f};
	return Mat(4, 4, CV_32FC1, mt).clone();
}
Mat VMat::getRYSMat_gpu(int volume_width, float sca, float ry)
{
	sca = 1.0f / sca;
	ry /= -57.2957795f;
	float T = (float) (volume_width/2);
	float iT = -T;
	float n = sin(ry);
	float c = cos(ry);
	float mt[16] = {sca*c, 0.0f, sca*n, iT*sca*(c+n) + T,
	0.0f, sca, 0.0f, iT*sca + T,
	-n*sca, 0.0f, sca*c, iT*sca*(c-n) + T,
	0.0f, 0.0f, 0.0f, 1.0f};

	return Mat(4, 4, CV_32FC1, mt).clone();
}
void VMat::save_obj(string fname, int objWid, float threshold)
{
	SIObj ob = siobj(objWid, threshold);
	ob.saveOBJ(fname);
}
void VMat::max_min_loc(float & minimum, float & maximum, Point3i * minimumLoc, Point3i * maximumLoc)
{
	Point3i a, b;
	minimum = data[0];
	maximum = data[0];
	a = b = Point3i(0,0,0);
	for(int z = 0; z < s; z++)
	{
		for(int y = 0; y < s; y++)
		{
			for(int x = 0; x < s; x++)
			{
				float v = at(x,y,z);
				if(minimum > v)
				{
					minimum = v;
					a.x = x;
					a.y = y;
					a.z = z;
				}
				if(maximum < v)
				{
					maximum = v;
					b.x = x;
					b.y = y;
					b.z = z;
				}
			}
		}
	}
	if(minimumLoc != NULL)
	{
		*minimumLoc = a;
	}
	if(maximumLoc != NULL)
	{
		*maximumLoc = b;
	}
}
Mat VMat::slice(int z)
{
	Mat mv = Mat::zeros(Size(s, s), CV_32FC1);
	for(int y = 0; y < s; y++)
	{
		for(int x = 0; x < s; x++)
		{
			mv.at<float>(Point2i(x,y)) = (float)at(x,y,z);
		}
	}
	double mx, mn;
	minMaxLoc(mv, &mn, &mx);
	mv -= mn;
	mv /= (mx-mn);
	return mv.clone();
}
double VMat::mse(VMat & mIn)
{
	if(s != mIn.s) return DBL_MAX;
	double rv = 0.0, tmp;
	for(int i = 0; i < s3; i++)
	{
		tmp = ((double)data[i])-((double)mIn.data[i]);
		rv += tmp*tmp;
	}
	return sqrt(rv) / (double)s3;
}
double VMat::correlate(VMat & mIn, double scale)
{
	if(s != mIn.s) return 0.0;
	int counter = 0;
	double rv = 0.0;
	double avg = (double)this->average_pixel_non_zeros();
	double mInavg = (double)mIn.average_pixel_non_zeros();
	for(int i = 0; i < s3; i++)
	{
		if(data[i] == 0.0f)
		{
			continue;
		}
		rv += (data[i]-avg) * (mIn.data[i]-mInavg);
		counter++;
	}
	if(counter == 0) return 0.0;
	return (rv / (double)counter) * scale;
}
double VMat::correlate2(VMat & mIn, double scale)
{
	if(s != mIn.s) return 0.0;
	normalize();
	mIn.normalize();
	double rv = 0.0;
	double avg = (double)this->average_pixel();
	double mInavg = (double)mIn.average_pixel();
	for(int i = 0; i < s3; i++)
	{
		rv += (data[i]-avg) * (mIn.data[i]-mInavg);
	}
	return (rv / (double)s3) * scale;
}
double VMat::meandifference(VMat & mIn)
{
	if(s != mIn.s) return DBL_MAX;
	double rv = 0.0;
	//int counter = 0;
	for(int i = 0; i < s3; i++)
	{
			
		if((data[i] < 0.00001f && data[i] > -0.00001f) || (mIn.data[i] < 0.00001f && mIn.data[i] > -0.00001f))
		{
			rv += abs( 
					((double)data[i])
				-   ((double)mIn.data[i])
			);
		}else
		{
			rv += 2.0 * abs( 
						((double)data[i])
					-   ((double)mIn.data[i])
				);
		}
		//counter++;
			
	}
	//cout << "counter: " << counter << " :: " << s3 << " >> " << (counter / (double) s3) << endl;
	return rv / (double)s3;
}
double VMat::meandifference2(VMat & mIn)
{
	if(s != mIn.s) return DBL_MAX;
	double rv = 0.0;
	for(int i = 0; i < s3; i++)
	{
		rv += abs( 
				((double)data[i])
			-   ((double)mIn.data[i])
		);
	}
	return rv / (double)s3;
}
void VMat::box_filter(int am)
{
	VMat other = s;
	int total_ = (am*2+1);
	total_ *= total_;
	for(int z = 0; z < s; z++)
	{
		for(int y = 0; y < s; y++)
		{
			for(int x = 0; x < s; x++)
			{
				if(x < am || y < am || z < am || x >= (s-am) || y >= (s-am) || z >= (s-am))
				{
					continue;
				}
				float sum = 0;
				for(int zi = z-am; zi <= z+am; zi++)
					for(int yi = y-am; yi <= y+am; yi++)
						for(int xi = x-am; xi <= x+am; xi++)
							sum += at(xi, yi, zi);
				sum /= total_;
				other.at(x, y, z) = sum;
			}
		}
	}
	*this = other;
}

void VMat::normalize()
{
	float mn, mx;
	max_min_loc(mn, mx);
	float mxx = mx-mn;
	for(int i = 0; i < s3; i++)
	{
		data[i] -= mn;
		data[i] /= mxx;
	}
}

double VMat::noise(double range)
{
	srand(time(NULL));
	double scalar = 1.0 / (double)RAND_MAX;
	double hr = (double)range * 0.5;
	

	double random_value = scalar * ((double)rand()) * range - hr; // random value between -range/2 and range/2
	double signal_value = (double)data[0];

	Point2d sig_(signal_value), noise_(random_value);

	data[0] += (float) random_value;

	for(int i = 1; i < s3; i++)
	{
		random_value = scalar * ((double)rand()) * range - hr; // random value between -range/2 and range/2
		signal_value = (double)data[i];

		sig_.x = (sig_.x > signal_value)? signal_value : sig_.x;
		sig_.y = (sig_.y < signal_value)? signal_value : sig_.y;

		noise_.x = (noise_.x > random_value)? random_value : noise_.x;
		noise_.y = (noise_.y < random_value)? random_value : noise_.y;


		data[i] += (float) random_value;
	}

	double signal_amplitude = abs(sig_.y - sig_.x);
	double noise_amplitude = abs(noise_.y - noise_.x);
	//cout << "amps: " << Point2d(signal_amplitude, noise_amplitude);
	return (20.0 * log10(signal_amplitude / noise_amplitude));
}

void VMat::threshold(float th)
{
	for(int i = 0; i < s3; i++)
	{
		if(data[i] < th)
			data[i] = 0.0f;
	}
}

Mat VMat::luke_dimensionsion_reduce_1a(float threshold, Size s, float min_radius)
{
	Mat count = Mat::zeros(s, CV_32FC1);
	Mat rv = Mat::zeros(s, CV_32FC1);
	int hw = this->s /2;
	R3 center = R3(1.0f,1.0f,1.0f) * (float)hw;
	float scalar_1 = s.width / (float) 360.0f;
	float scalar_2 = s.height / (float) 180.0f;
	float v, a1, a2;
	float mag;
	R3 p;
	int xi, yi;
	for(int z = 0; z < this->s; z++)
	{
		for(int y = 0; y < this->s; y++)
		{
			for(int x = 0; x < this->s; x++)
			{
				v = (*this)(x,y,z);
				if(v >= threshold)
				{
					p.x = (float)(x-hw); p.y = (float)(y-hw); p.z = (float)(z-hw);
					mag = p.mag();
					if(mag < min_radius) continue;
					p /= mag;
					p.get_dual_angles(a1, a2);
					xi = (int)((a1 * scalar_1)+0.5f);
					yi = (int)((a2 * scalar_2)+0.5f);
					//float rad = p.mag();
					if(xi >= 0 && yi >= 0 && xi < s.width && yi < s.height)
					{
						xi = (s.width-1) - xi;
						count.at<float>(yi,xi) +=1.0f;
						rv.at<float>(yi,xi) += v;
					}
				}
			}
		}
	}
	divide(rv, count, rv);
	ll_normalize(rv);
	return rv.clone();
}

float VMat::fast_y_rotation_estimation_1a(VMat & signal_2, Size s, float mr)
{
	Mat im1 = luke_dimensionsion_reduce_1a(0.01f, s);
	Mat im2 = signal_2.luke_dimensionsion_reduce_1a(0.01f, s, mr);
	Mat ham = ll_hanning_window(im1.size());
	ll_normalize(im1);
	ll_normalize(im2);
	im1 = im1.mul(ham);
	im2 = im2.mul(ham);
	double sw = (double)s.width;
	function<double(double)> f = [sw](double x)->double{ return x * (360.0 / sw);};
	Point2d rv = ll_phase_correlate(im1, im2);
	return (float)f(rv.x);
}

Mat VMat::luke_dimension_reduce_2a(float threshold, int axis, Size s)
{
	Mat rv = Mat::zeros(s, CV_32FC1);
	Mat count = rv.clone();
	float v;
	float scalar_1 = s.width / (float)this->s;
	float scalar_2 = s.height / (float)this->s;
	int xi, yi;

	if(axis == 1)
	{
		
		for(int z = 0; z < this->s; z++)
		{
			for(int x = 0; x < this->s; x++)
			{
				for(int y = 0; y < this->s; y++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(x * scalar_1);
						yi = (int)round(z * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) += v;
							count.at<float>(yi, xi) += 1.0f;
						}
					}
				}
			}
		}
	}else if(axis == 2)
	{
		for(int y = 0; y < this->s; y++)
		{
			for(int x = 0; x < this->s; x++)
			{
				for(int z = 0; z < this->s; z++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(x * scalar_1);
						yi = (int)round(y * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) += v;
							count.at<float>(yi, xi) += 1.0f;
						}
					}
				}
			}
		}
	}else
	{
		for(int z = 0; z < this->s; z++)
		{
			for(int y = 0; y < this->s; y++)
			{
				for(int x = 0; x < this->s; x++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(z * scalar_1);
						yi = (int)round(y * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) += v;
							count.at<float>(yi, xi) += 1.0f;
						}
					}
				}
			}
		}
	}


	divide(rv, count, rv);
	ll_normalize(rv);
	return rv.clone();
}

R3 VMat::luke_fast_translation_estimation_2a(VMat & signal_2, Size s)
{
	R3 rv;
	Mat im12 = this->luke_dimension_reduce_2a(0.01f, 2, s);
	Mat im10 = this->luke_dimension_reduce_2a(0.01f, 0, s);
	Mat im20 = signal_2.luke_dimension_reduce_2a(0.01f, 0, s);
	Mat im22 = signal_2.luke_dimension_reduce_2a(0.01f, 2, s);

	Mat ham = ll_hanning_window(im10.size());
	ll_normalize(im12);
	ll_normalize(im10);
	ll_normalize(im22);
	ll_normalize(im20);
	im12 = im12.mul(ham);
	im10 = im10.mul(ham);
	im22 = im22.mul(ham);
	im20 = im20.mul(ham);

	Point2d pc1 = ll_phase_correlate(im10, im20);
	Point2d pc2 = ll_phase_correlate(im12, im22);

	rv.z = pc1.x * (this->s / (float) im10.size().width);
	rv.x = pc2.x * (this->s / (float) im12.size().width);
	rv.y = pc2.y * (this->s / (float) im12.size().height);
	return rv;
}

Mat VMat::luke_dimension_reduce_3a(float threshold, int axis, Size s)
{
	Mat rv = Mat::zeros(s, CV_32FC1);
	float v;
	float scalar_1 = s.width / (float)this->s;
	float scalar_2 = s.height / (float)this->s;
	int xi, yi;

	if(axis == 1)
	{
		
		for(int z = 0; z < this->s; z++)
		{
			for(int x = 0; x < this->s; x++)
			{
				for(int y = 0; y < this->s; y++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(x * scalar_1);
						yi = (int)round(z * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) = v;
							continue;
						}
					}
				}
			}
		}
	}else if(axis == 2)
	{
		for(int y = 0; y < this->s; y++)
		{
			for(int x = 0; x < this->s; x++)
			{
				for(int z = 0; z < this->s; z++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(x * scalar_1);
						yi = (int)round(y * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) = v;
							continue;
						}
					}
				}
			}
		}
	}else
	{
		for(int z = 0; z < this->s; z++)
		{
			for(int y = 0; y < this->s; y++)
			{
				for(int x = 0; x < this->s; x++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(z * scalar_1);
						yi = (int)round(y * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) = v;
							continue;
						}
					}
				}
			}
		}
	}


	ll_normalize(rv);
	return rv.clone();
}

Mat VMat::luke_dimension_reduce_4a(float threshold, int axis, Size s)
{
	Mat rv = Mat::zeros(s, CV_32FC1);
	float v;
	float scalar_1 = s.width / (float)this->s;
	float scalar_2 = s.height / (float)this->s;
	int xi, yi;

	if(axis == 1)
	{
		
		for(int z = 0; z < this->s; z++)
		{
			for(int x = 0; x < this->s; x++)
			{
				for(int y = 0; y < this->s; y++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(x * scalar_1);
						yi = (int)round(z * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) = y;
							continue;
						}
					}
				}
			}
		}
	}else if(axis == 2)
	{
		for(int y = 0; y < this->s; y++)
		{
			for(int x = 0; x < this->s; x++)
			{
				for(int z = 0; z < this->s; z++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(x * scalar_1);
						yi = (int)round(y * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) = z;
							continue;
						}
					}
				}
			}
		}
	}else
	{
		for(int z = 0; z < this->s; z++)
		{
			for(int y = 0; y < this->s; y++)
			{
				for(int x = 0; x < this->s; x++)
				{
					v = (*this)(x,y,z);
					if(v >= threshold)
					{
						xi = (int)round(z * scalar_1);
						yi = (int)round(y * scalar_2);
						if(xi >= 0 && xi < s.width && yi >= 0 && yi < s.height)
						{
							rv.at<float>(yi, xi) = x;
							continue;
						}
					}
				}
			}
		}
	}


	ll_normalize(rv);
	return rv.clone();
}


void VMat::clean(float dist)
{
	for(int i = 0; i < s3; i++)
	{
		if(abs(data[i]-0.0f) < dist)
			data[i] = 0.0f;
	}
}

void VMat::edge_detect(float scalar, bool normalize_after)
{
	VMat v = clone();
	setAll(0.0f);
	for(int z = 0; z < s-1; z++)
	{
		for(int y = 0; y < s-1; y++)
		{
			for(int x = 0; x < s-1; x++)
			{
				float p = v(x,y,z);
				at(x,y,z) = R3(v(x+1,y,z)-p,v(x,y+1,z)-p,v(x,y,z+1)-p).mag() * scalar;
			}
		}
	}
	if(normalize_after) normalize();
}


string VMat::bin_string(int num_bins)
{
	vector<int> counts(num_bins, 0);
	float mn, mx;
	max_min_loc(mn, mx);
	float mx_mn = mx - mn;
	for(int z = 0; z < s3; z++)
	{
		float v = data[z];
		v -= mn;
		v /= (mx_mn);
		v *= (float)num_bins;
		int v_ = (int) round(v);
		if(v_ >= 0 && v < num_bins) counts[v_]++;
	}
	string rv = "";
	for(int i = 0; i < num_bins; i++)
	{
		rv += "bin [" + std::to_string((mn + (mx_mn/(float)num_bins) * i)) + " - " + std::to_string(mn + (mx_mn/(float)num_bins) * (i+1)) + " = " + std::to_string(counts[i]) + " ";
		if(i < num_bins - 1) rv += ", ";
	}
	return rv;
}

R3 VMat::center_location()
{
	float sf = s / 2.0f;
	return R3(sf, sf, sf);
}

//static methods of VMat:

Mat VMat::pca_lukes_pc_t(VMat & v1, VMat & v2, bool edge_detect, Size s)
{
	Mat m1 = VMat::pca(v1, v2, edge_detect);
	VMat tmp = v1.clone();
	tmp.transform_volume_forward(m1);
	Mat m2;
	R3 translation = tmp.luke_fast_translation_estimation_2a(v2, s);
	m2 = VMat::transformation_matrix(tmp.s, 0.0f, 0.0f, 0.0f, 1.0f, translation.x, translation.y, translation.z);
	Mat rv = m2 * m1;
	return rv.clone();
}

Mat VMat::pca(VMat & v1, VMat & v2, bool edge_detect, float clean_amount)
{
	VMat s1 = v1.clone();
	VMat s2 = v2.clone();
	if(edge_detect)
	{
		s1.edge_detect(2.0f, false);
		s2.edge_detect(2.0f, false);
	}

	Pixel3DSet p = s1.pixel3dset();
	Pixel3DSet p2 = s2.pixel3dset();
	Mat m = ll_algorithms::ll_pca_3d::LPCA::compute_alignment_for_pc(p, p2);
	return m.clone();
}



Mat VMat::pca_double(VMat & v1, VMat & v2, bool edge_detect, float clean_amount)
{
	VMat s1 = v1.clone();
	Mat m1 = VMat::pca(s1, v2, edge_detect, clean_amount);
	s1.transform_volume_forward(m1);
	Mat m2 = VMat::pca(s1, v2, edge_detect, clean_amount);
	Mat m = m2 * m1;
	return m.clone();
}

void VMat::or(VMat & v1, VMat & v2, float threshold)
{
	for(int i = 0; i < v1.s3; i++)
	{
		if(v1.data[i] >= threshold || v2.data[i] >= threshold)
		{
			v1.data[i] = max(v1.data[i], v2.data[i]);
		}
	}
}

void VMat::swap_quadrants(VMat & v)
{
	int w = v.s;
	int hw = w/2;
	//swap first two quads
	for(int z = 0; z < hw; z++)
	{
		for(int y = 0; y < hw; y++)
		{
			for(int x = 0; x < hw; x++)
			{
				float a = v(x,y,z);
				v(x,y,z) = v(x+hw,y+hw,z+hw);
				v(x+hw,y+hw,z+hw) = a;
			}
		}
	}
	//swap second 2 quads
	for(int z = 0; z < hw; z++)
	{
		for(int y = 0; y < hw; y++)
		{
			for(int x = hw; x < w; x++)
			{
				float a = v(x,y,z);
				v(x,y,z) = v(x-hw,y+hw,z+hw);
				v(x-hw,y+hw,z+hw) = a;
			}
		}
	}
	//swap next two quads
	for(int z = 0; z < hw; z++)
	{
		for(int y = hw; y < w; y++)
		{
			for(int x = 0; x < hw; x++)
			{
				float a = v(x,y,z);
				v(x,y,z) = v(x+hw,y-hw,z+hw);
				v(x+hw,y-hw,z+hw) = a;
			}
		}
	}

	//swap final two quads
	for(int z = 0; z < hw; z++)
	{
		for(int y = hw; y < w; y++)
		{
			for(int x = hw; x < w; x++)
			{
				float a = v(x,y,z);
				v(x,y,z) = v(x-hw,y-hw,z+hw);
				v(x-hw,y-hw,z+hw) = a;
			}
		}
	}
}

void VMat::log_transform(VMat & v)
{
	for(int i = 0; i < v.s3; i++)
	{
		v.data[i] = log(v.data[i]);
	}
}

Point3i VMat::filter_pc(Point3i a, int s)
{
	return Point3i(filter_pc(a.x,s), filter_pc(a.y,s), filter_pc(a.z,s));
}

int VMat::filter_pc(int a, int s)
{
	return (a > s/2) ? s-a: -a;
}

float VMat::log_polar_scalar(float s)
{
	return ((float) s) / log(((float) s) / 2.56f);
}

void VMat::filter_pc(Point3i pc, float & rotation, float & scale, int N)
{
	R3 q((float)pc.x, (float)pc.y, (float) pc.z);
	q.x *= (-360.0f / (float)N);
	q.y *= (180.0f / (float)N);
	q.z /= log_polar_scalar((float)N);
	q.z = exp(q.z);

	rotation = q.x;
	scale = 1.0f /  q.z;
}

void VMat::log_polar(R3 & p, int s)
{
	float sf = (float)s;
	float hw = sf * 0.5f;
	p -= R3(hw, hw, hw);
		
	float mag = p.mag();
	float a1, a2;
	p.normalize();
	p.get_dual_angles(a1, a2);
	a1 /= 360.0f;
	a1 *= sf;
	a2 /= 180.0f;
	a2 *= sf;
	float M = log_polar_scalar(sf);
	mag = M * log(mag);

	p.x = a1;
	p.y = a2;
	p.z = mag;
}

void VMat::log_polar_inv(R3 & p, int s)
{
	float sf = (float) s;
	p.x /= sf;
	p.x *= 360.0f;
	p.y /= sf;
	p.y *= 180.0f;
	float M = log_polar_scalar(sf);
	p.z /= M;
	M = exp(p.z);
	p.set_from_dual_angles(p.x, p.y);
	p *= M;
	float hw = sf * 0.5f;
	p += R3(hw, hw, hw);
}

void VMat::log_polar(VMat & s)
{
	VMat s2 = s;
	for(int z = 0; z < s2.s; z++)
	{
		for(int y = 0; y < s2.s; y++)
		{
			for(int x = 0; x < s2.s; x++)
			{
				R3 p((float)x, (float)y, (float)z);
				log_polar_inv(p, s.s);
				s(x,y,z) = s2.at(p);
			}
		}
	}
}

void VMat::log_polar_inv(VMat & s)
{
	VMat s2 = s;
	for(int z = 0; z < s.s; z++)
	{
		for(int y = 0; y < s.s; y++)
		{
			for(int x = 0; x < s.s; x++)
			{
				R3 p((float)x, (float)y, (float)z);
				log_polar(p, s.s);
				s(x,y,z) = s2(p);
			}
		}
	}
}

Mat VMat::new_up_mat(R3 new_up, R3 center)
{
	Mat m1 = correct_volume_up_vector_rotation_matrix(center, center + new_up * 10.0f);
	Mat translation_to_point = VMat::translation_matrix(-center.x, -center.y, -center.z);
	Mat translation_back = VMat::translation_matrix(center.x, center.y, center.z);
	Mat rv = translation_back * m1 * translation_to_point;
	return rv.clone();
}

Mat VMat::new_right_mat(R3 new_up, R3 center)
{
	Mat m1 = correct_volume_right_vector_rotation_matrix(center, center + new_up * 10.0f);
	Mat translation_to_point = VMat::translation_matrix(-center.x, -center.y, -center.z);
	Mat translation_back = VMat::translation_matrix(center.x, center.y, center.z);
	Mat rv = translation_back * m1 * translation_to_point;
	return rv.clone();
}

Mat VMat::correction_matrix_up_axis(R3 p1a, R3 p1b, R3 p2a, R3 p2b)
{
	Mat m1 = correct_volume_up_vector_rotation_matrix(p1a, p1b);
	Mat m2 = correct_volume_up_vector_rotation_matrix(p2a, p2b);
	Mat translation_to_point = VMat::translation_matrix(-p1a.x, -p1a.y, -p1a.z);
	Mat translation_back = VMat::translation_matrix(p2a.x, p2a.y, p2a.z);
	m2 = m2.inv();
	Mat rv = translation_back * m2 * m1 * translation_to_point;
	return rv.clone();
}

/*

r = up * fr * up
u = up * (fr * up) * up

100
010

010
001

r+up
100
010



*/

Mat VMat::correct_volume_up_vector_rotation_matrix(R3 p1a, R3 p1b)
{
	R3 up = (p1b - p1a).unit();
	R3 tmp_up(0.0f, 1.0f, 0.0f);
	R3 tmp_right(1.0f, 0.0f, 0.0f);
	tmp_right = (up * tmp_up < up * tmp_right) ? tmp_up : tmp_right;
	R3 forward = up ^ tmp_right;
	forward.normalize();
	R3 right = up ^ forward;
	right.normalize();
	forward = right ^ up;
	forward.normalize();
	float arr[] = 
	{
		right.x,	up.x,	forward.x,	0.0f,
		right.y,	up.y,	forward.y,	0.0f,
		right.z,	up.z,	forward.z,	0.0f,
		0.0f,		0.0f,	0.0f,		1.0f
	};

	Mat rotation_matrix(4, 4, CV_32FC1, arr);

	rotation_matrix = rotation_matrix.inv();
	return rotation_matrix.clone();
}


Mat VMat::correction_matrix_right_axis(R3 p1a, R3 p1b, R3 p2a, R3 p2b)
{
	Mat m1 = correct_volume_right_vector_rotation_matrix(p1a, p1b);
	Mat m2 = correct_volume_right_vector_rotation_matrix(p2a, p2b);
	Mat translation_to_point = VMat::translation_matrix(-p1a.x, -p1a.y, -p1a.z);
	Mat translation_back = VMat::translation_matrix(p2a.x, p2a.y, p2a.z);
	m2 = m2.inv();
	Mat rv = translation_back * m2 * m1 * translation_to_point;
	return rv.clone();
}

Mat VMat::correct_volume_right_vector_rotation_matrix(R3 p1a, R3 p1b)
{
	R3 up = (p1b - p1a).unit();
	R3 tmp_up(0.0f, 1.0f, 0.0f);
	R3 tmp_right(1.0f, 0.0f, 0.0f);
	tmp_right = (up * tmp_up < up * tmp_right) ? tmp_up : tmp_right;
	R3 forward = up ^ tmp_right;
	forward.normalize();
	R3 right = up ^ forward;
	right.normalize();
	forward = right ^ up;
	forward.normalize();
	float arr[] = 
	{
		up.x,	right.x,	forward.x,	0.0f,
		up.y,	right.y,	forward.y,	0.0f,
		up.z,	right.z,	forward.z,	0.0f,
		0.0f,	0.0f,		0.0f,		1.0f
	};

	Mat rotation_matrix(4, 4, CV_32FC1, arr);

	rotation_matrix = rotation_matrix.inv();
	return rotation_matrix.clone();
}

Mat VMat::pca_correct_right()
{
	Pixel3DSet p = pixel3dset();
	ll_algorithms::ll_pca_3d::LPCA pc1(p, 0.2f, ll_algorithms::ll_pca_3d::LPCA::COMPUTE_2);
	R3 newUp = pc1.eigenvecs[0];
	R3 center = pc1.mean;
	Mat m = new_right_mat(newUp, center);
	transform_volume_forward(m);
	return m.clone();
}

Mat VMat::pca_correct_up()
{
	Pixel3DSet p = pixel3dset();
	ll_algorithms::ll_pca_3d::LPCA pc1(p, 0.2f, ll_algorithms::ll_pca_3d::LPCA::COMPUTE_2);
	R3 newUp = pc1.eigenvecs[0];
	R3 center = pc1.mean;
	Mat m = new_up_mat(newUp, center);
	transform_volume_forward(m);
	return m.clone();
}
