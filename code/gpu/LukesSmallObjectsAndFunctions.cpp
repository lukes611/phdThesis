#include "LukesSmallObjectsAndFunctions.h"
#include "../basics/BitReaderWriter.h"
#include <time.h>
using namespace BIT_MANIPULATOR;
using namespace ll_pix3d;
using namespace ll_R3;
using namespace cv;
using namespace std;

//LT object:
LT::LT()
{
	secs = 0, msecs = 0;
}
LT::LT(const LT & input)
{
	a = input.a;
	b = input.b;
	secs = input.secs;
	msecs = input.msecs;
}
LT & LT::operator = (const LT & input)
{
	if(this == &input) return *this;
	a = input.a;
	b = input.b;
	secs = input.secs;
	msecs = input.msecs;
	return *this;
}
void LT::start()
{
	ftime(&a);
}
void LT::stop()
{
	ftime(&b);
	secs = static_cast<int>(b.time - a.time);
	msecs = (b.millitm - a.millitm);
}
void LT::print()
{
	printf("time: %u:%u", secs, msecs );
}



//phase correlation sollutions:

PCSolution::PCSolution(float _r, float _s, Point3i _t, double er)
{
	r = _r;
	s = _s;
	t = R3((float)_t.x, (float)_t.y, (float)_t.z);
	error = er;
	m = Pixel3DSet::transformation_matrix(0.0f, r, 0.0f, s, t.x, t.y, t.z, R3(192.0f,192.0f,192.0f));
}
PCSolution::PCSolution(float _r, float _s, R3 _t, double er)
{
	r = _r;
	s = _s;
	t = _t;
	error = er;
	m = Pixel3DSet::transformation_matrix(0.0f, r, 0.0f, s, t.x, t.y, t.z, R3(192.0f,192.0f,192.0f));
}
PCSolution::PCSolution(const PCSolution & a)
{
	r = a.r;
	s = a.s;
	t = a.t;
	error = a.error;
	m = a.m.clone();
}
PCSolution & PCSolution::operator = (const PCSolution & a)
{
	r = a.r;
	s = a.s;
	t = a.t;
	error = a.error;
	m = a.m.clone();
	return *this;
}
void PCSolution::print()
{
	cout << "Solution { rotation(" << r << "), scale(" << s << "), translation(" << t << ")\n";
}


//adding random noise:

unsigned char add_in_range(unsigned char v, double w)
{
	double rv = w + (double)v;
	rv = (rv > 255.0)? 255.0 : rv;
	rv = (rv < 0.0)? 0.0 : rv;
	return (unsigned char) rv;
}

Mat add_error_8uc3(Mat & input, float mean, float stddev)
{
	Mat rv = input.clone();
	Vec3b * pt = (Vec3b *)rv.data;
	int s = rv.size().width * rv.size().height;
	srand(time(NULL));
	double sum = 0.0;
	double scalar = 1.0 / RAND_MAX;
	for(int i = 0; i < s; i++, pt++)
	{
		sum = 0.0;
		for(int j = 0; j < 12; j++)
		{
			sum += scalar * (double)rand();
		}
		sum -= 6.0;
		sum *= stddev;
		sum += mean;
		double red = (((double)(*pt)[0]+sum));
		double green = (((double)(*pt)[1]+sum));
		double blue = (((double)(*pt)[2]+sum));
		red = (red > 255.0)? 255.0 : (red < 0.0)? 0.0 : red;
		green = (green > 255.0)? 255.0 : (green < 0.0)? 0.0 : green;
		blue = (blue > 255.0)? 255.0 : (blue < 0.0)? 0.0 : blue;

		(*pt)[0] = red;
		(*pt)[1] = green;
		(*pt)[2] = blue;
	}
	return rv;
}

Mat add_error_32fc1(Mat & input, float mean, float stddev)
{
	Mat rv = input.clone();
	float * pt = (float *)rv.data;
	int s = rv.size().width * rv.size().height;
	srand(time(NULL));
	double sum = 0.0;
	double scalar = 1.0 / RAND_MAX;
	for(int i = 0; i < s; i++, pt++)
	{
		if(pt[0] == 0.0f) continue;
		sum = 0.0;
		for(int j = 0; j < 12; j++)
		{
			sum += scalar * (double)rand();
		}
		sum -= 6.0;
		sum *= stddev;
		sum += mean;
		pt[0] = (float)((double)pt[0]+sum);
	}
	return rv;
}

Mat add_error_8uc3(Mat & input, float range, float * snr)
{
	Mat rv = input.clone();
	Vec3b * pt = (Vec3b *)rv.data;
	int s = rv.size().width * rv.size().height;
	srand(time(NULL));
	double scalar = 1.0 / RAND_MAX;
	double hr = (double)range * 0.5;
	double snrv = 0.0;
	for(int i = 0; i < s; i++, pt++)
	{
		double rn = (scalar * (double)rand()) * range - hr;
		double rn2 = (scalar * (double)rand()) * range - hr;
		double rn3 = (scalar * (double)rand()) * range - hr;
		double abs1 = abs(rn);
		abs1 = (abs1 != 0.0)? abs1 : 1.0;
		double abs2 = abs(rn2);
		abs2 = (abs2 != 0.0)? abs2 : 1.0;
		double abs3 = abs(rn3);
		abs3 = (abs3 != 0.0)? abs3 : 1.0;
		snrv += (double)(*pt)[0]/abs1;
		snrv += (double)(*pt)[1]/abs2;
		snrv += (double)(*pt)[2]/abs3;
		(*pt)[0] = add_in_range((*pt)[0], rn);
		(*pt)[1] = add_in_range((*pt)[1], rn);
		(*pt)[2] = add_in_range((*pt)[2], rn);
	}
	if(snr)
	{
		*snr = (float)snrv / (float)(s*3);
	}
	return rv;
	/*
	 255 / range
	*/
}
Mat add_error_32fc1(Mat & input, float range, float * snr)
{
	Mat rv = input.clone();
	float * pt = (float *)rv.data;
	int s = rv.size().width * rv.size().height;
	srand(time(NULL));
	double scalar = 1.0 / RAND_MAX;
	double hr = (double)range * 0.5;
	double snrv = 0.0;
	for(int i = 0; i < s; i++, pt++)
	{
		if(pt[0] == 0.0f) continue;
		double rn = scalar * (double)rand();
		rn *= range;
		rn -= hr;
		snrv += pt[0]/(abs(rn) != 0.0)?abs(rn) : 1.0;
		pt[0] = (float)((double)pt[0]+rn);
	}
	if(snr)
	{
		*snr = (float)snrv / (float)s;
	}
	return rv;
}



