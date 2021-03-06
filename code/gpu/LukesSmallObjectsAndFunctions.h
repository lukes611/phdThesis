#ifndef LukesSmallObjectsAndFunctions
#define LukesSmallObjectsAndFunctions

#include <stdio.h>
#include <stdlib.h>
#include <sys/timeb.h>
#include <string>
#include <vector>
#include "../basics/locv3.h"
#include "../basics/Pixel3DSet.h"
#include "../basics/R3.h"

class LT
{
public:
	struct timeb a, b;
	int secs, msecs;
	LT();
	LT(const LT & input);
	LT & operator = (const LT & input);
	void start();
	void stop();
	void print();

};

class PCSolution
{
public:
	float r, s;
	ll_R3::R3 t;
	cv::Mat m;
	double error;
	PCSolution(float _r, float _s, cv::Point3i _t, double er);
	PCSolution(float _r, float _s, ll_R3::R3 _t, double er);
	PCSolution(const PCSolution & a);
	PCSolution & operator = (const PCSolution & a);
	void print();
};


template <class T>
std::vector<T> reverse_vec(std::vector<T> & v)
{
	std::vector<T> rv;
	for(int i = v.size()-1; i >= 0; i--)
	{
		rv.push_back(v[i]);
	}
	return rv;
}

cv::Mat add_error_8uc3(cv::Mat & input, float mean, float stddev);
cv::Mat add_error_32fc1(cv::Mat & input, float mean, float stddev);

cv::Mat add_error_8uc3(cv::Mat & input, float range, float * snr = NULL);
cv::Mat add_error_32fc1(cv::Mat & input, float range, float * snr = NULL);

unsigned char add_in_range(unsigned char v, double w);

#endif