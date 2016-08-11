#include "measurements.h"
#include "Licp.h"
using namespace ll_R3;
using namespace Licp;

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
double avg(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b)
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
double mse(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b)
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


}