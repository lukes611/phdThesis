#include "measurements.h"
#include "Licp.h"
using namespace ll_R3;
using namespace Licp;

namespace ll_measure
{

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


}