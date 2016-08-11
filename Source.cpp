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



//add time measurement and then speedup
//add outlier removal

template <class T>
void randomize(vector<T> & ar)
{
	int s = ar.size();
	auto rnum = [&s]() -> int {
		return rand() % s;
	};
	for(int i = 0; i < s*2; i++)
	{
		int a = rnum();
		int b = rnum();
		T x = ar[a];
		ar[a] = ar[b];
		ar[b] = x;
	}
}

int main(int argc, char * * argv)
{
	
	SIObj ob; ob.open_obj("C:/lcppdata/obj/bunny_simplified2.obj");

	Pixel3DSet p1(ob._points);

	p1.normalize(256);

	Pixel3DSet p2 = p1;

	randomize(p2.points);

	p2.transform_set(5.0f, 6.0f, 1.0f, 1.0f, 2.0f, 5.0f, 0.0f, R3(1.0f, 1.0f, 1.0f) * 0.5f * 256.0f);

	cout << "opened object bunny with : " << ob._points.size() << " no. points\n";

	double error, seconds;
	vector<R3> out;
	int iterations;
	vector<int> ii; vector<double> dd;
	error = Licp::closestPointsf(p1.points, p2.points, ii, dd);
	printf("error-in: %lf\n", error);
	printf("hausdorff %lf\n", hausdorff(p1.points, p2.points));
	printf("mse %lf\n", mse(p1.points, p2.points));
	printf("% match %lf\n", percentMatch(p1.points, p2.points));

	Licp::icp_outlierRemoval(p1.points, p2.points, out, error, seconds, iterations, 10.0);
	printf("error-out: %lf\n", error);
	printf("hausdorff %lf\n", hausdorff(out, p2.points));
	printf("mse %lf\n", mse(out, p2.points));
	printf("% match %lf\n", percentMatch(out, p2.points));
	/*vector<R3> p1, p2;
	int imSize = 256;

	Mat km = VMat::transformation_matrix(384, 2.0f, 1.0f, 0.0f, 1.0f, 1.0f, 2.0f, 1.0f);

	

	for (int i = 0; i < imSize * imSize; i++)
	{
		R3 point(rp(), rp(), rp());
		p1.push_back(point);
		R3 point2 = point;
		Pixel3DSet::transform_point(km, point2);
		p2.push_back(point2);
	}

	randomize(p1);

	bool PRINT = false;

	if(PRINT)
	{
		cout << "p1:" << endl;
		for(int i = 0; i < p1.size(); i++) cout << p1[i] << endl;
	}
	if(PRINT)
	{
		cout << "P2:" << endl;
		for(int i = 0; i < p2.size(); i++) cout << p2[i] << endl;
	}
	cout << "icp-ing...\n";

	double dist = DBL_MAX;

	vector<R3> out = p1; vector<int> tmp; vector<double> dst;
	double error = closestPoints(p1, p2, tmp, dst);

	

	cout << "ind-fas**********\n";
	

	cout << "error in: " << error << endl;
	double seconds; int iters;
	icp_outlierRemoval(p1, p2, out, error, seconds, iters);
	cout << "error: " << error << endl;
	cout << "number of iterations: " << iters << endl;
	cout << "seconds: " << seconds << endl;
	
	if(PRINT)
	{
		cout << "out:" << endl;
		for(int i = 0; i < out.size(); i++) cout << out[i] << endl;
	}
	if(PRINT)
	{
		cout << "P2:" << endl;
		for(int i = 0; i < p2.size(); i++) cout << p2[i] << endl;
	}

	
	*/

	return 0;
}