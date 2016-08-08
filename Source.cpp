#include <iostream>
#include <string>
#include <vector>

#include "code\basics\locv3.h"
#include "code\basics\R3.h"
#include "code\basics\llCamera.h"
#include "code\basics\VMatF.h"


using namespace std;
using namespace ll_R3;
using namespace ll_cam;
//using namespace cv;

//proto-type for experiments: VMat pc(VMat a, VMat b, double & time, Mat & transform, double & mse);
//to-do:= pc, ipc, 

//a n^2 complexity search for the closest point for each point in src to dst,
//returns an array of mappings src[i] mapped to dst[array[i]]
// ipc wiki: https://en.wikipedia.org/wiki/Iterative_closest_point
// kind of good: http://www.mrpt.org/Iterative_Closest_Point_(ICP)_and_other_matching_algorithms
// //optimal resource: http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
// : least squares: https://en.wikipedia.org/wiki/Linear_least_squares_(mathematics)
//
void closestPoints(vector<R3> & src, vector<R3> & dst, vector<int> & indexes, vector<double> & distances)
{

}

float rp() {
	return (float)(rand() % 100);
}

int main(int argc, char * * argv)
{
	
	vector<R3> p1, p2;
	for (int i = 0; i < 5; i++)
	{
		R3 point(rp(), rp(), rp());
		p1.push_back(point);
		p2.push_back(point + R3(2, 0, 1));
	}

	for (int i = 0; i < p1.size(); i++)
		cout << p1[i] << " -> " << p2[i] << endl;


	vector<int> matches; vector<double> dists;
	closestPoints(p1, p2, matches, dists);



	//cout << endl;

	
	return 0;
}