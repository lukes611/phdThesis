#include "Licp.h"

using namespace ll_R3;

namespace Licp
{


Mat asMatRows(vector<R3> & inp)
{
	Mat ret = Mat::zeros(inp.size(), 4, CV_32FC1);
	for(int i = 0; i < inp.size(); i++)
	{
		ret.at<float>(i,0) = inp[i].x;
		ret.at<float>(i,1) = inp[i].y;
		ret.at<float>(i,2) = inp[i].z;
		ret.at<float>(i,3) = 1.0f;
	}
	return ret.clone();
}

}