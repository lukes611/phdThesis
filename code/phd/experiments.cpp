#include "experiments.h"

using namespace std;
#include "../basics/Pixel3DSet.h"

using namespace ll_pix3d;

namespace ll_experiments
{

void viewVideo(string name, bool viewColor, bool viewDepth, bool viewVD, int wks)
{
	CapturePixel3DSet video (name, 80);
	Pix3D p;
	Mat color, depth, vd;
	for(int i = 0; i < video.size(); i++)
	{
		video.read(p);
		
		if(viewColor)
		{
			p.colorImage(color);
			imshow("color", color);
		}

		if(viewDepth)
		{
			p.depthImage(depth);
			imshow("depth", depth);
		}

		if(viewVD)
		{
			p.vdImage(vd);
			imshow("vd",vd);
		}
		
		if(viewColor || viewDepth || viewVD)
		{
			if (wks > 0) waitKey(wks);
			else waitKey();
		}
	}
}

vector<int> rng(int to){ return rng(0, to, 1);}
vector<int> rng(int from, int to)
{
	return rng(from, to, 1);
}
vector<int> rng(int from, int to, int inc)
{
	vector<int> ret;
	if(from < to && inc > 0)
		for(int i = from; i < to; i+=inc) ret.push_back(i);
	else if(from > to && inc < 0) for(int i = from; i > to; i-=inc);
	return ret;
}


}

