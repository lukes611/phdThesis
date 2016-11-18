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
#include "code\phd\fmRansac.h"
#include "code\pc\TheVolumePhaseCorrelator.h"
#include "code\phd\Lpcr.h"
#include "code\script\LScript.h"
#include "code\phd\experiments.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
using namespace ll_experiments;





#include "code\basics\BitReaderWriter.h"

class Surfel {
public:
	R3 point;
	R3 normal;
	Vec3b color;
	Surfel(R3 p, R3 n, Vec3b c) {
		color = c;
		point = p;
		normal = n;
	}
	Surfel(const Surfel & s) {
		color = s.color;
		point = s.point;
		normal = s.normal;
	}
	Surfel & operator = (const Surfel & s) {
		color = s.color;
		point = s.point;
		normal = s.normal;
		return *this;
	}
};

class SurfelOTCube {
public:
	R3 corner;
	float size;
	vector<int> indices;
	vector<SurfelOTCube> children;
	SurfelOTCube() {}
	SurfelOTCube(R3 corner, float size) { this->corner = corner; this->size = size; }
	SurfelOTCube(vector<Surfel> & surfels) {
		vector<R3> p; for (int i = 0; i < surfels.size(); i++)
		{
			p.push_back(surfels[i].point);
			indices.push_back(i);
		}
		Pixel3DSet p3ds(p);
		R3 mn, mx;
		p3ds.min_max_R3(mn, mx);
		corner = mn;
		size = mx.max();

	}

	SurfelOTCube(const SurfelOTCube & c) {
		input(c);
	}
	SurfelOTCube & operator = (const SurfelOTCube & c) {
		input(c); return *this;
	}
	void input(const SurfelOTCube & c) {
		corner = c.corner;
		size = c.size;
		indices = c.indices;
		children = c.children;
	}
	bool isLeaf() { return children.size() == 0; }

	float avgCount() {
		stack<SurfelOTCube*> st;
		float c = 0.0f;
		float avg = 0.0f;
		st.push(this);
		while (!st.empty())
		{
			SurfelOTCube* that = st.top(); st.pop();
			if (that->isLeaf())
			{
				c += 1.0f;
				avg += (float)that->indices.size();
			}
			else
			{
				for (int i = 0; i < that->children.size(); i++)
				{
					st.push(&that->children[i]);
				}
			}
		}
		if (c == 0.0f) return -1.0f;
		return avg / c;
	}

	bool couldContain(R3 & p) {
		if (p.x < corner.x || p.x >= corner.x + size) return false;
		if (p.y < corner.y || p.y >= corner.y + size) return false;
		if (p.z < corner.z || p.z >= corner.z + size) return false;
		return true;
	}

	bool sphereCollision(R3 & center, float radius)
	{
		if (center.x + radius < corner.x) return false;
		if (center.x - radius > corner.x + size) return false;

		if (center.y + radius < corner.y) return false;
		if (center.y - radius > corner.y + size) return false;

		if (center.z + radius < corner.z) return false;
		if (center.z - radius > corner.z + size) return false;
		return true;
	}

	void split(vector<Surfel> * surfels, int depthLevels = 8) {
		if (depthLevels < 1) return;
		vector<SurfelOTCube> _children;
		float hs = size / 2.0f;

		_children.push_back(SurfelOTCube(corner, hs));
		_children.push_back(SurfelOTCube(corner + R3(0.0f, hs, 0.0f), hs));
		_children.push_back(SurfelOTCube(corner + R3(hs, hs, 0.0f), hs));
		_children.push_back(SurfelOTCube(corner + R3(hs, 0.0f, 0.0f), hs));

		for (int i = 0; i < 4; i++) {
			SurfelOTCube _ = _children[i];
			_.corner += R3(0.0f, 0.0f, hs);
			_children.push_back(_);
		}

		for (int i = 0; i < indices.size(); i++)
		{
			for (int j = 0; j < _children.size(); j++)
			{
				if (_children[j].couldContain(surfels->at(i).point)) {
					_children[j].indices.push_back(i);
					break;
				}
			}
		}

		int numChildrenAlive = 0; for (int i = 0; i < _children.size(); i++) numChildrenAlive += _children[i].indices.size() > 0 ? 1 : 0;
		if (numChildrenAlive > 1)
		{
			for (int i = 0; i < _children.size(); i++)
				if (_children[i].indices.size() > 0) children.push_back(_children[i]);
		}
		indices.clear();
		for (int i = 0; i < children.size(); i++) children[i].split(surfels, depthLevels - 1);

	}


};



class Surfels {
public:
	vector<Surfel> points;
	SurfelOTCube ot;
	Surfels(Pix3D & p) {
		for (int y = 1; y < 480 - 1; y++)
		{
			for (int x = 1; x < 640 - 1; x++)
			{
				/*
				4 3 2
				5 0 1
				6 7 8
				*/
				R3 n[9] = {
					p.points[index(x,y)],
					p.points[index(x + 1,y)],
					p.points[index(x + 1,y - 1)],
					p.points[index(x,y - 1)],
					p.points[index(x - 1,y - 1)],
					p.points[index(x - 1,y)],
					p.points[index(x - 1,y + 1)],
					p.points[index(x,y + 1)],
					p.points[index(x + 1,y + 1)],
				};
				bool ne[9] = {
					p.validDepth[index(x,y)],
					p.validDepth[index(x + 1,y)],
					p.validDepth[index(x + 1,y - 1)],
					p.validDepth[index(x,y - 1)],
					p.validDepth[index(x - 1,y - 1)],
					p.validDepth[index(x - 1,y)],
					p.validDepth[index(x - 1,y + 1)],
					p.validDepth[index(x,y + 1)],
					p.validDepth[index(x + 1,y + 1)],
				};
				vector<SI_FullTriangle> tris;
				//get quad points from all around
				if (!ne[0]) continue;
				if (ne[2]) {
					if (ne[1]) tris.push_back(SI_FullTriangle(n[0], n[1], n[2]));
					if (ne[3]) tris.push_back(SI_FullTriangle(n[0], n[2], n[3]));
				}
				if (ne[4]) {
					if (ne[3]) tris.push_back(SI_FullTriangle(n[0], n[3], n[4]));
					if (ne[5]) tris.push_back(SI_FullTriangle(n[0], n[4], n[5]));
				}

				if (ne[6]) {
					if (ne[5]) tris.push_back(SI_FullTriangle(n[0], n[5], n[6]));
					if (ne[7]) tris.push_back(SI_FullTriangle(n[0], n[6], n[7]));
				}

				if (ne[8]) {
					if (ne[7]) tris.push_back(SI_FullTriangle(n[0], n[7], n[8]));
					if (ne[1]) tris.push_back(SI_FullTriangle(n[0], n[8], n[1]));
				}
				if (tris.size() <= 0) continue;
				R3 normal;
				float trisizeI = 1.0f / (float)tris.size();
				for (int i = 0; i < tris.size(); i++) normal += (tris[i].normal() * trisizeI);
				normal.normalize();
				//compute normals and get average for the current point
				//set color, normal and point and add to list
				//if (tris.size() > 0) cout << "got some...\n";
				points.push_back(Surfel(n[0], normal, p.colors[index(x, y)]));
			}
		}
		ot = SurfelOTCube(points);
		ot.split(&points, 4);
	}

	vector<int> pointsWithinRadius(R3 & p, float radius)
	{
		vector<int> inds;
		stack<SurfelOTCube*> st;
		st.push(&ot);
		while (!st.empty())
		{
			SurfelOTCube * that = st.top(); st.pop();
			if (that->isLeaf())
			{
				inds.insert(inds.end(), that->indices.begin(), that->indices.end());
			}
			else
			{
				for (int i = 0; i < that->children.size(); i++)
				{
					bool collides = that->children[i].sphereCollision(p, radius);
					if (collides) st.push(&that->children[i]);
				}
			}
		}
		return inds;
	}

	static int index(int x, int y) {
		return y * 640 + x;
	}

	float errorF(R3 x, float h) {
		float sum = 0.0f;
		vector<int> subs = pointsWithinRadius(x, h);
		for (int j = 0; j < subs.size(); j++) {
			int i = subs[j];
			float _ = ((_k(subs, x, h)) * (x - points[i].point));
			sum += _b(subs, i, x, h) *  _*_;
		}
		return sum;
	}

	float _b(vector<int> & subs, int i, R3 x, float h) {
		float sum = 0.0f;
		float h2 = h*h;
		for (int _ = 0; _ < subs.size(); _++) {
			int j = subs[_];
			float d = x.dist(points[j].point);
			sum += exp(-d*d / h2);
		}
		float d = x.dist(points[i].point);
		return	exp(-d*d / h2) / sum;

	}

	R3 _k(vector<int>& subs, R3 x, float h) {
		R3 sum;
		for (int j = 0; j < subs.size(); j++) {
			int i = subs[j];
			sum += points[i].normal * _b(subs, i, x, h);
		}
		return sum;
	}

};

int main(int argc, char * * argv)
{
	CapturePixel3DSet video("Apartment.Texture.rotate", 3);
	Pix3D p; video.read(p);
	Surfels s = p;

	cout << s.points.size() << endl;
	cout << s.ot.avgCount() << endl;

	LTimer t; t.start();
	cout << s.errorF(s.points[154].point, 100.2f) << endl;
	t.stop();
	cout << t.getSeconds() << endl;

	return 0;
}
