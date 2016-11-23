#include <iostream>
#include <string>
#include <vector>
#include <stack>
#include <functional>

#include "code/basics/locv3.h"
#include "code/basics/R3.h"
#include "code/basics/llCamera.h"
#include "code/basics/LTimer.h"
#include "code/basics/Pixel3DSet.h"
#include "code/basics/VMatF.h"
#include "code/phd/measurements.h"
#include "code/phd/Licp.h"
#include "code/script/LScript.h"
#include "code/phd/fmRansac.h"
//#include "code/pc/TheVolumePhaseCorrelator.h"
//#include "code/phd/Lpcr.h"
//#include "code/phd/experiments.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
//using namespace ll_experiments;
using namespace ll_siobj;




#include "code/basics/BitReaderWriter.h"

class L3DFeat{
public:
    R3 p, n; float s;
    L3DFeat(R3 p, R3 n, float s)
    {
        this->p=p;
        this->n=n;
        this->s=s;
    }
    L3DFeat(const L3DFeat & input)
    {
        copyIn(input);
    }
    L3DFeat & operator = (const L3DFeat & input)
    {
        copyIn(input);
        return *this;
    }
    void copyIn(const L3DFeat & input)
    {
        p = input.p;
        n = input.n;
        s = input.s;
    }
};

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
	R3 corner; //the bottom corner
	float cubeSize; //the size of the cube
	vector<int> I; //the indices
	vector<SurfelOTCube> children; //the children


	SurfelOTCube()
	{
        corner *= 0.0f;
        cubeSize = 512.0f;
    }
	SurfelOTCube(R3 c, float s)
	{
        corner = c;
        cubeSize = s;
    }

    float halfSize()
    {
        return cubeSize * 0.5f;
    }

	SurfelOTCube(vector<Surfel> & surfels) {
		vector<R3> ls; //create a list of points

		R3 mn = surfels[0].point;
		R3 mx = mn;

		for (int i = 0; i < surfels.size(); i++) //for each point
		{
            R3 point = surfels[i].point;
            mx.max(point);
            mn.min(point);
			ls.push_back(point);
			I.push_back(i);
		}
		//cout << "min/max R3:= " << mn << " , " << mx << endl;
		corner = mn - R3(1.0f, 1.0f, 1.0f);
		cubeSize = (mx-mn).max() + 1.0f;

		//cout << "corner/size in constructor := " << corner << ", " << cubeSize << endl;
		//cout << "percentage which fit in @ constructor := " << percentageFit(&surfels) << endl;

	}

	SurfelOTCube(const SurfelOTCube & c) {
		input(c);
	}
	SurfelOTCube & operator = (const SurfelOTCube & c) {
		input(c); return *this;
	}
	void input(const SurfelOTCube & c) {
		corner = c.corner;
		cubeSize = c.cubeSize;
		I = c.I;
		children = c.children;
	}
	bool isLeaf() { return children.size() == 0; }

	void forEachChild(function<void(SurfelOTCube*)> f)
	{
        stack<SurfelOTCube*> Stack;
		Stack.push(this);
		while (!Stack.empty())
		{
			SurfelOTCube* that = Stack.top(); Stack.pop();
			if (that->isLeaf()) //if leaf
				f(that);
			else
			{
				for (int i = 0; i < that->children.size(); i++) //for each child
				{
					Stack.push(&that->children[i]);
				}
			}
		}
	}

	float avgCount() {
        int c = 0, n = 0;
        forEachChild([&c, &n](SurfelOTCube * that) -> void {
            c += that->I.size();
            n++;
        });
        return c / (float) n;
	}

	int count() {
        int c = 0;
        forEachChild([&c](SurfelOTCube * that) -> void {
            c += that->I.size();
        });
        return c;
	}


	float percentageFit(vector<Surfel> * surfels)
	{
        int fitCount = 0;
        for(int i = 0; i < I.size(); i++)
        {
            int j = I[i];
            R3 p = surfels->at(j).point;
            if(couldContain(p)) fitCount++;
        }
        if(I.size() == 0) return 1.0f;
        return fitCount / (float) I.size();
	}

	bool couldContain(R3 & p) {
		if (p.x < corner.x || p.x > (corner.x + cubeSize)) return false;
		if (p.y < corner.y || p.y > corner.y + cubeSize) return false;
		if (p.z < corner.z || p.z > corner.z + cubeSize) return false;
		return true;
	}

	bool sphereCollision(R3 & center, float radius)
	{
		if (center.x + radius < corner.x) return false;
		if (center.x - radius > corner.x + cubeSize) return false;

		if (center.y + radius < corner.y) return false;
		if (center.y - radius > corner.y + cubeSize) return false;

		if (center.z + radius < corner.z) return false;
		if (center.z - radius > corner.z + cubeSize) return false;
		return true;
	}

	void split(vector<Surfel> * surfels, int depthLevels = 8) {
		if (depthLevels < 1) return;
		vector<SurfelOTCube> ChildList;
		float hs = halfSize();

		ChildList.push_back(SurfelOTCube(corner, hs));
		ChildList.push_back(SurfelOTCube(corner + R3(hs, 0.0f, 0.0f), hs));
		ChildList.push_back(SurfelOTCube(corner + R3(hs, hs, 0.0f), hs));
		ChildList.push_back(SurfelOTCube(corner + R3(0.0f, hs, 0.0f), hs));

		ChildList.push_back(SurfelOTCube(corner + R3(0.0f, 0.0f, hs), hs));
		ChildList.push_back(SurfelOTCube(corner + R3(hs, 0.0f, hs), hs));
		ChildList.push_back(SurfelOTCube(corner + R3(hs, hs, hs), hs));
		ChildList.push_back(SurfelOTCube(corner + R3(0.0f, hs, hs), hs));




		//cout << "parent. c/s " << "percentage which fits: " << percentageFit(surfels) << endl;
        //int fit = 0, dontFit = 0;
		for (int i = 0; i < I.size(); i++)
		{
            int j = I[i];
            //bool foundHome = false;
			for (int c = 0; c < ChildList.size(); c++)
			{
                R3 point = surfels->at(j).point;
				if (ChildList[c].couldContain(point)) {
					ChildList[c].I.push_back(j);
					//fit++;
					//foundHome = true;
					break;
				}
			}

			//if(!foundHome)
			//{
            //    dontFit++;
                //cout << "bad sabta" << endl;
                //cout << surfels->at(i).point << endl;
                //cout << corner << " + " << size << endl;
                //cout << "i can fit " << couldContain(surfels->at(i).point) << endl;
            //}
		}



        //cout << "ratio fit to no-fit : " << (fit / (double)(fit+dontFit)) << " of " << (fit+dontFit) << " " << I.size() << endl;
		int numChildrenAlive = 0; for (int i = 0; i < ChildList.size(); i++) numChildrenAlive += ChildList[i].I.size() > 0 ? 1 : 0;
		if (numChildrenAlive > 1)
		{
			for (int i = 0; i < ChildList.size(); i++)
				if (ChildList[i].I.size() > 0) children.push_back(ChildList[i]);
		}else return;
		I.clear();
		for (int i = 0; i < children.size(); i++) children[i].split(surfels, depthLevels - 1);

	}


};



class Surfels {
public:
	vector<Surfel> points;
	SurfelOTCube ot;



	Surfels()
	{

	}

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
		computeOT();
	}

	void computeOT()
	{
        ot = SurfelOTCube(points);
		ot.split(&points, 4);
	}

	Surfels(SIObj & ob)
	{
        vector<SI_Triangle> tris = ob._triangles;
        vector<vector<SI_Triangle*>> ls(ob._points.size());
        //cout << "ls.size() = " << ls.size() << endl;
        for(int i = 0; i < tris.size(); i++) //for each triangle
        {
            ls[tris[i].a].push_back(&tris[i]);
            ls[tris[i].b].push_back(&tris[i]);
            ls[tris[i].c].push_back(&tris[i]);
        }

        for(int _ = 0; _ < ls.size(); _++)
        {
            vector<SI_Triangle*> L = ls[_];
            if(L.size() <= 0) continue;
            R3 norm;
            float scalar = 1.0f / (float) L.size();
            for(int i = 0; i < L.size(); i++)
            {
                SI_FullTriangle ft(ob._points[L[i]->a],
                                    ob._points[L[i]->b],
                                    ob._points[L[i]->c]);
                norm += (ft.normal() * scalar);
            }
            norm.normalize();
            points.push_back(Surfel(ob._points[_], norm, Vec3b(255,255,255)));
        }

        //cout << "points.size() = " << points.size() << endl;
        computeOT();
	}

	vector<int> pointsWithinRadius(R3 & p, float radius)
	{
		vector<int> inds;
		stack<SurfelOTCube*> st;
		st.push(&ot);
		while (!st.empty())
		{
			SurfelOTCube * that = st.top(); st.pop();
			if (that->isLeaf()) //if it is a leaf...
			{
                for(int i = 0; i < that->I.size(); i++)
                {
                    int j = that->I[i]; // for each index, j
                    R3 B = points[j].point; //B is the point to check
                    if(B.dist(p) <= radius) //B's distance from p is within radius: add to inds
                        inds.push_back(j);
                }
			}
			else //if not leaf
			{
				for (int i = 0; i < that->children.size(); i++) //for each child
				{
					bool collides = that->children[i].sphereCollision(p, radius);
					if (collides) st.push(&that->children[i]); //push back each child
				}
			}
		}
		return inds;
	}

	vector<int> pointsWithinRadiusSlow(R3 & p, float radius)
	{
		vector<int> inds;
		for(int i = 0; i < points.size(); i++)
            if(points[i].point.dist(p) <= radius) inds.push_back(i);
        return inds;
	}


	static int index(int x, int y) {
		return y * 640 + x;
	}

	float errorF(R3 x, float h) {
		float sum = 0.0f;
		vector<int> subs = pointsWithinRadius(x, 2.0f);
		for (int j = 0; j < subs.size(); j++) {
			int i = subs[j];
			float _ = ((_k(subs, x, h)) * (x - points[i].point));
			sum += _b(subs, i, x, h) *  _*_;
		}
		return sum;
	}

	float errorF(vector<int> & subs, R3 x, float h) {
		float sum = 0.0f;
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
        return sum.unit();
	}

	R3 blurNormal(vector<int> & subs, R3 x, float h)
	{
        return _k(subs, x, h);
	}


	static Mat R3ToColMat(R3 & input)
	{
        Mat m = Mat::zeros(Size(1, 3), CV_32FC1);
        m.at<float>(0,0) = input.x, m.at<float>(1,0) = input.y, m.at<float>(2,0) = input.z;
        return m.clone();
	}


	static Mat R3ToRowMat(R3 & input)
	{
        Mat m = Mat::zeros(Size(3, 1), CV_32FC1);
        m.at<float>(0,0) = input.x, m.at<float>(0,1) = input.y, m.at<float>(0,2) = input.z;
        return m.clone();
	}

    static float getFromMat(Mat & input, int r, int c)
    {
        return input.at<float>(r,c);
    }

    static R3 mat2R3(Mat & input)
    {
        if(input.size().width > input.size().height)
            return R3(getFromMat(input, 0, 0), getFromMat(input, 0, 1), getFromMat(input, 0, 2));
        return R3(getFromMat(input, 0, 0), getFromMat(input, 1, 0), getFromMat(input, 2, 0));
    }

    static Mat float2Mat(float x)
    {
        Mat m = Mat::zeros(Size(1, 1), CV_32FC1);
        m.at<float>(0,0) = x;
        return m.clone();
    }

    R3 blurPoint(R3 initialPoint, float scale, int maxIterations = -1)
    {
        maxIterations = maxIterations == -1 ? 200 : maxIterations;
        R3 prev = initialPoint;
        R3 bestPoint = prev;
        float bestScore = errorF(bestPoint, scale);

        //cout << "first error " << bestScore << endl;
        float prevError = bestScore;
        for(int i = 0; i < maxIterations; i++)
        {

            //cout << t1 << " -> " << t2 << " -> " << t3 << " = " << b0 << endl;
            R3 b1 = prev;
            float D = 0.2f;

            R3 JacobianMatrixR3(
                (errorF(b1 + R3(D, 0.0f, 0.0f),scale)-prevError) / D,
                (errorF(b1 + R3(0.0f, D, 0.0f),scale)-prevError) / D,
                (errorF(b1 + R3(0.0f, 0.0f, D),scale)-prevError) / D
            );
            Mat jacobian = R3ToRowMat(JacobianMatrixR3);
            Mat jacobianT = jacobian.t();


            Mat J = (jacobianT * jacobian);
            J = J.inv();
            J = J * jacobianT;

            Mat b1_m = R3ToColMat(b1);
            Mat current = float2Mat(prevError);

            Mat change = J * current;

            b1_m = b1_m - change;
            //cout << change << endl;
            b1 =  mat2R3(b1_m);
            float newError = errorF(b1, scale);
            if(bestScore > newError)
            {
                bestScore = newError;
                bestPoint = b1;
               //cout << "new error " << newError << endl;
               //cout << "current point " << b1 << endl;
            }else if(i > 20) break;

            prev = b1;
            prevError = newError;

        }
        return bestPoint;
    }


    R3 blurPoint2(vector<int> & subs, R3 initialPoint, float scale, int maxIterations = -1)
    {
        maxIterations = maxIterations == -1 ? 200 : maxIterations;
        R3 prev = initialPoint;
        R3 bestPoint = prev;
        float bestScore = errorF(subs, bestPoint, scale);

        //cout << "first error " << bestScore << endl;
        float prevError = bestScore;
        for(int i = 0; i < maxIterations; i++)
        {

            //cout << t1 << " -> " << t2 << " -> " << t3 << " = " << b0 << endl;
            R3 b1 = prev;
            float D = 0.2f;

            R3 JacobianMatrixR3(
                (errorF(subs, b1 + R3(D, 0.0f, 0.0f),scale)-prevError) / D,
                (errorF(subs, b1 + R3(0.0f, D, 0.0f),scale)-prevError) / D,
                (errorF(subs, b1 + R3(0.0f, 0.0f, D),scale)-prevError) / D
            );
            Mat jacobian = R3ToRowMat(JacobianMatrixR3);
            Mat jacobianT = jacobian.t();


            Mat J = (jacobianT * jacobian);
            J = J.inv();
            J = J * jacobianT;

            Mat b1_m = R3ToColMat(b1);
            Mat current = float2Mat(prevError);

            Mat change = J * current;

            b1_m = b1_m - change;
            //cout << change << endl;
            b1 =  mat2R3(b1_m);
            float newError = errorF(subs, b1, scale);
            if(bestScore > newError)
            {
                bestScore = newError;
                bestPoint = b1;
               //cout << "new error " << newError << endl;
               //cout << "current point " << b1 << endl;
            }else if(i > 20) break;

            prev = b1;
            prevError = newError;

        }
        return bestPoint;
    }


    float getH(int level)
    {
        float H0 = 1.0f;
        float F = ot.cubeSize * 0.02;
        return H0 * pow(F, (float) level);
    }

    Surfel project(int index, int level = 1)
    {
        R3 p = points[index].point;

        vector<int> subs = pointsWithinRadius(p, ot.cubeSize * 0.001f);
        //cout << "subs size: " << subs.size() << endl;
        float H = getH(level);
        R3 rv = blurPoint2(subs, p, H);
        //cout << " p1 ein" << endl;
        //p = rv;
        return Surfel(rv, blurNormal(subs, p, H), Vec3b(255, 255, 255));
    }

    Surfels compute(int level)
    {
        Surfels ret;
        for(int i = 0; i < points.size(); i++)
        {
            ret.points.push_back(project(i, level));

            //if(i % 100 == 0)
            {
                //cout << (i / (float)points.size()) << "%%" << endl;
            }
        }
        cout << endl;
        ret.computeOT();
        return ret;
    }

    SIObj toObj()
    {
        SIObj rt(points.size(), 0);
        for(int i = 0; i < points.size(); i++) rt._points[i] = points[i].point;
        return rt;
    }

    float featureMeasure(int index, int myLevel)
    {
        Surfel me = project(index, myLevel);
        Surfel sub = project(index, myLevel - 1);
        return me.normal * (me.point - sub.point);
    }

    bool isFeature(int index, int level)
    {
        Surfel p = project(index, level);
        vector<int> inds = pointsWithinRadius(p.point, 0.5f * getH(level));
        //cout << inds.size() << endl;
        int numGt = 0;
        int numLt = 0;
        float m1 = featureMeasure(index, level);
        for(int i = 0; i < inds.size(); i++)
        {
            int j = inds[i];
            if(featureMeasure(j, level) < m1) numLt++;
            if(featureMeasure(j, level-1) < m1) numLt++;
            if(featureMeasure(j, level+1) < m1) numLt++;

            if(featureMeasure(j, level) > m1) numGt++;
            if(featureMeasure(j, level-1) > m1) numGt++;
            if(featureMeasure(j, level+1) > m1) numGt++;
            if(numGt >= 1 && numLt >= 1) return false;
        }
        return true;
    }

    vector<L3DFeat> genFeatures()
    {
        vector<L3DFeat> ret;

        for(int sc = 1; sc <= 2; sc++)
        {
                for(int i = 0; i < points.size(); i++)
                {
                    if(isFeature(i, sc))
                    {
                        Surfel s = project(i, sc);
                        ret.push_back(L3DFeat(s.point, s.normal, getH(sc)));
                    }
                }
        }
        return ret;
    }
};

int main(int argc, char * * argv)
{
    SIObj p; p.open_obj("/home/luke/gitProjects/loqur/monkey1.obj");
    //cout << ob.stats() << endl;
	//CapturePixel3DSet video = CapturePixel3DSet::openCustom("/home/luke/lcppdata/pix3dc/films", "Apartment.Texture.rotate", 3);
	//Pix3D p; video.read(p);
	Surfels s = p;

    //Surfels s2 = s.compute(1);

    vector<L3DFeat> fts = s.genFeatures();
    vector<R3> ps;
    for(int i = 0; i < fts.size(); i++)
    {
        ps.push_back(fts[i].p);
    }

    Pixel3DSet p3(ps);


    p3.siobj().saveOBJ("/home/luke/Desktop/feats.obj");




	return 0;
}




