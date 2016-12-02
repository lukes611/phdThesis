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
#include "code/phd/LSift.h"
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
using namespace LukeLincoln;
using namespace ll_pix3d;


#include "code/basics/BitReaderWriter.h"

void l3(int s, function<void(int,int,int)> f, int from = 0)
{
    for(int z = from; z < s-from; z++)
    {
        for(int y = from; y < s-from; y++)
        {
            for(int x = from; x < s-from; x++)
                f(x,y,z);
        }
    }
}

int main(int argc, char * * argv)
{

    R3 R(5.0f, 2.0f, 10.0f);
	float S = 1.0f;
	R3 T(10.0f, 1.0f, 9.0f);
    


   

	SIObj ob;
    ob.open_obj("c:/lcppdata/obj/bunny_simplified2.obj");
	cout << ob._points.size() << " points and " << ob._triangles.size() << " triangles" << endl;
	
	//to-do : do 3d reg




    VMat v(ob, 128, 30);
	Mat M = VMat::transformation_matrix(v.s, R.x, R.y, R.z, S, T.x, T.y, T.z);

    LTimer t; t.start();
    //LukeLincoln::testFeatures(v, R, S, T);
	LukeLincoln::testMatches(v, R, S, T, true, 80);

	VMat v2 = v; v2.transform_volume_forward(M);

	Mat MM = lukes_siftRegister(v, v2, true, 80);
	//vector<Point3f> p1, p2;
    //LukeLincoln::lukes_sift(v, v2, p1, p2, true, 50);




	VMat v3 = v;
	v3.transform_volume_forward(MM);
	//cout << M << endl;
	//cout << MM << endl;;




	v.pixel3dset().save_obj("C:/Users/s2807774/Desktop/v.obj");
	v2.pixel3dset().save_obj("C:/Users/s2807774/Desktop/v2.obj");
	v3.pixel3dset().save_obj("C:/Users/s2807774/Desktop/v3.obj");


    t.stop();
    cout << " time taken: " << t.getSeconds() << endl;


    //v.save_obj("/home/luke/Desktop/b.obj", 128, 0.2f);
    //cout << "starting " << endl;
   // t.reset(); t.start();

    //vector<SiftFeature3D> f;
    //LukeLincoln::findFeatures(f, 1, v);

    //t.stop(); cout << t.getSeconds() << endl;

    //cout << "num feats " << f.size() << endl;

    //vector<R3> ps;
    //for(int i = 0; i < f.size(); i++) ps.push_back(R3(f[i].x, f[i].y, f[i].z));

    //Pixel3DSet(ps).save_obj("C:/Users/s2807774/Desktop/a.obj");

	//v2.save_obj("/home/luke/Desktop/a.obj", 256, 0.2f);
	//v2.threshold(0.6f);
	//v2.pixel3dset(0.0f).save_obj("/home/luke/Desktop/a.obj");
	
	return 0;
}




