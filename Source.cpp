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



#include "code/basics/BitReaderWriter.h"


int main(int argc, char * * argv)
{

     auto viewF = [](vector<LukeLincoln::SiftFeature2D>&f, Mat im, bool w = false) -> void {
		 Mat _ = featureVisualization(f, im);
		 stringstream c; c << "im_ " << f[0].angle;
		 imshow(c.str(), _);
		 if (w) waitKey();
    };

   


    double R = 25.0, S = 1.0;
    Point2d T(0.0, 0.0);

    Mat A, B, a, b;

    A = imread("c:/lcppdata/ims/lena.png");

    //ll_transform_image(A, A, 0.0, 1.0, 5.0, 5.0);

    cvtColor(A, a, cv::COLOR_BGR2GRAY); ll_UCF1_to_32F1(a);

    ll_transform_image(a, b, R, S, T.x, T.y);
    ll_transform_image(A, B, R, S, T.x, T.y);

    //cout << computeOrientation(a.size().width/2, a.size().height/2, a) << endl;
    //cout << LukeLincoln::computeOrientation(a.size().width/2, a.size().height/2, b) << endl;
    //return 0;




    vector<LukeLincoln::SiftFeature2D> f1 = findMultiScaleFeatures(a, 3);
    vector<SiftFeature2D> f2 = findMultiScaleFeatures(b, 3);

    viewF(f1, a, false);
    viewF(f2, b, true);

    //return 1;


    vector<Point2i> p1, p2;
    computeMatches(f1, f2, p1, p2, true, 300);

    Mat m = ll_transformation_matrix(A.size(), R, S, T.x, T.y);
    int Count = 0, Total = 0;

    vector<Point2i> gp1, gp2;

    for(int i = 0; i < p1.size(); i++)
    {
        Point2f pa(p1[i].x, p1[i].y);
        Point2f pb(p2[i].x, p2[i].y);
        pa = m * pa;
        pa -= pb;
        float dist = sqrt(pa.dot(pa));
        //cout << dist << endl;
        if(dist < 1.5f)
        {
             Count++;
             gp1.push_back(p1[i]);
             gp2.push_back(p2[i]);
        }

        Total++;
    }
    cout << Count << " / " << Total << endl;
    cout << "% accurate: " << (Count / (double)Total) << endl;


    Mat ma = ll_view_matches(A, B, gp1, gp2);
    imshow("ma", ma);
    waitKey();


	return 0;
}




