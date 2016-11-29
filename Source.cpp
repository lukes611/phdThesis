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

namespace ll_Sift
{


	typedef struct __llSiftMatch{
        SiftFeature2D a, b;
        double distance;

	} llSiftMatch;















    void computeMatches(vector<SiftFeature2D> & fvs1, vector<SiftFeature2D> & fvs2, vector<Point2i> & p1, vector<Point2i> & p2)
    {
        vector<llSiftMatch> matches;
        //a match has
        for(int i = 0; i < fvs1.size(); i++)
        {
            double best = fvs2[0].difference(fvs1[i]);
            int bj = 0;
            for(int j = 1; j < fvs2.size(); j++)
            {
                double er = fvs2[j].difference(fvs1[i]);
                if(er < best)
                {
                    best = er;
                    bj = j;
                }
            }
            llSiftMatch m; m.a = fvs1[i]; m.b = fvs2[bj];
            //imshow("f1", m.a.im);
            //imshow("f2", m.b.im); waitKey();
            matches.push_back(m);

        }

        auto f = [](const llSiftMatch & a, const llSiftMatch & b) -> bool { return a.distance < b.distance; };
		std::sort(matches.begin(), matches.end(), f);

        int am = 200;
        int mx = am < matches.size() ? am : matches.size();
        for(int i = 0; i < mx; i++)
        {
           // p1.push_back(Point2i(matches[i].a.x, matches[i].a.y));
           // p2.push_back(Point2i(matches[i].b.x, matches[i].b.y));
           p1.push_back(matches[i].a.truePoint());
           p2.push_back(matches[i].b.truePoint());
        }


    }

};

cv::Point2f operator*(cv::Mat M, const cv::Point2f& p)
{
    Mat src = Mat::zeros(Size(1, 3), CV_64FC1);

    src.at<double>(0,0)=p.x;
    src.at<double>(1,0)=p.y;
    src.at<double>(2,0)=1.0;

    Mat dst = M*src; //USE MATRIX ALGEBRA
    return cv::Point2f(dst.at<double>(0,0),dst.at<double>(1,0));
}

int main(int argc, char * * argv)
{

     auto viewF = [](vector<LukeLincoln::SiftFeature2D>&f, Mat im, bool w = false) -> void {
        auto x = im.clone();
        for(int i = 0; i < f.size(); i++)
        {
            Point2i p1 = f[i].truePoint();
            float X, Y;
            R3::GetUnitPointFromAngle(f[i].angle, X, Y);
            double rad = 2.0 * f[i].trueRad();
            X*= rad, Y*= rad;
            Point2i p2(p1.x + (int)X, p1.y + (int)Y);
            cv::line(x, p1, p2, Scalar(255));
            cv::circle(x, p1, rad, Scalar(255));
        }
        stringstream name; name << "i " << w;
        imshow(name.str(), x); if(w)waitKey();
    };

    /*Mat lenna = imread("/home/luke/lcppdata/ims/Lenna.png", CV_LOAD_IMAGE_GRAYSCALE);
    ll_UCF1_to_32F1(lenna);
    ll_normalize(lenna);
    blur(lenna, lenna, Size(3,3));
    Mat g = LukeLincoln::getGaussianDifferenceImage(Size(7,7), 2.0f, pow(2.0f, 0.5f));
    Mat gl;
    filter2D(lenna, gl, lenna.depth(), g);
    Mat out = Mat::zeros(lenna.size(), CV_32FC1);

    Mat cp = lenna.clone();


    ll_normalize(cp);

    for(int y = 10; y < lenna.size().height-10; y++)
    {
        for(int x = 10; x < lenna.size().width-10; x++)
        {
            if(LukeLincoln::isCorner2(x, y, lenna))
            {

                cp.at<float>(y,x) = 1.0f;
                //circle(cp, Point(x,y), 5, Scalar(1.0));

            }
            float dxx, dyy, dxy;
                partialDerivatives3(gl, x, y, dxx, dyy, dxy);
                //dxy = sqrt(dxx*dxx + dyy*dyy);

                //compute trace and determinant
                float tr = dxx + dyy;
                float det = dxx*dyy - dxy*dxy;
                float TTT = (11.0f*11.0f) / 10.0f;
                float V = (tr*tr) / det;
                out.at<float>(y,x) = V >= TTT ? 1.0f : 0.0f;
        }
    }
    ll_normalize(out);
    imshow("b", out);
    imshow("a", cp);
    waitKey();
*/



    double R = 2.0, S = 1.0;
    Point2d T(2.0, 1.0);

    Mat A, B, a, b;

    A = imread("/home/luke/lcppdata/ims/e.jpg");

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
    viewF(f2, b, false);

    //return 1;


    vector<Point2i> p1, p2;
    ll_Sift::computeMatches(f1, f2, p1, p2);

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
        if(dist < 3.0f)
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




