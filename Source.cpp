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




	bool isTrueFeature2(int x, int y, Mat & m)
	{
        float v = m.at<float>(y,x);
        for(int j = -1; j <= 1; j++)
            for(int i = -1; i <= 1; i++)
                if(m.at<float>(y+j,x+i) != v) return true;
        return false;
	}

	bool isTrueFeature(int x, int y, Mat & m)
	{
		float dxx = m.at<float>(y, x + 1) - m.at<float>(y, x - 1);
		float dyy = m.at<float>(y + 1, x) - m.at<float>(y - 1, x);
		float dxy = m.at<float>(y + 1, x + 1) - m.at<float>(y - 1, x - 1);

		if(dxx*dxx < 0.0001f) return false;
		if(dyy*dyy < 0.0001f) return false;

		dxx /= 2.f;
		dyy /= 2.f;
		dxy /= sqrt(8.0f);

		float tr = dxx + dyy;
		float det = dxx*dyy - dxy*dxy;

		float measure = (tr*tr) / det;
		float r = 10.0f;
		float r1 = r + 1.0f;
		//cout << measure << endl;

		return measure <= (r1*r1) / r;
	}

	float orientation(int x, int y, Mat & m)
	{
        R3::getAngle((m.at<float>(y + 1, x) - m.at<float>(y - 1, x)),
        (m.at<float>(y, x + 1) - m.at<float>(y, x - 1)));

	}

	float magnitude(int x, int y, Mat & m)
	{
        R3((m.at<float>(y + 1, x) - m.at<float>(y - 1, x)),
        (m.at<float>(y, x + 1) - m.at<float>(y, x - 1)), 0.0f).mag();

	}

    vector<float> computeOrientations(SiftFeature2D & feature, Mat & image)
    {
        vector<float> ret;
        float roiSizef = 1.5f * feature.scale;
        float roiSizefb = roiSizef + 4.0f;
        Mat g = getGaussianImage(Size(roiSizefb, roiSizefb), 1.5f * feature.scale);
        Mat roi = image(Rect(feature.x - roiSizefb*0.5f, feature.y - roiSizefb*0.5f, (int)roiSizefb, (int)roiSizefb));
        //filter2D(roi, roi, roi.depth(), g);
        Mat angles = Mat::zeros(Size(36, 1), CV_32FC1);
        for(int y = 1; y < roi.size().height-1; y++)
        {
            for(int x = 1; x < roi.size().width-1; x++)
            {
                float O = orientation(x, y, roi);
                int bin = (int)(O / 10.0f);
                angles.at<float>(0, bin) += magnitude(x,y, roi) * g.at<float>(x,y);
            }
        }
        ll_normalize(angles);

        float best = angles.at<float>(0,0);
        int bestInd = 0;

        for(int i = 0; i < 36; i++)
        {
            float v = angles.at<float>(0, i);
            if(v >= 0.8) ret.push_back(10.0f * (float)i);
            if(best < v)
            {
                best = v;
                bestInd = ret.size() - 1;
            }
        }
        if(ret.size() > 1 && false)
        {
            float tmp = ret[0];
            ret[0] = ret[bestInd];
            ret[bestInd] = tmp;
        }

        return ret;
    }

    float computeBestOrientation(SiftFeature2D & feature, Mat & image)
    {
        vector<float> angles = computeOrientations(feature, image);
        if(angles.size() == 0) return 0.0f;
        return angles[0];
    }

    void computeFeatureVectors(SiftFeature2D & ret, Mat & im)
    {
        ret.featureVector = Mat::zeros(Size(8, 16), CV_32FC1);
        Mat roi = im(Rect(ret.x - 7, ret.y - 7, 16, 16));
        Mat g = getGaussianImage(Size(16, 16), 16.0);

        for(int y = 0; y < 16; y++)
        {
            for(int x = 0; x < 16; x++)
            {
                float m = magnitude(x, y, roi) * g.at<float>(y,x);
                float o = orientation(x, y, roi) - ret.angle;
                if(o < 0.0f) o = o + 360.0f;
                int xI = x / 4;
                int yI = y / 4;
                int GridSection = yI * 4 + xI;
                int bin = (int)(o / 45.0f);
                ret.featureVector.at<float>(GridSection, bin) += m;
            }
        }
        ll_normalize(ret.featureVector);
    }



	vector<SiftFeature2D> findFeatures(Mat & input)
	{
		vector<SiftFeature2D> ret;
		double K = sqrt(2.0);
		Size kernelSize(7,7);
		int numLevels = 5;

		double R = K * 0.5;
		Mat g = getGaussianImage(kernelSize, R);
		Mat d = getGaussianDifferenceImage(kernelSize, R, K);
		Mat prevG; filter2D(input, prevG, input.depth(), g);
		Mat prevD; filter2D(input, prevD, input.depth(), d);


		R = R * K;
		g = getGaussianImage(kernelSize, R);
		d = getGaussianDifferenceImage(kernelSize, R, K);
		Mat currentG; filter2D(input, currentG, input.depth(), g);
		Mat currentD; filter2D(input, currentD, input.depth(), d);

		for (int i = 2; i <= numLevels; i++)
		{
			R = R * K;
			g = getGaussianImage(kernelSize, R);
			d = getGaussianDifferenceImage(kernelSize, R, K);
			Mat nextG; filter2D(input, nextG, input.depth(), g);
			Mat nextD; filter2D(input, nextD, input.depth(), d);


			//for each pixel, if feature vector: record
			for (int y = 50; y < input.size().height - 50; y++)
			{
				for (int x = 50; x < input.size().width - 50; x++)
				{
					float dv = currentD.at<float>(y, x);
					int lt = 0, gt = 0;
					for (int j = -1; j <= 1; j++)
					{
						for (int i = -1; i <= 1; i++)
						{
							if (i != 0 && j != 0)
							{
								if (dv > currentD.at<float>(y + j, x + i)) lt++;
								if (dv < currentD.at<float>(y + j, x + i)) gt++;
							}
							if (dv > prevD.at<float>(y + j, x + i)) lt++;
							if (dv < prevD.at<float>(y + j, x + i)) gt++;

							if (dv > nextD.at<float>(y + j, x + i)) lt++;
							if (dv < nextD.at<float>(y + j, x + i)) gt++;
						}
					}

					if (lt == 0 || gt == 0)
					{
						if (!isTrueFeature(x, y, currentG)) continue;
						//if (!isTrueFeature2(x,y, currentG)) continue;

						SiftFeature2D feature; feature.x = x; feature.y = y;
						feature.octave = 1.0;
						feature.scale = R/K;
						vector<float> bestAngles = computeOrientations(feature, currentG);//orientation(x, y, currentG);
						for(int _ = 0; _ < bestAngles.size(); _++)
						{
                            SiftFeature2D f = feature;
                            f.angle = bestAngles[_];
                            computeFeatureVectors(f, currentG);
                            ret.push_back(f);
                        }
					}
				}
			}

			//setup for next iteration
			prevG = currentG;
			currentG = nextG;

			prevD = currentD;
			currentD = nextD;
		}
		return ret;
	}

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

        for(int i = 0; i < 15; i++)
        {
            p1.push_back(Point2i(matches[i].a.x, matches[i].a.y));
            p2.push_back(Point2i(matches[i].b.x, matches[i].b.y));
        }


    }

};

int main(int argc, char * * argv)
{



    Mat lennaOrig = imread("/home/luke/lcppdata/ims/Lenna.png");
    Mat lenna = imread("/home/luke/lcppdata/ims/Lenna.png", CV_LOAD_IMAGE_GRAYSCALE);
    ll_UCF1_to_32F1(lenna);
    //ll_transform_image(lenna, lenna, 0.0, 1.0, 5.0, 5.0);

    Mat lenna2;
    ll_transform_image(lenna, lenna2, 10.0f, 1.0, 5.0f, -10.0);

    //cout << ll_Sift::orientation(lenna.size().width/2, lenna.size().height/2, lenna) << endl;
    //cout << ll_Sift::orientation(lenna.size().width/2, lenna.size().height/2, lenna2) << endl;
    //return 0;


    Mat lennaOrig2;
    ll_transform_image(lennaOrig, lennaOrig2, 10.0f, 1.0, 5.0f, -10.0);

    auto viewF = [](vector<LukeLincoln::SiftFeature2D>&f, Mat im, bool w = false) -> void {
        auto x = im.clone();
        for(int i = 0; i < f.size(); i++)
        {
            Point2i p1(f[i].x, f[i].y);
            float X, Y;
            R3::GetUnitPointFromAngle(f[i].angle, X, Y);
            double rad = 5.0 * f[i].scale;
            X*= rad, Y*= rad;
            Point2i p2(p1.x + (int)X, p1.y + (int)Y);
            cv::line(x, p1, p2, Scalar(255));
            cv::circle(x, p1, rad, Scalar(255));
        }
        stringstream name; name << "i " << w;
        imshow(name.str(), x); if(w)waitKey();
    };

    vector<LukeLincoln::SiftFeature2D> f1 = ll_Sift::findFeatures(lenna);

    vector<SiftFeature2D> f2 = ll_Sift::findFeatures(lenna2);

    viewF(f1, lenna, false);
    viewF(f2, lenna2, true);

    return 1;

    vector<Point2i> p1, p2;
    ll_Sift::computeMatches(f1, f2, p1, p2);

    Mat ma = ll_view_matches(lennaOrig, lennaOrig2, p1, p2);
    imshow("ma", ma);
    waitKey();


	return 0;
}




