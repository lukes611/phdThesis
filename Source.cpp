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

namespace ll_Sift
{

	typedef struct _llSiftFeat {
		int x, y, z;
		double scale;
		double octave;
		R3 normal;
		float angle1, angle2;
		vector<double> featureVector;
		Mat im;

        double measure(struct _llSiftFeat & o)
        {
            return cv::sum(abs(im - o.im))[0];
            double sum = 0.0;
            for(int i = 1; i < featureVector.size(); i++)
            {
                double er = featureVector[i] - o.featureVector[i];
                sum += er*er;
            }
            return sum;
        }

	} llSiftFeat;

	typedef struct __llSiftMatch{
        llSiftFeat a, b;
        double distance;
	} llSiftMatch;

	double gaussian(double x, double y, double R)
	{
		return (1.0 / (2.0 * 3.14159265359 * R*R)) * exp(-(x*x + y*y) / (2.0 * R*R));
	}

	double gaussian(double d, double R)
	{
		return (1.0 / (2.0 * 3.14159265359 * R*R)) * exp(-(d) / (2.0 * R*R));
	}

	double gaussianDifference(double d, double R, double K)
	{
		return gaussian(d, K*R) - gaussian(d, R);
	}

	Mat getGaussianImage(Size s, double R)
	{
		Mat ret = Mat::zeros(s, CV_32FC1);
		for (int y = 0; y < s.height; y++)
		{
			for (int x = 0; x < s.width; x++)
			{
				ret.at<float>(y, x) = (float)gaussian(ll_distance(x, y, s.width/2, s.height/2), R);
			}
		}
		return ret.clone();
	}

	Mat getGaussianDifferenceImage(Size s, double R, double K)
	{
		Mat ret = Mat::zeros(s, CV_32FC1);
		for (int y = 0; y < s.height; y++)
		{
			for (int x = 0; x < s.width; x++)
			{
				ret.at<float>(y, x) = (float)gaussianDifference(ll_distance(x, y, s.width / 2, s.height / 2), R, K);
			}
		}
		return ret.clone();
	}

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


    void computeFeatureVectors(llSiftFeat & ret, Mat & im, Size s)
    {
        double hs = s.width * 0.5;
        int rois = (int)(sqrt(2.0*hs*hs)*2.0 + 5.5);
        //form regular image based on
        Mat regularImg = im(Rect(ret.x, ret.y, rois, rois)).clone();
        int tl = (rois - s.width) / 2;
        Mat Img;
        ll_transform_image(regularImg, Img, -ret.angle1, 1.0, 0.0, 0.0);

        Img = Img(Rect(tl, tl, s.width, s.height));
        //cout << rois << " from " << s.width << endl;
        //ll_normalize(Img);

        //re-align it and get rid of pixels outside the circle
        //
        //get the log-polar of this
        Mat lp = Img;
        //ll_log_polar(Img, lp, 1.5);

        ret.im = lp.clone();
        //ll_normalize(lp);
        //get the 1dfft of combined rows
        Mat signal = Mat::zeros(Size(s.width * s.height, 1), CV_32FC1);
        float * ptr = (float*)lp.data; for(int i = 0; i < signal.size().width; i++) signal.at<float>(0, i) = ptr[i];
        Mat mag, pha;
        ll_fft::fft_magnitude_phase(signal, mag, pha, false);
        //mag = signal.clone();
        //return some of 1dfft's mag
        ll_normalize(mag);
        for(int i = 0; i < mag.size().width; i++) ret.featureVector.push_back((double)mag.at<float>(0, i));
    }



	vector<llSiftFeat> findFeatures(Mat & input)
	{
		vector<llSiftFeat> ret;
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

						llSiftFeat feature; feature.x = x; feature.y = y;
						feature.octave = 1.0;
						feature.scale = R/K;
						feature.angle1 = orientation(x, y, currentG);
						R3::GetUnitPointFromAngle(feature.angle1, feature.normal.x, feature.normal.y);

						//compute feature vector
						//by:
                        computeFeatureVectors(feature, currentG, Size(30,30));

						ret.push_back(feature);
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

    void computeMatches(vector<llSiftFeat> & fvs1, vector<llSiftFeat> & fvs2, vector<Point2i> & p1, vector<Point2i> & p2)
    {
        vector<llSiftMatch> matches;
        //a match has
        for(int i = 0; i < fvs1.size(); i++)
        {
            double best = fvs2[0].measure(fvs1[i]);
            int bj = 0;
            for(int j = 1; j < fvs2.size(); j++)
            {
                double er = fvs2[j].measure(fvs1[i]);
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

        for(int i = 0; i < 35; i++)
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
    ll_transform_image(lenna, lenna2, 10.0f, 1.0, 0.0f, 0.0);

    //cout << ll_Sift::orientation(lenna.size().width/2, lenna.size().height/2, lenna) << endl;
    //cout << ll_Sift::orientation(lenna.size().width/2, lenna.size().height/2, lenna2) << endl;
    //return 0;


    Mat lennaOrig2;
    ll_transform_image(lennaOrig, lennaOrig2, 10.0f, 1.0, 0.0f, 0.0);

    vector<ll_Sift::llSiftFeat> f1 = ll_Sift::findFeatures(lenna);

    vector<ll_Sift::llSiftFeat> f2 = ll_Sift::findFeatures(lenna2);

    vector<Point2i> p1, p2;
    ll_Sift::computeMatches(f1, f2, p1, p2);

    Mat ma = ll_view_matches(lennaOrig, lennaOrig2, p1, p2);
    imshow("ma", ma);
    waitKey();


	return 0;
}




