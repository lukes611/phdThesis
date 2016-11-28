#include "LSift.h"
#include "../basics/R3.h"

using namespace cv;
using namespace ll_R3;
using namespace std;

namespace LukeLincoln
{
    double SiftFeature2D::difference(const SiftFeature2D & o)
    {
        return cv::sum(abs(featureVector - o.featureVector))[0];
    }

    Point2f SiftFeature2D::truePoint()
    {
        return Point2f(x * pow(2.0f, (double)octave), y * pow(2.0f, (double)octave));
    }

    float SiftFeature2D::trueRad()
    {
        return pow(2.0, octave) + (float)scale;
    }

    double gaussian(double x, double y, double sigma)
	{
        double sigmaSquared = sigma * sigma;
		return (1.0 / (2.0 * 3.14159265359 * sigmaSquared)) * exp(-(x*x + y*y) / (2.0 * sigmaSquared));
	}

	double gaussian(double d, double sigma)
	{
        double sigmaSquared = sigma * sigma;
		return (1.0 / (2.0 * 3.14159265359 * sigmaSquared)) * exp(-(d) / (2.0 * sigmaSquared));
	}

    double gaussianDifference(double distanceFromOrigin, double sigma, double K)
	{
		return gaussian(distanceFromOrigin, K*sigma) - gaussian(distanceFromOrigin, sigma);
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

    void partialDerivatives1(Mat & im, int x, int y, float & dx, float & dy)
    {
        dx = im.at<float>(y, x + 1) - im.at<float>(y, x - 1);
        dy = im.at<float>(y + 1, x) - im.at<float>(y - 1, x);
    }

    void partialDerivatives2(Mat & im, int x, int y, float & dx, float & dy)
    {
        dx = (im.at<float>(y-1, x + 1) - im.at<float>(y-1, x - 1))
            +
            2.0f*(im.at<float>(y, x + 1) - 2.0f*im.at<float>(y, x - 1))
            +
            (im.at<float>(y+1, x + 1) - im.at<float>(y+1, x - 1))
        ;
        dy = (im.at<float>(y + 1, x-1) - im.at<float>(y - 1, x-1))
            +
            2.0f*(im.at<float>(y + 1, x) - 2.0f*im.at<float>(y - 1, x))
            +
            (im.at<float>(y + 1, x+1) - im.at<float>(y - 1, x+1));
        //dx /= 4.0f, dy /= 4.0f;
    }

    void partialDerivatives3(Mat & im, int x, int y, float & dx, float & dy, float & dxy)
    {
        dx =    im.at<float>(y,x-1) - 2.0f*im.at<float>(y,x) + im.at<float>(y,x+1);
        dy =    im.at<float>(y-1,x) - 2.0f*im.at<float>(y,x) + im.at<float>(y+1,x);
        dxy =   (im.at<float>(y+1, x+1) - im.at<float>(y+1, x-1) - im.at<float>(y-1, x+1) + im.at<float>(y-1, x-1) ) * 0.25f;
    }


    float computeOrientation(int x, int y, Mat & m)
	{
        float dx, dy;
        dy = m.at<float>(y + 1, x) - m.at<float>(y - 1, x);
        dx = m.at<float>(y, x + 1) - m.at<float>(y, x - 1);
        //partialDerivatives2(m, x, y, dx, dy);

        return R3::getAngle(dx, dy);
    }

    float computeMagnitude(int x, int y, Mat & m)
	{
        float dx, dy;
        dy = m.at<float>(y + 1, x) - m.at<float>(y - 1, x);
        dx = m.at<float>(y, x + 1) - m.at<float>(y, x - 1);

        return sqrt(dx * dx + dy * dy);
	}


    bool isCorner(int x, int y, Mat & m)
	{
		float dxx, dyy;
        partialDerivatives2(m, x, y, dxx, dyy);
        float dxy = dxx * dyy;

        //compute trace and determinant
		float tr = dxx + dyy;
		float det = dxx*dyy - dxy*dxy;

        //if determinant is 0, return true
        if(det == 0.0f) return true;

        //compute the measure
		float measure = (tr*tr) / det;


		/*float r = 10.0f;
		float r1 = r + 1.0f;
		float test = (r1*r1) / r;*/
		float test = 12.0f;
        return measure < 12;
	}

	bool isCorner2(int x, int y, Mat & m)
	{
		float dxx, dyy, dxy;
        partialDerivatives3(m, x, y, dxx, dyy, dxy);

        //compute trace and determinant
		float tr = dxx + dyy;
		float det = dxx*dyy - dxy*dxy;



        //if determinant is 0, return true
        if(det == 0.0f) return true;

        //compute the measure
		float measure = (tr*tr) / det;

        //std::cout << measure << std::endl;

		float r = 10.0f;
		float r1 = r + 1.0f;
		float test = (r1*r1) / r;
		//float test = 12.0f;
        return measure >= test || measure < 0.0f;
	}


	vector<float> computeOrientations(SiftFeature2D & feature, Mat & image)
    {
        vector<float> ret;
        float roiSizef = 1.5f * feature.scale;
        float roiSizefb = roiSizef + 0.0f;
        Mat g = getGaussianImage(Size(roiSizefb, roiSizefb), 1.5f * feature.scale);
        Mat roi = image(Rect(feature.x - roiSizefb*0.5f, feature.y - roiSizefb*0.5f, (int)roiSizefb, (int)roiSizefb));

        Mat angles = Mat::zeros(Size(36, 1), CV_32FC1);
        for(int y = 1; y < roi.size().height-1; y++)
        {
            for(int x = 1; x < roi.size().width-1; x++)
            {
                float O = computeOrientation(x, y, roi);
                int bin = (int)(O / 10.0f);
                if(bin >= 0 && bin < 36)
                    angles.at<float>(0, bin) += computeMagnitude(x,y, roi) * g.at<float>(x,y);
            }
        }
        ll_normalize(angles);

        float best = angles.at<float>(0,0);
        int bestInd = 0;

        for(int i = 0; i < 36; i++)
        {
            float v = angles.at<float>(0, i);
            if(v >= 0.8f) ret.push_back(10.0f * (float)i);
            if(best < v)
            {
                best = v;
                bestInd = ret.size() - 1;
            }
        }
        return ret;
    }


    void computeFeatureVectors(SiftFeature2D & ret, Mat & im)
    {
        ret.featureVector = Mat::zeros(Size(8, 16), CV_32FC1);
        //Mat roi = im(Rect(ret.x - 7, ret.y - 7, 16, 16));
        Mat g = getGaussianImage(Size(16, 16), 16.0);

        int X = ret.x - 7;
        int Y = ret.y - 7;
        //X=Y=0;
        for(int y = 0; y < 16; y++)
        {
            for(int x = 0; x < 16; x++)
            {
                float m = computeMagnitude(x + X, y + Y, im) * g.at<float>(y,x);
                float o = computeOrientation(x + X, y + Y, im) - ret.angle;
                if(o < 0.0f) o = o + 360.0f;
                int xI = x / 4;
                int yI = y / 4;
                int GridSection = yI * 4 + xI;
                int bin = (int)(o / 45.0f);
                if(bin < 8 && bin >= 0)
                    ret.featureVector.at<float>(GridSection, bin) += m;
            }
        }
        ll_normalize(ret.featureVector);
    }


    void findFeatures(vector<SiftFeature2D> & ret, int octave, Mat & input)
	{
		double K = sqrt(2.0);
		Size kernelSize(16,16);
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
			for (int y = 17; y < input.size().height - 17; y++)
			{
				for (int x = 17; x < input.size().width - 17; x++)
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
                        if (!isCorner2(x, y, currentG)) continue;

						SiftFeature2D feature; feature.x = x; feature.y = y;
						feature.octave = octave;
						feature.scale = R/K;
						vector<float> bestAngles = computeOrientations(feature, currentG);
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
		//return ret;
	}


    std::vector<SiftFeature2D> findFeatures(cv::Mat & input, int octave)
    {
        findMultiScaleFeatures(input, 1);
    }

    vector<SiftFeature2D> findMultiScaleFeatures(Mat & input, int numOctaves)
    {
        vector<SiftFeature2D> ret;
        for(int i = 0; i < numOctaves; i++)
        {
            int octave = i;
            double newWidth = input.size().width / pow(2.0, (double)octave);
            double newHeight = input.size().height / pow(2.0, (double)octave);
            Size ns((int)(newWidth + 0.5), (int)(newHeight + 0.5));

            Mat x;
            resize(input, x, ns);
            findFeatures(ret, octave, x);
            //imshow("x", x);
            //waitKey();

        }
        return ret;
    }

}

