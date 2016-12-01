#include "LSift.h"
#include "../basics/R3.h"

using namespace cv;
using namespace ll_R3;
using namespace std;

namespace LukeLincoln
{
	double correlate(cv::Mat &im_float_1, cv::Mat &im_float_2) {

		// convert data-type to "float"

		int n_pixels = im_float_1.rows * im_float_1.cols;

		// Compute mean and standard deviation of both images
		cv::Scalar im1_Mean, im1_Std, im2_Mean, im2_Std;
		meanStdDev(im_float_1, im1_Mean, im1_Std);
		meanStdDev(im_float_2, im2_Mean, im2_Std);

		// Compute covariance and correlation coefficient
		double covar = (im_float_1 - im1_Mean).dot(im_float_2 - im2_Mean) / n_pixels;
		double correl = covar / (im1_Std[0] * im2_Std[0]);

		return correl;
	}
	double SiftFeature2D::sad(const SiftFeature2D & o)
	{
		return sum(abs(featureVector - o.featureVector))[0];
	}

	double SiftFeature3D::sad(const SiftFeature3D & o)
	{
		return sum(abs(featureVector - o.featureVector))[0];
	}

    double SiftFeature2D::correlation(const SiftFeature2D & o)
    {
		Mat oo = o.featureVector;
		return correlate(featureVector, oo);
	}

    Point2f SiftFeature2D::truePoint()
    {
        return Point2f(x * pow(2.0f, (double)octave), y * pow(2.0f, (double)octave));
    }

    R3 SiftFeature3D::truePoint()
    {
        return R3(x * pow(2.0f, (double)octave), y * pow(2.0f, (double)octave), z * pow(2.0f, (double)octave));
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

	VMat getGaussianImage(int s, double sigma)
	{
        VMat ret = s;
        float hs = s * 0.5f;
        R3 c(hs, hs, hs);
        //ret.setAll(0.0f);
        for(int z = 0; z < ret.s; z++)
        {
            for (int y = 0; y < ret.s; y++)
            {
                for (int x = 0; x < ret.s; x++)
                {
                    R3 p((float)x, (float)y, (float)z);
                    ret.at(x, y, z) = (float)gaussian(p.dist(c), sigma);
                }
            }
        }
        return ret;
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

	VMat getGaussianDifferenceImage(int s, double sigma, double K)
	{
        VMat ret = s;
        float hs = s * 0.5f;
        R3 c(hs, hs, hs);
        //ret.setAll(0.0f);
        for(int z = 0; z < ret.s; z++)
        {
            for (int y = 0; y < ret.s; y++)
            {
                for (int x = 0; x < ret.s; x++)
                {
                    R3 p((float)x, (float)y, (float)z);
                    ret.at(x, y, z) = (float)gaussianDifference(p.dist(c), sigma, K);
                }
            }
        }
        return ret;
	}

    void partialDerivatives1(Mat & im, int x, int y, float & dx, float & dy)
    {
        dx = im.at<float>(y, x + 1) - im.at<float>(y, x - 1);
        dy = im.at<float>(y + 1, x) - im.at<float>(y - 1, x);
    }

    void partialDerivatives1(VMat & im, int x, int y, int z, float & dx, float & dy, float & dz)
    {
        dx = im.at(x+1, y, z) - im.at(x-1, y, z);
        dy = im.at(x, y+1, z) - im.at(x, y-1, z);
        dz = im.at(x, y, z+1) - im.at(x, y, z-1);
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

    void partialDerivatives3(VMat & im, int x, int y, int z, float & dx, float & dy, float & dz, float & dxy, float & dxz, float & dyz)
    {
        dx =    im.at(x-1, y, z) - 2.0f*im.at(x, y, z) + im.at(x+1, y, z);
        dy =    im.at(x,y-1,z) - 2.0f*im.at(x,y,z) + im.at(x,y+1,z);
        dz =    im.at(x,y,z-1) - 2.0f*im.at(x,y,z) + im.at(x,y,z+1);
        dxy =   (im.at(x+1,y+1,z) - im.at(x-1,y+1,z) - im.at(x+1,y-1,z) + im.at(x-1,y-1,z) ) * 0.25f;
        dxz =   (im.at(x+1,y,z+1) - im.at(x-1,y,z+1) - im.at(x+1,y,z-1) + im.at(x-1,y,z-1) ) * 0.25f;
        dyz =   (im.at(x,y+1,z+1) - im.at(x,y-1,z+1) - im.at(x,y+1,z-1) + im.at(x,y-1,z-1) ) * 0.25f;

    }


    float computeOrientation(int x, int y, Mat & m)
	{
        float dx, dy;
        dy = m.at<float>(y + 1, x) - m.at<float>(y - 1, x);
        dx = m.at<float>(y, x + 1) - m.at<float>(y, x - 1);

        return R3::getAngle(dx, dy);
    }

    Point2f computeOrientation(int x, int y, int z, VMat & m)
	{
        R3 d;
        Point2f p;
        partialDerivatives1(m, x, y, z, d.x, d.y, d.z);
        float mag = d.mag();
        if(mag == 0.0f) return p;
        p.x = atan(d.y / d.x) * 57.3f;
        p.y = acos(d.z / mag) * 57.3f;
    }

    float computeMagnitude(int x, int y, Mat & m)
	{
        float dx, dy;
        dy = m.at<float>(y + 1, x) - m.at<float>(y - 1, x);
        dx = m.at<float>(y, x + 1) - m.at<float>(y, x - 1);

        return sqrt(dx * dx + dy * dy);
	}

	float computeMagnitude(int x, int y, int z, VMat & m)
	{
        R3 d;
        partialDerivatives1(m, x, y, z, d.x, d.y, d.z);
        return d.mag();
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
		float tr = dxx*dxx + dyy*dyy;
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
		return measure < test;
        //return measure >= test || measure < 0.0f;
	}

    bool isCorner2(int x, int y, int z, VMat & m)
    {
		float dx, dy, dxy, dxz, dyz, dz;
        partialDerivatives3(m, x, y, z, dx, dy, dz, dxy, dxz, dyz);
        float _ptr[9] = {
            dx,     dxy,    dxz,
            dxy,    dy,     dyz,
            dxz,    dyz,    dz
        };
        Mat hesMat = Mat(Size(3,3), CV_32FC1, _ptr);

        //compute trace and determinant
		float tr = trace(hesMat)[0];
		float det = determinant(hesMat);



        //if determinant is 0, return true
        if(det == 0.0f) return false;

        //compute the measure
		float measure = (tr*tr) / det;

        //std::cout << measure << std::endl;

		float r = 10.0f;
		float r1 = r + 1.0f;
		float test = (r1*r1) / r;
		//float test = 12.0f;
		return measure >= test || measure < 0.0f;
        //return measure >= test || measure < 0.0f;
	}


	vector<float> computeOrientations(SiftFeature2D & feature, Mat & image)
    {
        vector<float> ret;
        float roiSizef = 5.0f * feature.scale;
        float roiSizefb = roiSizef + 0.0f;
        Mat g = getGaussianImage(Size(roiSizefb, roiSizefb), roiSizef);
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

	vector<Point2f> computeOrientations(SiftFeature3D & feature, VMat & image)
	{
		vector<Point2f> ret;
		float roiSizef = 5.0f * feature.scale;
		int rsf = (int)(roiSizef + 0.5f);
		VMat g = getGaussianImage(rsf, roiSizef);
		int rsfh = (int)(roiSizef * 0.5f);
		
		//angles: 36 bins * 2 angles

		Mat angles = Mat::zeros(Size(18, 18), CV_32FC1);
		
		
		for (int z = feature.z - rsfh, _z = 0; z <= feature.z + rsfh; z++, _z++)
		{
			for (int y = feature.y - rsfh, _y = 0; y <= feature.y + rsfh; y++, _y++)
			{
				for (int x = feature.x - rsfh, _x = 0; x <= feature.x + rsfh; x++, _x++)
				{
					Point2f O = computeOrientation(x, y, z, image);
					int binX = (int)((O.x + 90.0f) / 10.0f);
					int binY = (int)((O.y) / 10.0f);
					if (binX >= 0 && binX < 18 && binY >= 0 && binY < 18)
						angles.at<float>(binY, binX) += computeMagnitude(x, y, z, image) * g.at(_x, _y, _z);
				}
			}
		}
		ll_normalize(angles);

		//float best = angles.at<float>(0, 0);
		//Point2i bestInd(0,0);

		for (int y = 0; y < 18; y++)
		{
			for (int x = 0; x < 18; x++)
			{
				float v = angles.at<float>(y, x);
				if (v >= 0.8f) ret.push_back(Point2f(x * 10.0f - 90.0f, y * 10.0f));
				/*if (best < v)
				{
					best = v;
					bestInd = ret.size() - 1;
				}*/
			}
		}
		
		return ret;
	}


    void computeFeatureVectors(SiftFeature2D & ret, Mat & im)
    {
        ret.featureVector = Mat::zeros(Size(8, 16), CV_64FC1);
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
				if (o > 360.0f) o = o - 360.0f;
				if (o < 0.0f || o > 360.0f) continue;
                int xI = x / 4;
                int yI = y / 4;
                int GridSection = yI * 4 + xI;
                int bin = (int)(o / 45.0f);
                if(bin < 8 && bin >= 0)
                    ret.featureVector.at<double>(GridSection, bin) += (double)m;
            }
        }
        //ll_normalize(ret.featureVector);
    }

	void computeFeatureVectors(SiftFeature3D & ret, VMat & im)
	{

		int binSize = 4;
		int binsSize = 4 * 4; // == 16
		int gridSize = 4 * 4 * 4; //== 64
		int totalSize = gridSize * binsSize; // == 1024

		ret.featureVector = Mat::zeros(Size(totalSize, 1), CV_64FC1);
		VMat g = getGaussianImage(16, 16.0);

		int X = ret.x - 7;
		int Y = ret.y - 7;
		int Z = ret.z - 7;

		Point2f fo(ret.angle1, ret.angle2);
		fo.x = fo.x < 0.0f ? fo.x + 180.0f : fo.x;

		//X=Y=0;
		for (int z = 0; z < 16; z++)
		{
			for (int y = 0; y < 16; y++)
			{
				for (int x = 0; x < 16; x++)
				{
					float m = computeMagnitude(x + X, y + Y, z+Z, im) * g.at(x, y, z);
					Point2f o = computeOrientation(x + X, y + Y, z + Z, im);
					o.x = o.x < 0.0f ? o.x + 180.0f : o.x;
					
					o.x -= fo.x;
					o.x = o.x < 0.0f ? 180.0f + o.x : o.x;
					o.y -= fo.y;
					o.y = o.y < 0.0f ? 180.0f + o.y : o.y;


					if (o.x < 0.0f || o.x > 180.0f) continue;
					if (o.y < 0.0f || o.y > 180.0f) continue;
					int xI = x / 4;
					int yI = y / 4;
					int zI = z / 4;
					int GridSection = zI * 16 + yI * 4 + xI;
					int bin1 = (int)(o.x / 45.0f);
					int bin2 = (int)(o.y / 45.0f);
					if (bin1 < 4 && bin1 >= 0 && bin2 >= 0 && bin2 < 4)
						ret.featureVector.at<double>(0, GridSection*binsSize + bin2*4 + bin1) += (double)m;
				}
			}
		}
		//ll_normalize(ret.featureVector);
	}


    void findFeatures(vector<SiftFeature2D> & ret, int octave, Mat & input)
	{
		double K =  sqrt(2.0);
		Size kernelSize(14,14);
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


	void findFeatures(vector<SiftFeature3D> & ret, int octave, VMat & input)
	{
		double K =  sqrt(2.0);
		int kernelSize = 7;//was 14 but very slow
		int numLevels = 3;



		double R = K * 0.5;
		VMat g = getGaussianImage(kernelSize, R);
		VMat d = getGaussianDifferenceImage(kernelSize, R, K);
		VMat prevG = input; prevG.filter(g);
		VMat prevD = input; prevD.filter(d);



		R = R * K;
		g = getGaussianImage(kernelSize, R);
		d = getGaussianDifferenceImage(kernelSize, R, K);
		VMat currentG = input; currentG.filter(g);
		VMat currentD = input; currentD.filter(d);

		for (int i = 2; i <= numLevels; i++)
		{
			R = R * K;
			g = getGaussianImage(kernelSize, R);
			d = getGaussianDifferenceImage(kernelSize, R, K);
			VMat nextG = input; nextG.filter(g);
			VMat nextD = input; nextD.filter(d);


			//for each pixel, if feature vector: record
			for(int z = 17; z < input.s - 17; z++)
			{
                for (int y = 17; y < input.s - 17; y++)
                {
                    for (int x = 17; x < input.s - 17; x++)
                    {
                        float dv = currentD.at(x, y, z);
                        int lt = 0, gt = 0;
                        for(int r = -1; r <= 1; r++)
                        {
                            for (int j = -1; j <= 1; j++)
                            {
                                for (int i = -1; i <= 1; i++)
                                {
                                    if (i != 0 && j != 0 && r != 0)
                                    {
                                        if (dv > currentD.at(x+i, y+j, z+r)) lt++;
                                        if (dv < currentD.at(x+i, y+j, z+r)) gt++;
                                    }
                                    if (dv > prevD.at(x+i, y+j, z+r)) lt++;
                                    if (dv < prevD.at(x+i, y+j, z+r)) gt++;

                                    if (dv > nextD.at(x+i, y+j, z+r)) lt++;
                                    if (dv < nextD.at(x+i, y+j, z+r)) gt++;
                                }
                            }
                        }

                        if (lt == 0 || gt == 0)
                        {
                            //Point2i __(x,y,z);
                            //R3 __(x,y,z);
                            //cout << "found feature " << __ << endl;
                            if (!isCorner2(x, y, z, currentG)) continue;

                            SiftFeature3D feature; feature.x = x; feature.y = y; feature.z = z;
                            feature.octave = octave;
                            feature.scale = R/K;

                            
                            vector<Point2f> bestAngles = computeOrientations(feature, currentG);
                            for(int _ = 0; _ < bestAngles.size(); _++)
                            {
                                SiftFeature3D f = feature;
                                f.angle1 = bestAngles[_].x;
								f.angle2 = bestAngles[_].y;
                                computeFeatureVectors(f, currentG);
                                ret.push_back(f);
                            }
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

    std::vector<SiftFeature3D> findFeatures(VMat & input, int octave)
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

    vector<SiftFeature3D> findMultiScaleFeatures(VMat & input, int numOctaves)
    {
        vector<SiftFeature3D> ret;
        for(int i = 0; i < numOctaves; i++)
        {
            int octave = i;
            double ns = input.s / pow(2.0, (double)octave);

            VMat x = input.resize(ns);
            findFeatures(ret, octave, x);
        }
        return ret;
    }

	void computeMatches(vector<SiftFeature2D> & fvs1, vector<SiftFeature2D> & fvs2, vector<Point2i> & p1, vector<Point2i> & p2, bool sort, int limit)
	{
		vector<tuple<SiftFeature2D, SiftFeature2D, double>> matches;

		//correl norm	= 32
		//correl		= 32
		//sad norm		= 36
		//sad			= 49

		for (int i = 0; i < fvs1.size(); i++)
		{
			double best = fvs1[i].sad(fvs2[0]);
			int bj = 0;
			for (int j = 1; j < fvs2.size(); j++)
			{
				double er = fvs1[i].sad(fvs2[j]);
				if (er < best)
				{
					best = er;
					bj = j;
				}
			}

			tuple<SiftFeature2D, SiftFeature2D, double> match = make_tuple(fvs1[i], fvs2[bj], best);
			matches.push_back(match);

		}

		if (sort)
		{
			//std::cout << "sorting " << "\n";
			auto f = [](const tuple<SiftFeature2D, SiftFeature2D, double> & a, const tuple<SiftFeature2D, SiftFeature2D, double> & b)
				-> bool { return get<2>(a) < get<2>(b); };
			std::sort(matches.begin(), matches.end(), f);

			int am = limit == -1 ? matches.size() : limit;
			int N = am < matches.size() ? am : matches.size();
			for (int i = 0; i < N; i++)
			{
				tuple<SiftFeature2D, SiftFeature2D, double> & match = matches[i];
				p1.push_back(get<0>(match).truePoint());
				p2.push_back(get<1>(match).truePoint());
			}

		}
		else
		{
			for (int i = 0; i < matches.size(); i++)
			{
				tuple<SiftFeature2D, SiftFeature2D, double> & match = matches[i];
				p1.push_back(get<0>(match).truePoint());
				p2.push_back(get<1>(match).truePoint());
			}
		}




	}

	void computeMatches(vector<SiftFeature3D> & fvs1, vector<SiftFeature3D> & fvs2, vector<Point3i> & p1, vector<Point3i> & p2, bool sort, int limit)
	{
		vector<tuple<SiftFeature3D, SiftFeature3D, double>> matches;

		//correl norm	= 32
		//correl		= 32
		//sad norm		= 36
		//sad			= 49

		for (int i = 0; i < fvs1.size(); i++)
		{
			double best = fvs1[i].sad(fvs2[0]);
			int bj = 0;
			for (int j = 1; j < fvs2.size(); j++)
			{
				double er = fvs1[i].sad(fvs2[j]);
				if (er < best)
				{
					best = er;
					bj = j;
				}
			}

			tuple<SiftFeature3D, SiftFeature3D, double> match = make_tuple(fvs1[i], fvs2[bj], best);
			matches.push_back(match);

		}

		if (sort)
		{
			//std::cout << "sorting " << "\n";
			auto f = [](const tuple<SiftFeature3D, SiftFeature3D, double> & a, const tuple<SiftFeature3D, SiftFeature3D, double> & b)
				-> bool { return get<2>(a) < get<2>(b); };
			std::sort(matches.begin(), matches.end(), f);

			int am = limit == -1 ? matches.size() : limit;
			int N = am < matches.size() ? am : matches.size();
			for (int i = 0; i < N; i++)
			{
				tuple<SiftFeature3D, SiftFeature3D, double> & match = matches[i];
				R3 _p1p = get<0>(match).truePoint();
				R3 _p2p = get<1>(match).truePoint();
				p1.push_back(Point3i(_p1p.x, _p1p.y, _p1p.z));
				p2.push_back(Point3i(_p2p.x, _p2p.y, _p2p.z));
			}

		}
		else
		{
			for (int i = 0; i < matches.size(); i++)
			{
				tuple<SiftFeature3D, SiftFeature3D, double> & match = matches[i];
				R3 _p1p = get<0>(match).truePoint();
				R3 _p2p = get<1>(match).truePoint();
				p1.push_back(Point3i(_p1p.x, _p1p.y, _p1p.z));
				p2.push_back(Point3i(_p2p.x, _p2p.y, _p2p.z));
			}
		}




	}

	cv::Point2f operator*(cv::Mat M, const cv::Point2f& p)
	{
		Mat src = Mat::zeros(Size(1, 3), CV_64FC1);

		src.at<double>(0, 0) = p.x;
		src.at<double>(1, 0) = p.y;
		src.at<double>(2, 0) = 1.0;

		Mat dst = M*src; //USE MATRIX ALGEBRA
		return cv::Point2f(dst.at<double>(0, 0), dst.at<double>(1, 0));
	}

	cv::Point3f operator*(cv::Mat M, const cv::Point3f& p)
	{
		Mat src = Mat::zeros(Size(1, 4), CV_32FC1);

		src.at<float>(0, 0) = p.x;
		src.at<float>(1, 0) = p.y;
		src.at<float>(2, 0) = p.z;
		src.at<float>(3, 0) = 1.0;

		Mat dst = M*src; //USE MATRIX ALGEBRA
		return cv::Point3f(dst.at<float>(0, 0), dst.at<float>(1, 0), dst.at<float>(2,0));
	}

	void lukes_sift(cv::Mat & A, cv::Mat & B, std::vector<cv::Point2i> & p1, std::vector<cv::Point2i> & p2, bool sort, int top)
	{
		Mat im1, im2;
		if (A.type() == CV_32FC1) im1 = A; else { im1 = A.clone(); ll_UCF1_to_32F1(im1); }
		if (B.type() == CV_32FC1) im2 = B; else { im2 = B.clone(); ll_UCF1_to_32F1(im2); }

		vector<SiftFeature2D> f1 = findMultiScaleFeatures(im1, 3);
		vector<SiftFeature2D> f2 = findMultiScaleFeatures(im2, 3);

		computeMatches(f1, f2, p1, p2, sort, top);
	}

	cv::Mat featureVisualization(std::vector<SiftFeature2D> & f, cv::Mat im)
	{
		auto x = im.clone();
		for (int i = 0; i < f.size(); i++)
		{
			Point2i p1 = f[i].truePoint();
			float X, Y;
			R3::GetUnitPointFromAngle(f[i].angle, X, Y);
			double rad = 2.0 * f[i].trueRad();
			X *= rad, Y *= rad;
			Point2i p2(p1.x + (int)X, p1.y + (int)Y);
			cv::line(x, p1, p2, Scalar(255));
			cv::circle(x, p1, rad, Scalar(255));
		}
		return x.clone();
	}

	void testFeatures(Mat & im, double R, double S, Point2d T)
	{

		Mat M = ll_transformation_matrix(im.size(), R, S, T.x, T.y);
		Mat im2; ll_transform_image(im, im2, M);



		vector<LukeLincoln::SiftFeature2D> f1 = findMultiScaleFeatures(im, 3);
		vector<SiftFeature2D> f2 = findMultiScaleFeatures(im2, 3);

		Mat check = Mat::zeros(im.size(), CV_8UC1);

		int Total = f1.size();
		int Count = 0;

		for (int i = 0; i < f2.size(); i++)
		{
			Point2i p = f2[i].truePoint();
			if (p.x >= 0 && p.x < check.size().width && p.y >= 0 && p.y < check.size().height) check.at<unsigned char>(p) = 1;
		}

		for (int i = 0; i < f1.size(); i++)
		{
			Point2f _ = M * f1[i].truePoint();
			Point2i p = _;
			if (p.x >= 0 && p.x < check.size().width && p.y >= 0 && p.y < check.size().height)
				if (check.at<unsigned char>(p) == 1) Count++;

		}

		cout << "testing features: " << Count << " found in other image out of " << Total;
		cout << ", " << (100.0*Count / (double)Total) << "% were found." << endl;

	}

	void testFeatures(VMat & im, R3 R, float S, R3 T)
	{

		Mat M = VMat::transformation_matrix(im.s, R.x, R.y, R.z, S, T.x, T.y, T.z);
		VMat im2 = im; im2.transform_volume_forward(M, 0.0f);



		vector<SiftFeature3D> f1 = findMultiScaleFeatures(im, 1);
		vector<SiftFeature3D> f2 = findMultiScaleFeatures(im2, 1);

		bool * check = new bool[im.s3]; for(int i = 0; i < im.s3; i++) check[i] = false;

		int Total = f1.size();
		int Count = 0;

		for (int i = 0; i < f2.size(); i++)
		{
			R3 _p = f2[i].truePoint();
			Point3i p(_p.x, _p.y, _p.z);
			if (p.x >= 0 && p.x < im.s && p.y >= 0 && p.y < im.s && p.z >= 0 && p.z < im.s)
                check[p.z * im.s2 + p.y * im.s + p.x] = true;
		}

		for (int i = 0; i < f1.size(); i++)
		{
			R3 _ = f1[i].truePoint();
			VMat::transform_point(M, _);

			Point3i p(_.x, _.y, _.z);
			if (p.x >= 0 && p.x < im.s && p.y >= 0 && p.y < im.s && p.z >= 0 && p.z < im.s)
				if (check[p.z * im.s2 + p.y * im.s + p.x]) Count++;

		}

		cout << "testing features: " << Count << " found in other image out of " << Total;
		cout << ", " << (100.0*Count / (double)Total) << "% were found." << endl;

		delete [] check;

	}

	void testMatches(cv::Mat & im, double R, double S, cv::Point2d T, bool sort, int top)
	{
		Mat M = ll_transformation_matrix(im.size(), R, S, T.x, T.y);

		Mat im2;

		ll_transform_image(im, im2, R, S, T.x, T.y);


		vector<LukeLincoln::SiftFeature2D> f1 = findMultiScaleFeatures(im, 3);
		vector<SiftFeature2D> f2 = findMultiScaleFeatures(im2, 3);

		vector<Point2i> p1, p2;
		computeMatches(f1, f2, p1, p2, sort, top);



		int Count = 0, Total = 0;

		vector<Point2i> gp1, gp2;

		for (int i = 0; i < p1.size(); i++)
		{
			Point2f pa(p1[i].x, p1[i].y);
			Point2f pb(p2[i].x, p2[i].y);
			pa = M * pa;
			float dist = ll_distance<float>(pa.x, pa.y, pb.x, pb.y);
			if (dist < 1.5f)
			{
				Count++;
				gp1.push_back(p1[i]);
				gp2.push_back(p2[i]);
			}

			Total++;
		}
		cout << "testing matches: " << Count << " correct out of " << Total;
		cout << ", " << (100.0*Count / (double)Total) << "% were accurate." << endl;
	}

	void testMatches(VMat & im, R3 R, float S, R3 T, bool sort, int top)
	{
		
		Mat M = VMat::transformation_matrix(im.s, R.x, R.y, R.z, S, T.x, T.y, T.z);

		VMat im2 = im; im2.transform_volume_forward(M);


		vector<LukeLincoln::SiftFeature3D> f1 = findMultiScaleFeatures(im, 1);
		vector<SiftFeature3D> f2 = findMultiScaleFeatures(im2, 1);

		vector<Point3i> p1, p2;
		computeMatches(f1, f2, p1, p2, sort, top);



		int Count = 0, Total = 0;

		for (int i = 0; i < p1.size(); i++)
		{
			Point3f pa(p1[i].x, p1[i].y, p1[i].z);
			Point3f pb(p2[i].x, p2[i].y, p2[i].z);
			pa = M * pa;
			Point3f dv = pa - pb;
			float dist = sqrt(dv.dot(dv));
			if (dist < 1.5f)
				Count++;
		
			Total++;
		}
		cout << "testing matches: " << Count << " correct out of " << Total;
		cout << ", " << (100.0*Count / (double)Total) << "% were accurate." << endl;
	}


	Mat lukes_siftRegister(Mat & a, Mat & b, bool sort, int getTopXFeatures, double allowedError)
	{
		vector<Point> pnts1, pnts2;
		Mat c;
		lukes_sift(a, b, pnts1, pnts2, sort, getTopXFeatures);
		Mat hm = findHomography(pnts1, pnts2, CV_RANSAC, allowedError);
		return hm.clone();
	}
}

