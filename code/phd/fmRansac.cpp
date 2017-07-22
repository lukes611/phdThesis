#include "fmRansac.h"
#include "../basics/LTimer.h"

using namespace cv;
using namespace ll_pix3d;
using namespace std;
using namespace ll_experiments;
using namespace ll_R3;


namespace ll_fmrsc
{

	int featureMatch(string algorithm, Pix3D & object1, Pix3D & object2, vector<Point2i> & p1, vector<Point2i> & p2, bool sort, int top)
	{
		//cout << "wow" << endl;
		p1.clear(); p2.clear();
		vector<Point2i> pts1, pts2;
		Mat colA, colB, vdA, vdB;

		//cout << object1.hasVd() << endl;
		//system("pause");

		object1.colorImage(colA); object2.colorImage(colB);
		object1.vdImage(vdA); object2.vdImage(vdB);

		//imshow("colA", colA);
		//waitKey();

		
		if(algorithm == "sift")
			ll_sift(colA, colB, pts1, pts2, sort, top);
		else //surf
			ll_surf(colA, colB, pts1, pts2, sort, top);
		for(int i = 0; i < pts1.size() && i < pts2.size(); i++)
		{
			if(vdA.at<unsigned char>(pts1[i]) != 0x00 && vdB.at<unsigned char>(pts2[i]) != 0x00)
			{
				p1.push_back(pts1[i]);
				p2.push_back(pts2[i]);
			}
		}
		return p1.size();
	}

	bool registerPix3D(string fm_algorithm, Pix3D & object1, Pix3D & object2, Mat & matrix, double & seconds, bool sort, int top)
	{
		seconds = 0.0;
		LTimer t; t.start();
		vector<Point2i> points1, points2;
		int numPoints = featureMatch(fm_algorithm, object1, object2, points1, points2, sort, top);
		
		vector<Point3f> src, dst;
		Mat ret;
		Mat inliers;
		for(int i = 0; i < numPoints; i++)
		{
			int index1 = points1[i].y * 640 + points1[i].x;
			int index2 = points2[i].y * 640 + points2[i].x;
			Point3f pointA(object1.points[index1].x, object1.points[index1].y, object1.points[index1].z);
			Point3f pointB(object2.points[index2].x, object2.points[index2].y, object2.points[index2].z);
			src.push_back( pointA );
			dst.push_back( pointB );
		}
		int rv = estimateAffine3D(src, dst, ret, inliers, 1.0, 0.999999999999);
		matrix = Mat::eye(Size(4,4), CV_32FC1);
		for(int y = 0; y < 3; y++)
		{
			for(int x = 0; x < 4; x++)
			{
				matrix.at<float>(y,x) = (float)ret.at<double>(y,x);
			}
		}
		t.stop();
		seconds = t.getSeconds();
		return rv == 1;
	}



	//kitti versions:
	int featureMatch(string algorithm, kitti::KittiPix3dSet & object1, kitti::KittiPix3dSet & object2, vector<Point2i> & p1, vector<Point2i> & p2, bool sort, int top)
	{
		p1.clear(); p2.clear();
		vector<Point2i> pts1, pts2;
		Mat colA, colB, vdA, vdB;


		colA = object1.colorImage.clone(); colB = object2.colorImage.clone();

		vdA = object1.validDepthImage.clone();
		vdB = object2.validDepthImage.clone();

		R3 tmp;

		if(algorithm == "sift")
			ll_sift(colA, colB, pts1, pts2, sort, top);
		else //surf
			ll_surf(colA, colB, pts1, pts2, sort, top);
		for(int i = 0; i < pts1.size() && i < pts2.size(); i++)
		{

			if(vdA.at<unsigned char>(pts1[i]) != 0x00 && vdB.at<unsigned char>(pts2[i]) != 0x00

                || (object1.getClosePoint(pts1[i], tmp) && object2.getClosePoint(pts2[i], tmp))

			)
			{
				p1.push_back(pts1[i]);
				p2.push_back(pts2[i]);
			}
		}
		return p1.size();
	}

	bool registerPix3D(string fm_algorithm, kitti::KittiPix3dSet & object1, kitti::KittiPix3dSet & object2, Mat & matrix, double & seconds, bool sort, int top)
	{
		seconds = 0.0;

		LTimer t; t.start();
		vector<Point2i> points1, points2;
		int numPoints = featureMatch(fm_algorithm, object1, object2, points1, points2, sort, top);
        cout << numPoints << endl;
		vector<Point3f> src, dst;
		Mat ret;
		Mat inliers;
		for(int i = 0; i < numPoints; i++)
		{
			//int index1 = points1[i].y * object1.colorImage.size().width + points1[i].x;
			//int index2 = points2[i].y * object1.colorImage.size().width + points2[i].x;
			R3 a, b;
			object1.getClosePoint(points1[i], a);
			object2.getClosePoint(points2[i], b);
			//Point3f pointA(object1.points[index1].x, object1.points[index1].y, object1.points[index1].z);
			//Point3f pointB(object2.points[index2].x, object2.points[index2].y, object2.points[index2].z);
			Point3f pointA(a.x, a.y, a.z);
			Point3f pointB(b.x, b.y, b.z);

			src.push_back( pointA );
			dst.push_back( pointB );
		}

		int rv = estimateAffine3D(src, dst, ret, inliers, 1.0, 0.999999999999);
		matrix = Mat::eye(Size(4,4), CV_32FC1);

		if(rv)
		{
            for(int y = 0; y < 3; y++)
            {
                for(int x = 0; x < 4; x++)
                {
                    matrix.at<float>(y,x) = (float)ret.at<double>(y,x);
                }
            }
		}else cout << "FM2D FAILED" << endl;
        t.stop();
		seconds = t.getSeconds();
        return rv == 1;
	}




}
