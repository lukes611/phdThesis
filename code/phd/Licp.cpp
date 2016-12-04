#include "Licp.h"
#include "../basics/LTimer.h"
#include "../basics/Pixel3DSet.h"
using namespace ll_R3;
using namespace ll_pix3d;
using namespace std;
using namespace cv;

//#define USE_KD

namespace Licp
{


Mat asMatRows(vector<R3> & inp)
{
	Mat ret = Mat::zeros(inp.size(), 3, CV_32FC1);
	for(int i = 0; i < inp.size(); i++)
	{
		ret.at<float>(i,0) = inp[i].x;
		ret.at<float>(i,1) = inp[i].y;
		ret.at<float>(i,2) = inp[i].z;
	}
	return ret.clone();
}

Mat asMatRows(Pixel3DSet & inp)
{
	Mat ret = Mat::zeros(inp.size(), 4, CV_32FC1);
	for(int i = 0; i < inp.size(); i++)
	{
		ret.at<float>(i,0) = inp[i].x;
		ret.at<float>(i,1) = inp[i].y;
		ret.at<float>(i,2) = inp[i].z;
		ret.at<float>(i,3) = inp.gsNPixel(i) * 255.0f;
	}
	return ret.clone();
}

float knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs)
{
    vector<float> dists;
    return knn(m_destinations, m_object, ptpairs, dists);
}

float knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists)
{
    // find nearest neighbors using FLANN
    cv::Mat m_indices(m_object.rows, 1, CV_32S);
    cv::Mat m_dists(m_object.rows, 1, CV_32F);

    Mat dest_32f; m_destinations.convertTo(dest_32f,CV_32FC2);
    Mat obj_32f; m_object.convertTo(obj_32f,CV_32FC2);

    assert(dest_32f.type() == CV_32F);
	//cout << "h" << endl;
    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
    flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64) );
	//cout << "p" << endl;
    int* indices_ptr = m_indices.ptr<int>(0);
    //float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) {
        ptpairs.push_back(indices_ptr[i]);
    }

    dists.resize(m_dists.rows);
    m_dists.copyTo(Mat(dists));

    return cv::sum(m_dists)[0];
}

double closestPoints(vector<R3> & src, vector<R3> & dst, vector<int> & indexes, vector<double> & distances)
{
	//clear the output
	indexes.clear(); distances.clear();
	double averageDistance = 0.0f;

	if(dst.size() <= 0 || src.size() <= 0) return averageDistance; //if dst has no elements: return
	double scalar = 1.0 / (double) src.size(); //scalar for computing the averageDistane
	for(int i = 0; i < src.size(); i++) //for each of the src input
	{
		R3 srcPoint = src[i];
		double bestDistance = (double)srcPoint.dist(dst[0]);
		int bestIndex = 0;
		for(int j = 1; j < dst.size(); j++) //search for the closest point in dst
		{
			double dist = (double)srcPoint.dist(dst[j]);
			if(dist < bestDistance)
			{
				bestIndex = j;
				bestDistance = dist;
			}
		}
		distances.push_back(bestDistance);
		indexes.push_back(bestIndex);
		averageDistance += bestDistance * scalar;
	}
	return averageDistance;
}

double closestPointsf(vector<R3> & src, vector<R3> & dst, vector<int> & indexes, vector<double> & distances)
{
	distances.clear();
	indexes.clear();
	#ifndef USE_KD

	Mat srcm = asMatRows(src), dstm = asMatRows(dst);
	vector<float> dists;
	double error = (double) knn(dstm, srcm, indexes, dists);
	for(int i = 0; i < dists.size(); i++) distances.push_back((double)dists[i]);
	double divisor = src.size() > 0 ? (double)(src.size()) : 1.0;
	return error / divisor;

	#else

    Pixel3DSet p1(src);
    Pixel3DSet p2(dst);
    LKDNode n; n.init(p2);
    n.split(p2);

    double error = 0.0;
    for(int i = 0; i < src.size(); i++)
    {
        R3 p = src[i];
        R3 w;
        int index = 0;
        n.NN(p2,p, index, w);
        indexes.push_back(index);
        float D = p.dist(p2[index]);
        distances.push_back(D);
        error += (D*D) / (float) src.size();
    }
    return error;
	#endif
}

double closestPointsf(Pixel3DSet & src, Pixel3DSet & dst, vector<int> & indexes, vector<double> & distances)
{
    //cout << "timing " << endl;
    double error;
    //LTimer t; t.start();
	distances.clear();
	indexes.clear();
	#ifndef USE_KD
	Mat srcm = asMatRows(src), dstm = asMatRows(dst);
	vector<float> dists;
	error = (double) knn(dstm, srcm, indexes, dists);
	for(int i = 0; i < dists.size(); i++) distances.push_back((double)dists[i]);
	double divisor = src.size() > 0 ? (double)(src.size()) : 1.0;
	error /= divisor;
	#else


	LKDNode n; n.init(dst);
    n.split(dst);
    cout << n.averageLeafSize() << endl;
    error = 0.0;
    for(int i = 0; i < src.size(); i++)
    {
        R3 p = src[i];
        R3 w;
        int index = 0;
        n.NN(dst,p, index, w);
        if(i%1000 == 0 )cout << i << " / " << src.size() << endl;
        indexes.push_back(index);
        float D = p.dist(dst[index]);
        distances.push_back(D);
        error += (D*D) / (float) src.size();
    }


	#endif
    //t.stop();
    //cout << "took " << t.getSeconds() << endl;

	return error;
}

Mat leastSquaresTransform(vector<R3> & src, vector<R3> & dst, vector<int> & indexes)
{
	//M x src = dst
	Mat ret;
	Mat src_m = Mat::zeros(4, indexes.size(), CV_32FC1);
	Mat dst_m = Mat::zeros(4, indexes.size(), CV_32FC1);
	for(int i = 0; i < indexes.size(); i++)
	{
		src_m.at<float>(0, i) = src[i].x;
		src_m.at<float>(1, i) = src[i].y;
		src_m.at<float>(2, i) = src[i].z;
		src_m.at<float>(3, i) = 1.0f;

		int j = indexes[i];

		dst_m.at<float>(0, i) = dst[j].x;
		dst_m.at<float>(1, i) = dst[j].y;
		dst_m.at<float>(2, i) = dst[j].z;
		dst_m.at<float>(3, i) = 1.0f;
	}
	//src^T x M^T = dst^T
	src_m = src_m.t();
	dst_m = dst_m.t();
	Mat Mt = least_squares(src_m, dst_m);
	Mat M = Mt.t();
	return M.clone();
}

Mat icp(vector<R3> & src, vector<R3> & dst, vector<R3> & out,
		double & error_out, double & time, int & iterations,
		double minError, int maxIterations)
{
	time = 0.0;
	LTimer clock; clock.start();
	out.clear();
	Mat ret = Mat::eye(Size(4,4), CV_32FC1);
	vector<double> dl;
	vector<int> ind;
	out = src;
	iterations = 0;

	double best_distance = DBL_MAX;
	error_out = best_distance;

	Mat T = ret.clone();

	while(true)
	{
		dl.clear(); ind.clear();
		double current_distance = closestPointsf(out, dst, ind, dl);
		if(current_distance >= best_distance) break; //cannot do any better
		ret = T * ret;
		error_out = current_distance;
		if(minError > 0.0 && current_distance < minError) break;
		if(maxIterations >= 0 && iterations >= maxIterations) break;
		best_distance = current_distance;
		T = leastSquaresTransform(out, dst, ind);
		for(int i = 0; i < out.size(); i++) ll_pix3d::Pixel3DSet::transform_point(T, out[i]);
		iterations++;
	}
	clock.stop();
	time = clock.getSeconds();
	return ret.clone();
}


Mat icp(Pixel3DSet & src, Pixel3DSet & dst, Pixel3DSet & out,
		double & error_out, double & time, int & iterations,
		double minError, int maxIterations)
{
	time = 0.0;
	LTimer clock; clock.start();
	out.clear();
	Mat ret = Mat::eye(Size(4,4), CV_32FC1);
	vector<double> dl;
	vector<int> ind;
	out = src;
	iterations = 0;

	double best_distance = DBL_MAX;
	error_out = best_distance;

	Mat T = ret.clone();

	while(true)
	{
		dl.clear(); ind.clear();
		double current_distance = closestPointsf(out, dst, ind, dl);
		if(current_distance >= best_distance) break; //cannot do any better
		ret = T * ret;
		error_out = current_distance;
		if(minError > 0.0 && current_distance < minError) break;
		if(maxIterations >= 0 && iterations >= maxIterations) break;
		best_distance = current_distance;
		T = leastSquaresTransform(out.points, dst.points, ind);
		for(int i = 0; i < out.size(); i++) ll_pix3d::Pixel3DSet::transform_point(T, out[i]);
		iterations++;
	}
	clock.stop();
	time = clock.getSeconds();
	return ret.clone();
}


Mat icp_outlierRemoval(vector<R3> & src, vector<R3> & _dst, vector<R3> & out,
					   double & error_out, double & time, int & iterations,
					   double outlierRemovalTimesMean, double minError,
					   int maxIterations)
{
	time = 0.0;
	LTimer clock; clock.start();
	out.clear();
	Mat ret = Mat::eye(Size(4,4), CV_32FC1);
	vector<double> dl;
	vector<int> ind;
	out = src;
	vector<R3> dst = _dst;
	iterations = 0;

	double best_distance = DBL_MAX;
	error_out = best_distance;

	Mat T = ret.clone();

	while(true)
	{
		dl.clear(); ind.clear();
		double current_distance = Licp::closestPointsf(out, dst, ind, dl);
		if(current_distance >= best_distance) break; //cannot do any better
		ret = T * ret;
		error_out = current_distance;
		if(minError > 0.0 && current_distance < minError) break;
		if(maxIterations >= 0 && iterations >= maxIterations) break;
		best_distance = current_distance;
		if(best_distance <= 0.0) break;

		//outlier removal
		//cout << current_distance << " is current, or is: " << outlierRemovalTimesMean << endl;
		vector<R3> srcTest, dstTest;
		vector<int> indTest;
		for(int i = 0; i < dl.size(); i++)
		{
			double test = dl[i] / best_distance;
			//cout << dl[i] << " : " << best_distance << " : " << test << " <= " << outlierRemovalTimesMean << endl;
			if(dl[i]/best_distance < outlierRemovalTimesMean)
			{
				srcTest.push_back(out[i]);
				dstTest.push_back(dst[ind[i]]);
				indTest.push_back(i);
			}
		}

		//cout << "i/o(" << out.size() << "/" << srcTest.size() << ")\n";

		T = Licp::leastSquaresTransform(srcTest, dstTest, indTest);
		for(int i = 0; i < out.size(); i++) ll_pix3d::Pixel3DSet::transform_point(T, out[i]);
		iterations++;
	}
	clock.stop();
	time = clock.getSeconds();
	return ret.clone();
}


Mat icp_outlierRemoval(Pixel3DSet & src, Pixel3DSet & _dst, Pixel3DSet & out,
					   double & error_out, double & time, int & iterations,
					   double outlierRemovalTimesMean, double minError,
					   int maxIterations)
{
	time = 0.0;
	LTimer clock; clock.start();
	out.clear();
	Mat ret = Mat::eye(Size(4,4), CV_32FC1);
	vector<double> dl;
	vector<int> ind;
	out = src;
	Pixel3DSet dst = _dst;
	iterations = 0;

	double best_distance = DBL_MAX;
	error_out = best_distance;

	Mat T = ret.clone();

	while(true)
	{
		dl.clear(); ind.clear();
		double current_distance = Licp::closestPointsf(out, dst, ind, dl);
		if(current_distance >= best_distance) break; //cannot do any better
		ret = T * ret;
		error_out = current_distance;
		if(minError > 0.0 && current_distance < minError) break;
		if(maxIterations >= 0 && iterations >= maxIterations) break;
		best_distance = current_distance;
		if(best_distance <= 0.0) break;

		//outlier removal
		//cout << current_distance << " is current, or is: " << outlierRemovalTimesMean << endl;
		vector<R3> srcTest, dstTest;
		vector<int> indTest;
		for(int i = 0; i < dl.size(); i++)
		{
			double test = dl[i] / best_distance;
			//cout << dl[i] << " : " << best_distance << " : " << test << " <= " << outlierRemovalTimesMean << endl;
			if(dl[i]/best_distance < outlierRemovalTimesMean)
			{
				srcTest.push_back(out[i]);
				dstTest.push_back(dst[ind[i]]);
				indTest.push_back(i);
			}
		}

		//cout << "i/o(" << out.size() << "/" << srcTest.size() << ")\n";

		T = Licp::leastSquaresTransform(srcTest, dstTest, indTest);
		for(int i = 0; i < out.size(); i++) ll_pix3d::Pixel3DSet::transform_point(T, out[i]);
		iterations++;
	}
	clock.stop();
	time = clock.getSeconds();
	return ret.clone();
}


}

