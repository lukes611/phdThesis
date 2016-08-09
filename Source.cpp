#include <iostream>
#include <string>
#include <vector>

#include "code\basics\locv3.h"
#include "code\basics\R3.h"
#include "code\basics\llCamera.h"
#include "code\basics\VMatF.h"
#include "code\basics\LTimer.h"


using namespace std;
using namespace ll_R3;
using namespace ll_cam;
//using namespace cv;

//proto-type for experiments: VMat pc(VMat a, VMat b, double & time, Mat & transform, double & mse);
//to-do:= pc, ipc, 

Mat r3List2MatRows44(vector<R3> & inp)
{
	Mat ret = Mat::zeros(inp.size(), 4, CV_32FC1);
	for(int i = 0; i < inp.size(); i++)
	{
		ret.at<float>(i,0) = inp[i].x;
		ret.at<float>(i,1) = inp[i].y;
		ret.at<float>(i,2) = inp[i].z;
		ret.at<float>(i,3) = 1.0f;
	}
	return ret.clone();
}

float flann_knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists = vector<float>()) {
    // find nearest neighbors using FLANN
    cv::Mat m_indices(m_object.rows, 1, CV_32S);
    cv::Mat m_dists(m_object.rows, 1, CV_32F);
 
    Mat dest_32f; m_destinations.convertTo(dest_32f,CV_32FC2);
    Mat obj_32f; m_object.convertTo(obj_32f,CV_32FC2);
 
    assert(dest_32f.type() == CV_32F);
 
    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
    flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64) ); 
 
    int* indices_ptr = m_indices.ptr<int>(0);
    //float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) {
        ptpairs.push_back(indices_ptr[i]);
    }
 
    dists.resize(m_dists.rows);
    m_dists.copyTo(Mat(dists));
 
    return cv::sum(m_dists)[0];
}

//a n^2 complexity search for the closest point for each point in src to dst,
//returns an array of mappings src[i] mapped to dst[array[i]]
// ipc wiki: https://en.wikipedia.org/wiki/Iterative_closest_point
// kind of good: http://www.mrpt.org/Iterative_Closest_Point_(ICP)_and_other_matching_algorithms
// //optimal resource: http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
// : least squares: https://en.wikipedia.org/wiki/Linear_least_squares_(mathematics)
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
	Mat srcm = r3List2MatRows44(src), dstm = r3List2MatRows44(dst);
	vector<float> dists;
	double error = (double)flann_knn(dstm, srcm, indexes, dists);
	for(int i = 0; i < dists.size(); i++) distances.push_back((double)dists[i]);
	double divisor = max(src.size(), dst.size()) > 0 ? (double)(src.size() * src.size()) : 1.0;
	return error / divisor;
}

//returns x from equation Mx = y
Mat least_squares(Mat M, Mat y){
	Mat mt = M.t();
	Mat pre = mt * M;
	pre = pre.inv();
	return pre * mt * y;
}

//part two: get matrix from the two point sets using least squares
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




float rp() {
	return (float)(rand() % 100);
}



void findBestReansformSVD(Mat& _m, Mat& _d) {
	
	

    Mat m; _m.convertTo(m,CV_32F);
    Mat d; _d.convertTo(d,CV_32F);
 
	

    Scalar d_bar = mean(d);
    Scalar m_bar = mean(m);
    Mat mc = m - m_bar;
    Mat dc = d - d_bar;

	
 
    mc = mc.reshape(1); dc = dc.reshape(1);
    
	

    Mat H = Mat::zeros(2, 2, CV_32FC1);//(2,2,CV_32FC1, Scalar(0));
    for(int i=0;i<mc.rows;i++) {
        Mat mci = mc(Range(i,i+1),Range(0,2));
        Mat dci = dc(Range(i,i+1),Range(0,2));
        H = H + mci.t() * dci;
    }
 
	

    cv::SVD svd(H);
 
	

    Mat R = svd.vt.t() * svd.u.t();

	

    double det_R = cv::determinant(R);
    if(abs(det_R + 1.0) < 0.0001) {
        float _tmp[4] = {1,0,0,cv::determinant(svd.vt*svd.u)};
        R = svd.u * Mat(2,2,CV_32FC1,_tmp) * svd.vt;
    }
    float* _R = R.ptr<float>(0);
    Scalar T(d_bar[0] - (m_bar[0]*_R[0] + m_bar[1]*_R[1]),d_bar[1] - (m_bar[0]*_R[2] + m_bar[1]*_R[3]));
 
	

    m = m.reshape(1);

	cout << R.size() << " " << m.size() << endl;
	system("pause");

    m = R * m;

	

    m = m.reshape(2);
    m = m + T;// + m_bar;
    m.convertTo(_m,CV_32S);

	
}

Mat icp(vector<R3> & src, vector<R3> & dst, vector<R3> & out, double & error_out, double & time, double minError = -1.0, int maxIterations = -1)
{
	time = 0.0;
	LTimer clock; clock.start();
	out.clear();
	Mat ret = Mat::eye(Size(4,4), CV_32FC1);
	vector<double> dl; vector<int> ind;
	out = src;
	

	double best_distance = DBL_MAX;
	error_out = best_distance;
	int iteration = 0;

	Mat T = ret.clone();

	while(true)
	{
		dl.clear(); ind.clear();
		double current_distance = closestPointsf(out, dst, ind, dl);
		if(current_distance >= best_distance) break; //cannot do any better
		ret = T * ret;
		error_out = current_distance;
		if(minError > 0.0 && current_distance < minError) break;
		if(maxIterations >= 0 && iteration >= maxIterations) break;
		best_distance = current_distance;
		T = leastSquaresTransform(out, dst, ind);
		for(int i = 0; i < out.size(); i++) Pixel3DSet::transform_point(T, out[i]);
		iteration++;
	}
	clock.stop();
	time = clock.getSeconds();
	return ret.clone();
}

//add time measurement and then speedup

template <class T>
void randomize(vector<T> & ar)
{
	int s = ar.size();
	auto rnum = [&s]() -> int {
		return rand() % s;
	};
	for(int i = 0; i < s*2; i++)
	{
		int a = rnum();
		int b = rnum();
		T x = ar[a];
		ar[a] = ar[b];
		ar[b] = x;
	}
}

int main(int argc, char * * argv)
{
	
	vector<R3> p1, p2;
	Mat km = VMat::transformation_matrix(100, 4.0f, 3.0f, 0.0f, 1.0f, 1.0f, 2.0f, 3.0f);

	for (int i = 0; i < 640 * 480; i++)
	{
		R3 point(rp(), rp(), rp());
		p1.push_back(point);
		R3 point2 = point;
		Pixel3DSet::transform_point(km, point2);
		p2.push_back(point2);
	}

	randomize(p1);

	bool PRINT = false;

	if(PRINT)
	{
		cout << "p1:" << endl;
		for(int i = 0; i < p1.size(); i++) cout << p1[i] << endl;
	}
	if(PRINT)
	{
		cout << "P2:" << endl;
		for(int i = 0; i < p2.size(); i++) cout << p2[i] << endl;
	}
	cout << "icp-ing...\n";

	double dist = DBL_MAX;

	vector<R3> out = p1; vector<int> tmp; vector<double> dst;
	double error = closestPoints(p1, p2, tmp, dst);

	//cout << "ind-luk**********\n";
	//for(int i = 0; i < tmp.size(); i++) cout << tmp[i] << endl;
	//cout << endl;

	cout << "ind-fas**********\n";
	//Mat _a = r3List2MatRows44(p1);
	//Mat _b = r3List2MatRows44(p2);
	//vector<int> tmp2;
	//float er = flann_knn(_b, _a, tmp2) / (float)(p1.size()*p1.size());
	//cout << "error -> " << er << endl;
	//for(int i = 0; i < tmp2.size(); i++) cout << tmp2[i] << endl;
	//cout << endl;

	cout << "error in: " << error << endl;
	double seconds;
	icp(p1, p2, out, error, seconds, -1.0, -1);
	cout << "error: " << error << endl;
	cout << "seconds: " << seconds << endl;
	/*
	while(true)
	{
		vector<double> dl; vector<int> ind;
		double nd = closestPoints(out, p2, ind, dl);
		cout << "old distance: " << dist << endl;
		cout << "new distance: " << nd << endl;
		system("pause");
		if(nd >= dist) break; //cannot do any better
		dist = nd;
		Mat transform = leastSquaresTransform(out, p2, ind);
		for(int i = 0; i < out.size(); i++) Pixel3DSet::transform_point(transform, out[i]);
		
	}

	*/
	if(PRINT)
	{
		cout << "out:" << endl;
		for(int i = 0; i < out.size(); i++) cout << out[i] << endl;
	}
	if(PRINT)
	{
		cout << "P2:" << endl;
		for(int i = 0; i < p2.size(); i++) cout << p2[i] << endl;
	}

	/*Mat a = r3List2MatRows44(p1);
	Mat b = r3List2MatRows44(p2);

	cout << a << endl;
	cout << b << endl;
*/
	//vector<int> pairs;
	//vector<float> dists;
	//cout << (flann_knn(a, b, pairs, dists) / (double)(a.rows * a.rows)) << endl;

	//for(int i = 0; i < pairs.size(); i++) cout << pairs[i] << endl;
	/*
	Mat destination = a.clone();
	Mat X = b.clone();
	vector<int> pair;
	double lastDist = DBL_MAX;

	Mat lastGood = X.clone();

	while(true) {
		pair.clear(); dists.clear();
		double dist = flann_knn(destination, X, pair, dists);
		cout << pair.size() << " " << p1.size() << endl; 
		//cout << "dist: " << dist << endl;
		if(lastDist <= dist) {
			X = lastGood;
			break;  //converged?
		}
		lastDist = dist;
		X.copyTo(lastGood);
 
		cout << "distance: " << dist << endl;
 
		Mat X_bar(X.size(),X.type());
		for(int i=0;i<X.rows;i++) {
			Point p = destination.at<Point>(pair[i],0);
			X_bar.at<Point>(i,0) = p;
		}
 
		cout << "h a " << endl;
		
 

		X = X.reshape(2);
		cout << "x now " << X << endl;
		X_bar = X_bar.reshape(2);
		cout << X_bar << " is xbar" << endl;
		findBestReansformSVD(X,X_bar);
		break;
		X = X.reshape(1); // back to 1-channel
	}

	cout << destination << endl;
	cout << X << endl;


	/*for (int i = 0; i < p1.size(); i++)
		cout << p1[i] << " -> " << p2[i] << endl;


	vector<int> matches; vector<double> dists;
	double d1= closestPoints(p1, p2, matches, dists);
	cout << "average distance is: " << d1 << endl;
	cout << "cpts:**********\n";
	for(int i = 0; i < matches.size(); i++)
		cout << i << " matched with " << matches[i] << " with distance: " << dists[i] << endl;
	

	Mat m = leastSquaresTransform(p1, p2, matches);


	cout << "computed m: " << m << endl;

	//transform p1:
	cout << "transforming p1 set\n";
	for(int i = 0; i < p1.size(); i++)
	{
		Pixel3DSet::transform_point(m, p1[i]);
	}
	matches.clear(); dists.clear();
	double d2 = closestPoints(p1, p2, matches, dists);
	cout << "average distance is now: " << d2 << endl;
	cout << "aligned cpts:**********\n";
	for(int i = 0; i < matches.size(); i++)
		cout << i << " matched with " << matches[i] << " with distance: " << dists[i] << endl;
	
	
	for (int i = 0; i < p1.size(); i++)
		cout << p1[i] << " -> " << p2[i] << endl;
	*/

	return 0;
}