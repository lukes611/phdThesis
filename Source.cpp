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
	double divisor = src.size() > 0 ? (double)(src.size()) : 1.0;
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
	return (float)(rand() % 384);
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

Mat icp(vector<R3> & src, vector<R3> & dst, vector<R3> & out, 
		double & error_out, double & time, int & iterations, 
		double minError = -1.0, int maxIterations = -1)
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
		for(int i = 0; i < out.size(); i++) Pixel3DSet::transform_point(T, out[i]);
		iterations++;
	}
	clock.stop();
	time = clock.getSeconds();
	return ret.clone();
}


Mat icp_outlierRemoval(vector<R3> & src, vector<R3> & _dst, vector<R3> & out, 
					   double & error_out, double & time, int & iterations, 
					   double outlierRemovalTimesMean = 7.0, double minError = -1.0, 
					   int maxIterations = -1)
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
		double current_distance = closestPointsf(out, dst, ind, dl);
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

		T = leastSquaresTransform(srcTest, dstTest, indTest);
		for(int i = 0; i < out.size(); i++) Pixel3DSet::transform_point(T, out[i]);
		iterations++;
	}
	clock.stop();
	time = clock.getSeconds();
	return ret.clone();
}

//add time measurement and then speedup
//add outlier removal

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
	
	SIObj ob; ob.open_obj("C:/lcppdata/obj/bunny_simplified2.obj");

	Pixel3DSet p1(ob._points);

	p1.normalize(256);

	Pixel3DSet p2 = p1;

	p2.transform_set(0.0f, 3.0f, 1.0f, 1.0f, 2.0f, 0.0f, 0.0f, R3(1.0f, 1.0f, 1.0f) * 0.5f * 256.0f);

	cout << "opened object bunny with : " << ob._points.size() << " no. points\n";

	double error, seconds;
	vector<R3> out;
	int iterations;
	vector<int> ii; vector<double> dd;
	error = closestPointsf(p1.points, p2.points, ii, dd);
	printf("error-in: %lf\n", error);
	icp(p1.points, p2.points, out, error, seconds, iterations);
	printf("error-out: %lf\n", error);

	/*vector<R3> p1, p2;
	int imSize = 256;

	Mat km = VMat::transformation_matrix(384, 2.0f, 1.0f, 0.0f, 1.0f, 1.0f, 2.0f, 1.0f);

	

	for (int i = 0; i < imSize * imSize; i++)
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

	

	cout << "ind-fas**********\n";
	

	cout << "error in: " << error << endl;
	double seconds; int iters;
	icp_outlierRemoval(p1, p2, out, error, seconds, iters);
	cout << "error: " << error << endl;
	cout << "number of iterations: " << iters << endl;
	cout << "seconds: " << seconds << endl;
	
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

	
	*/

	return 0;
}