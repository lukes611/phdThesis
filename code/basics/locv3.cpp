#include "locv3.h"
#include "R3.h"
#include "LTimer.h"
#include <functional>

string ll_type(int type)
{
	string r;
	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);
	switch ( depth ) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}
	r += "C";
	r += (chans+'0');
	return r;
}

int ll_error(string error_name, bool ll_pause, bool ll_exit, int return_value)
{
	cout << "ERROR: ";
	cout << error_name << "\n";
	if(ll_pause)
	{
		system("pause");
	}
	if(ll_exit)
	{
		exit(return_value);
	}
	return return_value;
}

Vec3b ll_color_blend(Vec3b color1, Vec3b color2, double scalar1, double scalar2)
{
	return Vec3b(
		static_cast<unsigned char>(static_cast<double>(color1[0]) * scalar1 + static_cast<double>(color2[0]) * scalar2),
		static_cast<unsigned char>(static_cast<double>(color1[1]) * scalar1 + static_cast<double>(color2[1]) * scalar2),
		static_cast<unsigned char>(static_cast<double>(color1[2]) * scalar1 + static_cast<double>(color2[2]) * scalar2)
	);
}

void ll_sift(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, bool sort, int top)
{
	p1.clear();
	p2.clear();
	cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
	std::vector<KeyPoint> keypoints_1, keypoints_2;    
	f2d->detect( im1, keypoints_1 );
	f2d->detect( im2, keypoints_2 );
	Mat descriptors_1, descriptors_2;    
	f2d->compute( im1, keypoints_1, descriptors_1 );
	f2d->compute( im2, keypoints_2, descriptors_2 );
	BFMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );
	unsigned int matches_size = static_cast<unsigned int>(matches.size());
	if(sort)
	{
		auto f = [](const DMatch & a, const DMatch & b) -> bool { return a.distance < b.distance; };
		std::sort(matches.begin(), matches.end(), f);
		//for(unsigned int i = 0; i < matches_size; i++)
		//	for(unsigned int j = 0; j < matches_size-1; j++)
		//		if(matches[j].distance > matches[j+1].distance)
		//			swap(matches[j], matches[j+1]);
	}
	if(top > -1)
	{
		unsigned int minimum = min(matches_size, static_cast<unsigned int>(top));
		for(unsigned int i = 0; i < minimum; i++)
		{
			p1.push_back(keypoints_1[matches[i].queryIdx].pt);
			p2.push_back(keypoints_2[matches[i].trainIdx].pt);
		}
	}else
	{
		for(unsigned int i = 0; i < matches_size; i++)
		{
			p1.push_back(keypoints_1[matches[i].queryIdx].pt);
			p2.push_back(keypoints_2[matches[i].trainIdx].pt);
		}
	}


}

void ll_surf(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, bool sort, int top)
{
	p1.clear();
	p2.clear();
	cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create();
	std::vector<KeyPoint> keypoints_1, keypoints_2;    
	f2d->detect( im1, keypoints_1 );
	f2d->detect( im2, keypoints_2 );
	Mat descriptors_1, descriptors_2;    
	f2d->compute( im1, keypoints_1, descriptors_1 );
	f2d->compute( im2, keypoints_2, descriptors_2 );
	BFMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );
	unsigned int matches_size = static_cast<unsigned int>(matches.size());
	if(sort)
	{
		auto f = [](const DMatch & a, const DMatch & b) -> bool { return a.distance < b.distance; };
		std::sort(matches.begin(), matches.end(), f);
		//for(unsigned int i = 0; i < matches_size; i++)
		//	for(unsigned int j = 0; j < matches_size-1; j++)
		//		if(matches[j].distance > matches[j+1].distance)
		//			swap(matches[j], matches[j+1]);
	}
	if(top > -1)
	{
		unsigned int minimum = min(matches_size, static_cast<unsigned int>(top));
		for(unsigned int i = 0; i < minimum; i++)
		{
			p1.push_back(keypoints_1[matches[i].queryIdx].pt);
			p2.push_back(keypoints_2[matches[i].trainIdx].pt);
		}
	}else
	{
		for(unsigned int i = 0; i < matches_size; i++)
		{
			p1.push_back(keypoints_1[matches[i].queryIdx].pt);
			p2.push_back(keypoints_2[matches[i].trainIdx].pt);
		}
	}


}

void ll_surf_FastTopX(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, int top)
{
	cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create(100.0, 2);
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	f2d->detect(im1, keypoints_1);
	f2d->detect(im2, keypoints_2);
	Mat descriptors_1, descriptors_2;
	f2d->compute(im1, keypoints_1, descriptors_1);
	f2d->compute(im2, keypoints_2, descriptors_2);
	BFMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match(descriptors_1, descriptors_2, matches);
	unsigned int matches_size = static_cast<unsigned int>(matches.size());

	float * dists = new float[top];
	int tms = matches.size();
	int ms = tms < top ? tms : top;
	p1 = vector<Point>(ms);
	p2 = vector<Point>(ms);
	int mInd = 0;

	for (int i = 0; i < ms; i++) {
		p1[i] = keypoints_1[matches[i].queryIdx].pt;
		p2[i] = keypoints_2[matches[i].trainIdx].pt;
		dists[i] = matches[i].distance;
		if (matches[i].distance > matches[mInd].distance)
			mInd = i;
	}

	for (int i = ms; i < tms; i++)
	{
		if (matches[i].distance < matches[mInd].distance) {
			dists[mInd] = matches[i].distance;
			p1[mInd] = keypoints_1[matches[i].queryIdx].pt;
			p2[mInd] = keypoints_2[matches[i].trainIdx].pt;
			for (int j = 0; j < ms; j++)
			{
				if (matches[j].distance > matches[mInd].distance)
				{
					mInd = j;
				}
			}
		}
	}

}

void ll_sift_FastTopX(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, int top)
{
	cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	f2d->detect(im1, keypoints_1);
	f2d->detect(im2, keypoints_2);
	Mat descriptors_1, descriptors_2;
	f2d->compute(im1, keypoints_1, descriptors_1);
	f2d->compute(im2, keypoints_2, descriptors_2);
	BFMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match(descriptors_1, descriptors_2, matches);
	unsigned int matches_size = static_cast<unsigned int>(matches.size());

	float * dists = new float[top];
	int tms = matches.size();
	int ms = tms < top ? tms : top;
	p1 = vector<Point>(ms);
	p2 = vector<Point>(ms);
	int mInd = 0;

	for (int i = 0; i < ms; i++) {
		p1[i] = keypoints_1[matches[i].queryIdx].pt;
		p2[i] = keypoints_2[matches[i].trainIdx].pt;
		dists[i] = matches[i].distance;
		if (matches[i].distance > matches[mInd].distance)
			mInd = i;
	}

	for (int i = ms; i < tms; i++)
	{
		if (matches[i].distance < matches[mInd].distance) {
			dists[mInd] = matches[i].distance;
			p1[mInd] = keypoints_1[matches[i].queryIdx].pt;
			p2[mInd] = keypoints_2[matches[i].trainIdx].pt;
			for (int j = 0; j < ms; j++)
			{
				if (matches[j].distance > matches[mInd].distance)
				{
					mInd = j;
				}
			}
		}
	}

}

Mat ll_combine_images(Mat & im1, Mat & im2, bool horizontal)
{
	if(horizontal)
	{
		Size s(im1.size().width + im2.size().width, max(im1.size().height, im2.size().height));
		Mat rv = Mat::zeros(s, CV_8UC3);
		for(int y = 0; y < s.height; y++)
		{
			if(y < im1.size().height)
			{
				for(int x = 0; x < im1.size().width; x++)
					rv.at<Vec3b>(Point2i(x,y)) = im1.at<Vec3b>(Point2i(x,y));
			}

			if(y < im2.size().height)
			{
				for(int x = 0; x < im2.size().width; x++)
					rv.at<Vec3b>(Point2i(x+im1.size().width,y)) = im2.at<Vec3b>(Point2i(x,y));
			}
	
	
		}
		return rv.clone();
	}
	Size s(max(im1.size().width, im2.size().width), im1.size().height + im2.size().height + 2);
	Mat rv = Mat::zeros(s, CV_8UC3);

	for(int y = 0; y < im1.size().height; y++)
	{
		for (int x = 0; x < im1.size().width; x++)
				rv.at<Vec3b>(Point2i(x,y)) = im1.at<Vec3b>(Point2i(x,y));
		for(int x = 0; x < im2.size().width; x++)
			rv.at<Vec3b>(Point2i(x,y + im1.size().height)) = im2.at<Vec3b>(Point2i(x,y));
	}

	return rv.clone();
}


void ll_draw_matches(Mat & im, Size & left_offset, vector<Point2i> & p1, vector<Point2i> & p2, int limit)
{
	if(p1.size() != p2.size()) return;
	unsigned int len = 0;
	if(limit == -1)
	{
		len = static_cast<unsigned int>(p1.size());
	}else
	{
		len = min(static_cast<unsigned int>(p1.size()), static_cast<unsigned int>(limit));
	}
	for(unsigned int i = 0; i < len; i++)
	{
		double r = rand() / (double)RAND_MAX; r *= 255.0;
		double g = rand() / (double)RAND_MAX; g *= 255.0;
		double b = rand() / (double)RAND_MAX; b *= 255.0;
		cv::line(im, p1[i], p2[i] + Point2i(left_offset.width,0), Scalar(r,g,b));
	}
}

Mat ll_view_matches(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, int limit)
{
	Mat a, b;
	if(im1.channels() <= 1) cvtColor(im1, a, CV_GRAY2RGB); else a = im1;
	if(im2.channels() <= 1) cvtColor(im2, b, CV_GRAY2RGB); else b = im2;
	Mat im = ll_combine_images(a, b);
	ll_draw_matches(im, im1.size(), p1, p2, limit);
	return im.clone();
}

Mat ll_draw_horizontal_lines(Mat & im1, int number, Scalar color)
{
	double inc = im1.size().height / static_cast<double>(number);
	Mat rv = im1.clone();
	double iter = 0.0;
	int w = static_cast<int>(im1.size().width);
	int tmp = 0;
	for(int i = 0; i < number; i++)
	{
		tmp = static_cast<int>(round(iter));
		cv::line(rv, Point(0, tmp), Point(w, tmp), color);
		iter += inc;
	}
	return rv.clone();
}

Mat ll_draw_vertical_lines(Mat & im1, int number, Scalar color)
{
	double inc = im1.size().width / static_cast<double>(number);
	Mat rv = im1.clone();
	double iter = 0.0;
	int h = static_cast<int>(im1.size().height);
	int tmp = 0;
	for(int i = 0; i < number; i++)
	{
		tmp = static_cast<int>(round(iter));
		cv::line(rv, Point(tmp, 0), Point(tmp, h), color);
		iter += inc;
	}
	return rv.clone();
}

Mat ll_fundamental_matrix(vector<Point2i> & p1, vector<Point2i> & p2)
{
	return findFundamentalMat(p1, p2, CV_FM_RANSAC, 0.01, 0.9999).clone(); //was .01, .99 //then: 0.00001, .999999999
}

bool ll_stereo_rectify(Mat & im1, Mat & im2, int mod, int feature_limit, bool use_surf)
{
	vector<Point2i> points1, points2;
	if(use_surf) ll_surf(im1, im2, points1, points2, true, feature_limit);
	else {
		ll_sift(im1, im2, points1, points2, true, -1);
	}
	Mat fm = ll_fundamental_matrix(points1, points2);
	Mat h1, h2;
	bool complete = stereoRectifyUncalibrated(points1, points2, fm, im1.size(), h1, h2, 5.0);
	if(complete)
	{
		Mat H;
		if(mod == 0)
		{
			invert(h2, h2);
			H = h2 * h1;
			warpPerspective(im1, im1, H, im1.size());
		}else if(mod == 1)
		{
			invert(h1, h1);
			H = h1 * h2;
			warpPerspective(im2, im2, H, im2.size());
		}else
		{
			warpPerspective(im1, im1, h1, im1.size());
			warpPerspective(im2, im2, h2, im2.size());
		}
	}
	return complete;
}


Mat ll_depthbm(Mat & iml, Mat & imr, int ndisparities, int sad_size)
{
	sad_size = (sad_size < 5)?5: sad_size;
	ndisparities = (ndisparities % 16 != 0) ? (ndisparities / 16) * 16 : ndisparities;
	ndisparities = (ndisparities < 16)?16: ndisparities;
	sad_size = (sad_size % 2 == 0)? sad_size + 1 : sad_size;

	Mat left, right;
	cvtColor(iml, left, CV_BGR2GRAY);
	cvtColor(imr, right, CV_BGR2GRAY);
	Mat disp1 = Mat(left.size().height, left.size().width, CV_16SC1);
	Ptr<StereoBM> bmp = StereoBM::create(ndisparities, sad_size);
	bmp->compute(left, right, disp1);
	Point2d minmax;
	minMaxLoc(disp1, &minmax.x, &minmax.y);
	float abs_max = SHRT_MAX;
	disp1.convertTo(disp1, CV_32FC1, 1.0f / (minmax.y-minmax.x));
	//imshow("d1", disp1);
	return disp1.clone();
}

Mat ll_depthsgbm(Mat & iml, Mat & imr, int ndisparities, int sad_size)
{
	//sad_size = (sad_size < 5)?5: sad_size;
	ndisparities = (ndisparities % 16 != 0) ? (ndisparities / 16) * 16 : ndisparities;
	ndisparities = (ndisparities < 16)?16: ndisparities;
	sad_size = (sad_size % 2 == 0)? sad_size + 1 : sad_size;

	Mat left, right;
	cvtColor(iml, left, CV_BGR2GRAY);
	cvtColor(imr, right, CV_BGR2GRAY);
	Mat disp1 = Mat(left.size().height, left.size().width, CV_16SC1);
	
	Ptr<StereoSGBM> bmp = StereoSGBM::create(0, ndisparities, sad_size, 0, 0, 0, 0, 0, 0, 0, 0);
	
	bmp->compute(left, right, disp1);
	
	Point2d minmax;
	minMaxLoc(disp1, &minmax.x, &minmax.y);
	disp1.convertTo(disp1, CV_32FC1, 1.0f / (minmax.y-minmax.x));
	return disp1.clone();
}

double ll_get_angle(double x, double y)
{
	double angle = ll_rad2deg * atan(y / x);
	if (x < 0.0){
		angle += 180.0;
	}
	else if (y < 0.0){
		angle += 360.0;
	}
	return angle;
}

void ll_UCF1_to_32F1(Mat & im)
{
	if(im.channels() != 1)
	{
		cvtColor(im, im, COLOR_BGR2GRAY);
	}
	Mat cp = im.clone();
	im = Mat::zeros(cp.size(), CV_32FC1);
	float * pt = (float * ) im.data;
	for(int i = 0; i < cp.size().width*cp.size().height; i++)
	{
		pt[i] = ((float)cp.data[i])/255.0f;
	}
}
void ll_32F1_to_UCF1(Mat & im)
{
	if(im.channels() != 1)
	{
		cvtColor(im, im, COLOR_BGR2GRAY);
	}
	Mat cp = im.clone();
	im = Mat::zeros(cp.size(), CV_8UC1);
	float * pt = (float * ) cp.data;
	for(int i = 0; i < cp.size().width*cp.size().height; i++)
	{
		im.data[i] = (uchar)((pt[i])*255.0f);
	}
}
void ll_normalize(Mat & inp)
{
	double mn , mx;
	minMaxLoc(inp, &mn, &mx);
	inp -= mn;
	inp /= (mx-mn);
}
void ll_normalize(Mat & inp, Mat & mask, float defaultValue)
{
	double mn , mx;
	minMaxLoc(inp, &mn, &mx, NULL, NULL, mask);
	double diff = mx-mn;
	if(diff == 0.0) return;
	unsigned char * maskP = mask.data;
	float * inpP = (float *) inp.data;
	for(int i = 0; i < inp.rows * inp.cols; i++, inpP++, maskP++)
	{
		if(*maskP == 0x00)
		{
			*inpP = defaultValue;
		}else
		{
			*inpP = (float)(((double)*inpP - mn) / (diff));
		}
	}
}


Mat ll_translation_matrix(double tx, double ty)
{
	double tm[] = {1.0, 0.0, tx, 0.0, 1.0, ty, 0.0, 0.0, 1.0};
	Mat rv(3, 3, CV_64FC1, tm);
	return rv.clone();
}
Mat ll_rotation_matrix(double angle_degrees)
{
	double ang = angle_degrees / ll_rad2deg;
	double cs = cos(ang);
	double sn = sin(ang);
	double tm[] = {cs, -sn, 0.0, sn, cs, 0.0, 0.0, 0.0, 1.0};
	Mat rv(3, 3, CV_64FC1, tm);
	return rv.clone();
}
Mat ll_scale_matrix(double scx, double scy)
{
	double tm[] = {scx, 0.0, 0.0, 0.0, scy, 0.0, 0.0, 0.0, 1.0};
	Mat rv(3, 3, CV_64FC1, tm);
	return rv.clone();
}
Mat ll_scale_center_matrix(double scaleSize, Size imSize)
{
	double hw = imSize.width * 0.5;
	double hh = imSize.height * 0.5;
	Mat translateBack = ll_translation_matrix(hw, hh);
	Mat translateToOrigin = ll_translation_matrix(-hw, -hh);
	Mat scl = ll_scale_matrix(scaleSize, scaleSize);
	return translateBack*scl*translateToOrigin;
}
Mat ll_rotation_center_matrix(double angle_degrees, Size imSize)
{
	double hw = imSize.width * 0.5;
	double hh = imSize.height * 0.5;
	Mat translateBack = ll_translation_matrix(hw, hh);
	Mat translateToOrigin = ll_translation_matrix(-hw, -hh);
	Mat rot = ll_rotation_matrix(angle_degrees);
	return translateBack*rot*translateToOrigin;
}
void ll_transform_image(Mat & inp, Mat & outp, double rotation, double scale, double transX, double transY)
{
	Mat rote = ll_rotation_center_matrix(rotation, inp.size());
	Mat scal = ll_scale_center_matrix(scale, inp.size());
	Mat trans = ll_translation_matrix(transX, transY);
	warpPerspective(inp, outp, trans*scal*rote, inp.size());
}
void ll_transform_image(Mat & inp, Mat & outp, Mat & transform)
{
	warpPerspective(inp, outp, transform, inp.size());
}
Mat ll_transformation_matrix(Size s, double rotation, double scale, double transX, double transY)
	{
		Mat rote = ll_rotation_center_matrix(rotation, s);
		Mat scal = ll_scale_center_matrix(scale, s);
		Mat trans = ll_translation_matrix(transX, transY);
		Mat rv = trans*scal*rote;
		return rv.clone();
	}
void ll_swap_quadrants(Mat & inp)
{
	int cx = inp.size().width / 2;
	int cy = inp.size().height / 2;
	Mat q0(inp, Rect(0, 0, cx, cy));
	Mat q1(inp, Rect(cx, 0, cx, cy));
	Mat q2(inp, Rect(0, cy, cx, cy));
	Mat q3(inp, Rect(cx, cy, cx, cy));
	Mat tmp;
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);

	q1.copyTo(tmp);
	q2.copyTo(q1);
	tmp.copyTo(q2);
}

namespace ll_fft
{

	Mat fft_image(Mat & gsc)
	{
		Mat tmp = gsc.clone();
		ll_UCF1_to_32F1(tmp);
		Mat fourierDomain;
		dft(tmp, fourierDomain, DFT_SCALE | DFT_COMPLEX_OUTPUT);
		return fourierDomain;
	}
	void fft_real_imag(Mat & inp, Mat & outRe, Mat & outIm, bool swap_quads)
	{
		Mat im;
		if(inp.channels() > 1)
		{
			cvtColor(inp, im, COLOR_BGR2GRAY);
		}else
		{
			im = inp.clone();
		}
		Mat fd = fft_image(im);
		Mat re_im[2];
		split(fd, re_im);
		outRe = re_im[0].clone();
		outIm = re_im[1].clone();
		if(swap_quads)
		{
			ll_swap_quadrants(outRe);
			ll_swap_quadrants(outIm);
		}
	}
	Mat fft_reim_cplx(Mat & inp, bool swap_quads)
	{
		Mat im;
		if(inp.channels() > 1)
		{
			cvtColor(inp, im, COLOR_BGR2GRAY);
		}else
		{
			im = inp.clone();
		}
		Mat fd = fft_image(im);
		if(swap_quads)
			ll_swap_quadrants(fd);
		return fd;
	}
	Mat ifft_reim_cplx(Mat & fdIn, bool swap_quads)
	{
		Mat rv;
		Mat fd = fdIn.clone();
		if(swap_quads)
			ll_swap_quadrants(fd);
		dft(fd, rv, DFT_INVERSE | DFT_REAL_OUTPUT);
		ll_normalize(rv);
		return rv;
	}
	void fft_magnitude_phase(Mat & inp, Mat & outMag, Mat & outPha, bool swap_quads)
	{
		Mat im;
		if(inp.channels() > 1)
		{
			cvtColor(inp, im, COLOR_BGR2GRAY);
		}else
		{
			im = inp.clone();
		}
		Mat fd = fft_image(im);
		Mat re_im[2];
		split(fd, re_im);
		cartToPolar(re_im[0], re_im[1], outMag, outPha, true);
		if(swap_quads)
		{
			ll_swap_quadrants(outMag);
			ll_swap_quadrants(outPha);
		}
	}
	void ifft_magnitude_phase(Mat & magI, Mat & phaI, Mat & outp, bool swap_quads)
	{
		Mat re_im[2];
		Mat mag = magI.clone();
		Mat pha = phaI.clone();
		if(swap_quads)
		{
			ll_swap_quadrants(mag);
			ll_swap_quadrants(pha);
		}
		polarToCart(mag, pha, re_im[0], re_im[1], true);
		Mat fd;
		merge(re_im, 2, fd);
		dft(fd, outp, DFT_INVERSE | DFT_REAL_OUTPUT);
		ll_normalize(outp);
	}
	void ifft_real_imag(Mat & re, Mat & im, Mat & outp, bool swap_quads)
	{
		Mat re_im[2];
		re_im[0] = re.clone();
		re_im[1] = im.clone();
		if(swap_quads)
		{
			ll_swap_quadrants(re_im[0]);
			ll_swap_quadrants(re_im[1]);
		}
		Mat fd;
		merge(re_im, 2, fd);
		dft(fd, outp, DFT_INVERSE | DFT_REAL_OUTPUT);
		ll_normalize(outp);
	}

}

void ll_log_polar(Mat & imIn, Mat & outp, double scalarInput)
{
	outp = imIn.clone();
	int w = outp.size().width;
	int h = outp.size().height;
	Point2i center(w/2, h/2);
	IplImage im1 = imIn;
	outp = Mat::zeros(imIn.size(), imIn.type());
	IplImage im2 = outp;
	
	CvPoint2D32f c;
	c.x = (static_cast<float>(center.x), c.y = static_cast<float>(center.y));
	cvLogPolar(&im1, &im2, c, ll_lp_scalar(imIn.size().width, scalarInput), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
}
Mat ll_hamming_window(Size s)
{
	Mat rv = Mat::zeros(s, CV_32FC1);
	unsigned int w = static_cast<unsigned int>(s.width);
	unsigned int h = static_cast<unsigned int>(s.height);
	double cx = s.width * 0.5;
	double cy = s.height * 0.5;
	double a = 0.43836;
	double b = 1.0 - a;
	double N = sqrt(cx*cx + cy*cy) * 2.0;
	double p2 = N - 1.0;
	for(unsigned int y = 0; y < h; y++)
	{
		for(unsigned int x = 0; x < w; x++)
		{
			double xf = static_cast<double>(x);
			double yf = static_cast<double>(y);
			double dx = xf-cx;
			double dy = yf-cy;
			dx *= dx;
			dy *= dy;
			double dist = sqrt(dx + dy);
			double p1 = 2.0 * ll_pi * dist;
			double scalar = a + b * cos(p1 / p2);
			Point pnt(x,y);
			rv.at<float>(pnt) = static_cast<float>(scalar);
		}
	}
	return rv.clone();
}
Mat ll_hanning_window(Size s)
{
	Mat rv;
	createHanningWindow(rv, s, CV_32FC1);
	return rv.clone();
}

void ll_imshow_32FC1(string name, Mat & im)
{
	Mat im2 = im.clone();
	ll_normalize(im2);
	imshow(name, im2);
}
Point2d ll_phase_correlate(Mat & im1, Mat & im2)
{
	return phaseCorrelate(im1, im2);
}


void ll_phase_correlate_rst(Mat & im1i, Mat & im2i, double & rotation, double & scale, double & transX, double & transY, double logTransformScalar, int filter, bool showIms)
{
	Mat im1 = im1i.clone();
	Mat im2 = im2i.clone();
	
	ll_UCF1_to_32F1(im1);
	ll_UCF1_to_32F1(im2);
	
	

	//initial filtering
	Mat ham;
	switch(filter)
	{
	case 1: ham = ll_hanning_window(im1i.size()); break;
	case 2: ham = ll_hamming_window(im1i.size()); break;
	}
	if(filter != 0)
	{
		im1 = im1.mul(ham);
		im2 = im2.mul(ham);
	}

	ll_normalize(im1);
	ll_normalize(im2);
	
	ll_32F1_to_UCF1(im1);
	ll_32F1_to_UCF1(im2);
	
	Mat m1, m2, p1, p2;
		
	ll_fft::fft_magnitude_phase(im1, m1, p1);
	ll_fft::fft_magnitude_phase(im2, m2, p2);
	m1 /= static_cast<float>(m1.size().width * m1.size().height);
	m2 /= static_cast<float>(m1.size().width * m1.size().height);
	
	log(m1, m1);
	log(m2, m2);
	
	Laplacian(m1, m1, m1.depth(), 1);
	Laplacian(m2, m2, m1.depth(), 1);
	
	ll_log_polar(m2.clone(), m2, logTransformScalar);
	ll_log_polar(m1.clone(), m1, logTransformScalar);
	
	if(showIms)
	{
		ll_imshow_32FC1("log_polar", m2);
		ll_imshow_32FC1("log_polar2", m1);
	}
	
	Point2d rs = phaseCorrelate(m1, m2);
		
	int hw = static_cast<int>(m1.size().width);
	int hh = static_cast<int>(m1.size().height);
		
	rotation = rs.y;
	rotation /= 512.0;
	rotation *= 360.0;
		
	scale = rs.x;
	scale /= ll_lp_scalar(m1.size().width, logTransformScalar);
	scale = exp(scale);
	scale = 1.0 / scale;
		

	m1 = im1i.clone();
	m2 = im1i.clone();
	ll_transform_image(m1, m1, rotation, scale, 0.0, 0.0);
	ll_transform_image(m2, m2, rotation+180.0, scale, 0.0, 0.0);
	Mat im2Cp = im2i.clone();
	ll_UCF1_to_32F1(im2Cp);
	ll_UCF1_to_32F1(m1);
	ll_UCF1_to_32F1(m2);
		
	Point2d trv = phaseCorrelate(m1, im2Cp);
	Point2d trv2 = phaseCorrelate(m2, im2Cp);
		
	ll_transform_image(im1i, m1, rotation, scale, trv.x, trv.y);
	ll_transform_image(im1i, m2, rotation+180.0, scale, trv.x, trv.y);
	double diff1 = ll_mse<unsigned char>(m1, im2i);
	double diff2 = ll_mse<unsigned char>(m2, im2i);
		
		
	if(diff2 < diff1)
	{
		transX = trv2.x;
		transY = trv2.y;
		rotation += 180.0;
		return;
	}
	transX = trv.x;
	transY = trv.y;
}

void ll_phase_correlate_rs(Mat & im1i, Mat & im2i, double & rotation, double & scale, double logTransformScalar, int filter)
{
	Mat im1 = im1i.clone();
	Mat im2 = im2i.clone();
	
	ll_UCF1_to_32F1(im1);
	ll_UCF1_to_32F1(im2);
	
	ll_normalize(im1);
	ll_normalize(im2);

	//initial filtering
	Mat ham;
	switch(filter)
	{
	case 1: ham = ll_hanning_window(im1i.size()); break;
	case 2: ham = ll_hamming_window(im1i.size()); break;
	}
	
	Mat m2, m1;
	//log polar takes float mat
	ll_log_polar(im2, m2, logTransformScalar);
	ll_log_polar(im1, m1, logTransformScalar);
	

	if(filter != 0)
	{
		m1 = m1.mul(ham);
		m2 = m2.mul(ham);
	}

	Point2d rs = phaseCorrelate(m1, m2);
		
	int hw = static_cast<int>(m1.size().width);
	int hh = static_cast<int>(m1.size().height);
		
	rotation = rs.y;
	rotation /= 512.0;
	rotation *= 360.0;
		
	scale = rs.x;
	scale /= ll_lp_scalar(m1.size().width, logTransformScalar);
	scale = exp(scale);
	scale = scale;
		

	ll_transform_image(im1, m1, rotation, scale, 0.0, 0.0);
	ll_transform_image(im1, m2, rotation+180.0, scale, 0.0, 0.0);


	ll_32F1_to_UCF1(m1);
	ll_32F1_to_UCF1(m2);
	//m1 and m2 are the possible sollutions
	double diff1 = ll_mse<unsigned char>(m1, im2i);
	double diff2 = ll_mse<unsigned char>(m2, im2i);
	
	if(diff2 < diff1) rotation += 180.0;
		
}

double ll_lp_scalar(int w, double possibleScale)
{
	return ((double)w) / log(((double)w) / possibleScale);
}

double ll_lp_scalar(int nw, int w, double possibleScale)
{
	return ((double)nw) / log(((double)w) / possibleScale);
}


void ll_phase_correlate_rst(Mat & im1, Mat im2)
{
	double r, s, x, y;
	ll_phase_correlate_rst(im1, im2, r, s, x, y);
	ll_transform_image(im1, im1, r, s, x, y);
}

bool ll_phase_correlate_rst_rectify(Mat & im1, Mat im2)
{
	double r, s, x, y;
	ll_phase_correlate_rst(im1, im2, r, s, x, y);
	ll_transform_image(im1, im1, r, s, 0.0, y);
	if (x >= 0.0) return true;
	return false;
}

void ll_find_image_intersection(Point2d p, Point2d v, Size image_size, Point2d & p1, Point2d & p2)
{
	vector<double> s(4);
	if(abs(v.x) < 0.0000001)
	{
		s[0] = image_size.width * 2.0;
		s[1] = image_size.width * -2.0;
	}else
	{
		s[0] = -p.x / v.x;
		s[1] = (image_size.width - p.x) / v.x;
	}
	if(abs(v.y) < 0.0000001)
	{
		s[2] = image_size.height * 2.0;
		s[3] = image_size.height * -2.0;
	}else
	{
		s[2] = -p.y / v.y;
		s[3] = (image_size.height - p.y) / v.y;
	}
	sort(s.begin(), s.end());
	p1 = p + v * s[1];
	p2 = p + v * s[3];
}


Mat ll_depth2dbm(Mat & iml, Mat & imr, int ndisparities, int sad_size)
{
	Mat rv = Mat::zeros(iml.size(), CV_32FC1);
	if(iml.size() != imr.size()) return rv.clone();
	unsigned int w = static_cast<unsigned int>(iml.size().width);
	unsigned int h = static_cast<unsigned int>(iml.size().height);
	int offset = ndisparities+sad_size;
	double maximum_distance = ll_distance<int>(0, 0, ndisparities, ndisparities);
	for(unsigned int y = offset; y < h-offset; y++)
	{
		for(unsigned int x = offset; x < w-offset; x++)
		{
			//loop for ndisparities
			double best_error = DBL_MAX;
			Point2d p;
			for(unsigned int y_ = y; y_ <= y + ndisparities; y_++)
			{
				for(unsigned int x_ = x; x_ <= x + ndisparities; x_++)
				{
					//loop for sad size
					double error = 0.0;
					for(unsigned int y__ = y_ - sad_size, y__2 = y - sad_size; y__ <= y_ + sad_size; y__++, y__2++)
					{
						for(unsigned int x__ = x_ - sad_size, x__2 = x - sad_size; x__ <= x_ + sad_size; x__++, x__2++)
						{
							double tmp = static_cast<double>(iml.at<float>(y__2, x__2)) - static_cast<double>(imr.at<float>(y__, x__));
							error += tmp * tmp;
						}
					}
					if(error < best_error)
					{
						best_error = error;
						p = Point2d(static_cast<double>(x_), static_cast<double>(y_));
					}
				}
			}
			Point2d p2(static_cast<double>(x), static_cast<double>(y));
			double distance = ll_distance<double>(p.x, p.y, p2.x, p2.y);
			rv.at<float>(y,x) = static_cast<float>( distance / maximum_distance );
		}
	}
	return rv.clone();
}


Mat ll_depth2dbm_both_sides(Mat & iml, Mat & imr, int ndisparities, int sad_size)
{
	//create return value
	Mat rv = Mat::zeros(iml.size(), CV_32FC1);
	cout << iml.size() << " " << imr.size() << endl;
	if(iml.size() != imr.size()) return rv.clone(); //check left and right images have the same size
	int w = (int)iml.size().width;
	int h = (int)iml.size().height;
	int offset = ndisparities+sad_size;
	float maximum_distance = (float)ll_distance<float>(0.0f, 0.0f, (float)ndisparities, (float)ndisparities);
	//X,Y is the pixel to compute the depth for
	for(int Y = offset; Y < h-offset; Y++)
	{
		for(int X = offset; X < w-offset; X++)
		{
			//loop for ndisparities
			double best_error = DBL_MAX;
			Point2f p;
			//Y-Search-Area  = YSA
			//X-Search-Area  = XSA
			for(int YSA = -ndisparities; YSA <= ndisparities; YSA++)
			{
				for(int XSA = -ndisparities; XSA <= ndisparities; XSA++)
				{
					//loop for sad size
					double error = 0.0;
					//YBIndex, XBIndex is the block index of block
					for(int YBIndex = -sad_size; YBIndex <= sad_size; YBIndex++)
					{
						for(int XBIndex = -sad_size; XBIndex <= sad_size; XBIndex++)
						{
							double tmp = static_cast<double>(iml.at<unsigned char>(Y + YBIndex, X + XBIndex)) - static_cast<double>(imr.at<unsigned char>(Y + YSA + YBIndex, X + XSA + XBIndex));
							error += tmp * tmp;
						}
					}
					if(error < best_error)
					{
						best_error = error;
						p = Point2f((float) (X + XSA), (float) (Y + YSA));
					}
				}
			}
			Point2f p2((float) X, (float) Y);
			double distance = ll_distance<double>(p.x, p.y, p2.x, p2.y);
			rv.at<float>(Y,X) = (float)( distance / maximum_distance );
		}
	}
	return rv.clone();
}

Mat ll_getColoredDepthMap(Mat & inputDepth)
{
	Mat tmp;
	if(tmp.type() == CV_32FC1)
	{
		tmp = inputDepth.clone();
		ll_32F1_to_UCF1(tmp);
	}else if(tmp.type() == CV_8UC1)
	{
		tmp = inputDepth;
	}else
	{
		return tmp.clone();
	}
	Mat _a;
	double minimum, maximum;

	minMaxIdx(tmp, &minimum, &maximum);
	double scale = 255.0 / (maximum-minimum);
	tmp.convertTo(_a, CV_8UC1, scale, -minimum * scale);
	applyColorMap(_a, tmp, COLORMAP_JET);
	return tmp.clone();

}

void ll_pad2(Mat & input, Mat & output)
{
	int m = max(input.size().width, input.size().height);
	int sizes[] = {2,4,8,16,32,64,128,256,512,1024,2048, 4096, 8192,16384};
	int index = 0;
	for (; index < 14; index++) if (m > sizes[index]) continue; else break;
	int newSize = sizes[index];
	Mat padded;
	int difx = newSize - input.size().width;
	int dify = newSize - input.size().height;
	padded.create(newSize, newSize, input.type());
	padded.setTo(cv::Scalar::all(0));
	input.copyTo(padded(Rect(difx/2, dify/2, input.cols, input.rows)));

	output = padded.clone();
}

Mat ll_edgeDetection(Mat & im)
{
	Mat rv = Mat::zeros(im.size(), CV_32FC1);
	Size s = im.size();
	int w = s.width-1;
	int h = s.height-1;
	for(int y = 0; y < h; y++)
	{
		for(int x = 0; x < w; x++)
		{
			float a = im.at<float>(y,x);
			float b = im.at<float>(y,x+1);
			float c = im.at<float>(y+1, x);
			b -= a;
			c -= a;
			b*=b;
			c*=c;
			rv.at<float>(y,x) = sqrt(b+c);
		}
	}
	return rv.clone();
}

vector<string> ll_split(string & s, char delim)
{
	vector<string> ret;
	string rem = "";
	int size = s.size();
	for (int i = 0; i < size; i++)
	{
		if (s[i] == delim)
		{
			if (rem.size() > 0) ret.push_back(rem);
			rem = "";
		}else
			rem += s[i];
	}
	if (rem.size() > 0) ret.push_back(rem);
	return ret;
}

Mat ll_surfRegister(Mat & a, Mat & b, bool sort, int getTopXFeatures, double allowedError)
{
	vector<Point> pnts1, pnts2;
	Mat c;
	ll_surf(a, b, pnts1, pnts2, sort, getTopXFeatures);
	Mat hm = findHomography(pnts1, pnts2, CV_RANSAC, allowedError);
	return hm.clone();
}

Mat ll_siftRegister(Mat & a, Mat & b, bool sort, int getTopXFeatures, double allowedError)
{
	vector<Point> pnts1, pnts2;
	Mat c;
	ll_sift(a, b, pnts1, pnts2, sort, getTopXFeatures);
	Mat hm = findHomography(pnts1, pnts2, CV_RANSAC, allowedError);
	return hm.clone();
}

double SuperFast2DPC::FastPC(Mat & im1, Mat & im2, Point2d & translation)
{
	assert(im1.size() == im2.size());

	Mat hwindow = ll_hanning_window(im1.size());


	Mat a = im1.clone(), b = im2.clone();
	ll_UCF1_to_32F1(a); ll_UCF1_to_32F1(b); //copy im1 and im2 into a and b

	a = a.mul(hwindow);
	b = b.mul(hwindow);
	float sf1 = 0.0f, sf2 = 0.0f;
	int hw = im1.size().width / 2;
	int hh = im1.size().height / 2;


	LTimer time;
	time.start(); //start the timer

	Mat Yavg = Mat::zeros(Size(a.rows, 1), CV_32FC1); //create signal buffers
	Mat Xavg = Mat::zeros(Size(a.cols, 1), CV_32FC1);
	Mat Yavg2 = Mat::zeros(Size(a.rows, 1), CV_32FC1);
	Mat Xavg2 = Mat::zeros(Size(a.cols, 1), CV_32FC1);

	for (int y = 0; y < a.rows; y++)
	{
		for (int x = 0; x < a.cols; x++) //for each pixel, summation into correct signal bucket
		{
			float value = a.at<float>(y, x);
			float value2 = b.at<float>(y, x);
			if (value > 0.0f)
			{
				Yavg.at<float>(0, y) += value;
				Xavg.at<float>(0, x) += value;
			}
			if (value2 > 0.0f)
			{
				Yavg2.at<float>(0, y) += value2;
				Xavg2.at<float>(0, x) += value2;
			}
		}
	}

	/*ll_normalize(Xavg);
	ll_normalize(Xavg2);
	ll_normalize(Yavg);
	ll_normalize(Yavg2);*/

	translation.x = SuperFast2DPC::PC1D(Xavg, Xavg2);
	translation.y = SuperFast2DPC::PC1D(Yavg, Yavg2);

	time.stop(); //end the timer
	return time.getSeconds();
}

double SuperFast2DPC::lukeDimentionReduce1D(Mat & im, Mat & ret, float thresh, float minRad)
{
	Mat a = im.clone();
	ll_UCF1_to_32F1(a);
	LTimer timer;
	timer.start();

	Size _size = a.size();
	int w = _size.width, h = _size.height;
	float hw = 0.5f * w, hh = 0.5f * h;
	float scalar_1 = w / (float) 360.0f;
	float hwf = hw, hhf = hh;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			float v = a.at<float>(y, x);
			if (v < thresh) continue;
			float xf = x, yf = y;
			xf -= hwf; yf -= hhf;
			float mag = sqrt(xf * xf + yf * yf);
			if (mag < minRad) continue;
			float X = ll_R3::R3::getAngle(xf / mag, yf / mag) * scalar_1;
			int Xi = (int)X;
			if (Xi >= 0 && Xi < w)
				ret.at<float>(0, Xi) += v;

		}
	timer.stop();
	ll_normalize(ret);
	return timer.getSeconds();
}

double SuperFast2DPC::FastPCR(Mat & a, Mat & b, double & rotation, float thresh, float minRad)
{
	assert(a1.size() == b1.size());
	LTimer timer;
	timer.start();

	Size _size = a.size();
	int w = _size.width, h = _size.height;
	float hw = 0.5f * w, hh = 0.5f * h;
	float scalar_1 = w / (float) 360.0f;
	Mat signal1 = Mat::zeros(Size(w, 1), CV_32FC1);
	Mat signal2 = Mat::zeros(Size(w, 1), CV_32FC1);
	float hwf = hw, hhf = hh;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			float v = a.at<float>(y, x);
			float v2 = b.at<float>(y, x);
			if (v < thresh && v2 < thresh) continue;
			float xf = x, yf = y;
			xf -= hwf; yf -= hhf;
			float mag = sqrt(xf * xf + yf * yf);
			if (mag < minRad) continue;
			float X = ll_R3::R3::getAngle(xf / mag, yf / mag) * scalar_1;
			int Xi = (int)X;
			if (Xi >= 0 && Xi < w)
			{
				if (v > thresh) signal1.at<float>(0, Xi) += v;
				if (v2 > thresh) signal2.at<float>(0, Xi) += v2;
			}

		}
	timer.stop();
	ll_normalize(signal1);
	ll_normalize(signal2);

	double scalar_2 = 360.0 / a.size().width;
	rotation = SuperFast2DPC::PC1D(signal1, signal2);
	rotation *= scalar_2;


	return timer.getSeconds();
}

double SuperFast2DPC::phase_correlate_rt(Mat & a, Mat & b, double & rotation, Point2d & translation)
{
	Mat m1, m2, p1, p2, ac;
	LTimer timer;
	timer.start();

	ll_fft::fft_magnitude_phase(a, m1, p1);
	ll_fft::fft_magnitude_phase(b, m2, p2);
	ll_center_pixel<float>(m1) = 0.0f;
	ll_center_pixel<float>(m2) = 0.0f;
	m1 /= static_cast<float>(m1.size().width * m1.size().height);
	m2 /= static_cast<float>(m1.size().width * m1.size().height);

	

	

	m1 *= 100000000.0f;
	m2 *= 100000000.0f;

	log(m1, m1);
	log(m2, m2);

	//Laplacian(m1, m1, m1.depth(), 1);
	//Laplacian(m2, m2, m2.depth(), 1);

	timer.stop();
	double ms = timer.getSeconds();

	ms += FastPCR(m1, m2, rotation);

	timer.reset();
	timer.start();

	ll_transform_image(a, ac, rotation, 1.0, 0.0, 0.0);

	timer.stop();
	ms += timer.getSeconds();

	ms += FastPC(ac, b, translation);


	return ms;
}

double SuperFast2DPC::phase_correlate_rt2(Mat & a, Mat & b, double & rotation, Point2d & translation)
{
	Mat m1, m2, p1, p2, ac;
	LTimer timer;
	timer.start();

	ll_fft::fft_magnitude_phase(a, m1, p1);
	ll_fft::fft_magnitude_phase(b, m2, p2);
	ll_center_pixel<float>(m1) = 0.0f;
	ll_center_pixel<float>(m2) = 0.0f;
	m1 /= static_cast<float>(m1.size().width * m1.size().height);
	m2 /= static_cast<float>(m1.size().width * m1.size().height);

	

	
	m1 *= 100000000.0f;
	m2 *= 100000000.0f;

	log(m1, m1);
	log(m2, m2);

	/*Laplacian(m1, m1, m1.depth(), 1);
	Laplacian(m2, m2, m2.depth(), 1);*/


	timer.stop();
	double ms = timer.getSeconds();

	ms += FastPCR(m1, m2, rotation);
	rotation -= 180.0f;

	timer.reset();
	timer.start();

	ll_transform_image(a, ac, rotation, 1.0, 0.0, 0.0);

	timer.stop();
	ms += timer.getSeconds();

	ms += FastPC(ac, b, translation);


	return ms;
}

double SuperFast2DPC::phase_correlate_rtOptimal(Mat & a, Mat & b, double & rotation, Point2d & translation)
{
	double r1, r2; Point2d t1, t2;

	double ms1 = phase_correlate_rt(a.clone(), b.clone(), r1, t1);
	double ms2 = phase_correlate_rt2(a.clone(), b.clone(), r2, t2);
	Mat a1, a2;
	ll_transform_image(a, a1, r1, 1.0, t1.x, t1.y);
	ll_transform_image(a, a2, r2, 1.0, t2.x, t2.y);

	double e1 = ll_mse<unsigned char>(a1, b);
	double e2 = ll_mse<unsigned char>(a2, b);

	if (e2 < e1)
	{
		rotation = r2;
		translation = t2;
		return ms2;
	}
	rotation = r1;
	translation = t1;
	return ms1;
}

double SuperFast2DPC::phase_correlate_rtOptimal_ed(Mat & a, Mat & b, double & rotation, Point2d & translation)
{
	double r1, r2; Point2d t1, t2;

	Mat _a = a.clone(), _b = b.clone();
	ll_UCF1_to_32F1(_a); ll_UCF1_to_32F1(_b);
	_a = ll_edgeDetection(_a); _b = ll_edgeDetection(_b);
	ll_normalize(_a); ll_normalize(_b);
	ll_32F1_to_UCF1(_a); ll_32F1_to_UCF1(_b);


	double ms1 = phase_correlate_rt(_a.clone(), _b.clone(), r1, t1);
	double ms2 = phase_correlate_rt2(_a.clone(), _b.clone(), r2, t2);
	Mat a1, a2;
	ll_transform_image(a, a1, r1, 1.0, t1.x, t1.y);
	ll_transform_image(a, a2, r2, 1.0, t2.x, t2.y);

	double e1 = ll_mse<unsigned char>(a1, b);
	double e2 = ll_mse<unsigned char>(a2, b);

	if (e2 < e1)
	{
		rotation = r2;
		translation = t2;
		return ms2;
	}
	rotation = r1;
	translation = t1;
	return ms1;
}


double SuperFast2DPC::PC1D(Mat & n, Mat & n2)
{
	double ret = phaseCorrelate(n, n2).x;
	return ret;
}

void ll_pad(Mat & input, Mat & output, Size s)
{
	if(input.size() == s)
	{
		output = input;
		return;
	}
	Mat padded;
	int difx = s.width - input.size().width;
	int dify = s.height - input.size().height;
	padded.create(s, input.type());
	padded.setTo(cv::Scalar::all(0));
	input.copyTo(padded(Rect(difx/2, dify/2, input.cols, input.rows)));
	output = padded.clone();
}

Mat least_squares(Mat M, Mat y)
{
	Mat mt = M.t();
	Mat pre = mt * M;
	pre = pre.inv();
	return pre * mt * y;
}