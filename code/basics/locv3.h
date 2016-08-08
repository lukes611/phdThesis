/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description: 
		Contains some objects and algorithms which are wrappers for opencv3

	depends on: opencv 3.0
*/
#pragma once



#include <opencv2\core\core.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\video\video.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2\core\cuda.hpp>
#include <opencv2\highgui.hpp>

using namespace cv;
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <stdint.h>
#include <vector>

#define LL_DEFAULT_LOG_TRANSFORM_SCALAR 3.56

using namespace std;
const double ll_rad2deg = 57.295779513082320876798154814105;
const double ll_pi = 3.14159265358979323846;
double ll_lp_scalar(int w, double possibleScale = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
double ll_lp_scalar(int nw, int w, double possibleScale = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
namespace std
{
	template <class T>
	T round(T input)
	{
		return floor(input+(T)0.5);
	}
}

string ll_type(int type); //gets the type as a string from opencv::Mat.type()

int ll_error(string error_name = "an ll_error occurred", bool ll_pause = true, bool ll_exit = true, int return_value = -1); //signal an error

//compute the 2d distance between two points
template <class T>
double ll_distance(T x1, T y1, T x2, T y2)
{
	return sqrt(static_cast<double>((x1-x2)*(x1-x2)) + static_cast<double>((y1-y2)*(y1-y2)));
}

Vec3b ll_color_blend(Vec3b color1, Vec3b color2, double scalar1, double scalar2);// interpolates from color1 to color2 using scalars scalar1 and scalar2
//return value = color1 * scalar1 + color2 * scalar2

void ll_sift(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, bool sort = false, int top = -1);//performs sift
//feature matching on two images

void ll_surf(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, bool sort = false, int top = -1);//performs sift
//feature matching on two images


void ll_surf_FastTopX(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, int top);

void ll_sift_FastTopX(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, int top);
//performs surf
//feature matching on two images

Mat ll_combine_images(Mat & im1, Mat & im2, bool horizontal = true); /*creates a new image, which is im1 and im2 next to eachother*/

void ll_draw_matches(Mat & im, Size & left_offset, vector<Point2i> & p1, vector<Point2i> & p2, int limit = -1);// draws matches on 
//a combined image

Mat ll_view_matches(Mat & im1, Mat & im2, vector<Point2i> & p1, vector<Point2i> & p2, int limit = -1); //view feature matches

Mat ll_draw_horizontal_lines(Mat & im1, int number, Scalar color = Scalar(255,255,255)); //draws number of horizontal lines

Mat ll_draw_vertical_lines(Mat & im1, int number, Scalar color = Scalar(255,255,255)); //draws number of vertical lines

Mat ll_fundamental_matrix(vector<Point2i> & p1, vector<Point2i> & p2); //basic version to find the fundamental matrix

bool ll_stereo_rectify(Mat & im1, Mat & im2, int mod = 1, int feature_limit = 100, bool use_surf = false); //0-mods im1, 1 mods im2, 2 mods both
//set feature limit to -1 to include all features

//some stereo disparity mapping functions
Mat ll_depthbm(Mat & iml, Mat & imr, int ndisparities = 80, int sad_size = 21); //block matching depth mapping
Mat ll_depthsgbm(Mat & iml, Mat & imr, int ndisparities = 80, int sad_size = 21); //semi-global block matching depth mapping
Mat ll_depth2dbm(Mat & iml, Mat & imr, int ndisparities = 80, int sad_size = 21); //2d block matching disparity image generation
Mat ll_depth2dbm_both_sides(Mat & iml, Mat & imr, int ndisparities = 80, int sad_size = 21); //2d block matching disparity image generation (search both sides of window)

Mat ll_getColoredDepthMap(Mat & inputDepth);

double ll_get_angle(double x, double y); //compute the angle given a 2d vector

void ll_UCF1_to_32F1(Mat & im); //convert an image from unsigned char single channel to float single channel
void ll_32F1_to_UCF1(Mat & im); //convert an image from float single channel to unsiged char single channel
void ll_normalize(Mat & inp); //normalize an image so pixels are within [0,1]
void ll_normalize(Mat & inp, Mat & mask, float defaultValue = 0.0f); //normalize an image so pixels are within [0,1], set all pixels which 
//register as 0 in the mask to the defaultValue

Mat ll_scale_matrix(double scx, double scy); //generate 2d scale matrix
Mat ll_rotation_matrix(double angle_degrees); //generate a 2d rotation matrix
Mat ll_translation_matrix(double tx, double ty); //generate a 2d translation matrix
Mat ll_scale_center_matrix(double scaleSize, Size imSize); //generate a 2d scale matrix which performs 
Mat ll_transformation_matrix(Size s, double rotation, double scale, double transX, double transY);
void ll_transform_image(Mat & inp, Mat & outp, double rotation, double scale, double transX, double transY); //transforms the image by some rst values
void ll_transform_image(Mat & inp, Mat & outp, Mat & transform); //transforms the image by some transformation matrix
void ll_swap_quadrants(Mat & inp); //swaps the quadrants (used for fft visualization and processing
void ll_log_polar_v2(Mat & img, Mat & out); //a version of ruben's log polar transform


//for performing ffts
namespace ll_fft
{
	Mat fft_image(Mat & gsc);
	void fft_real_imag(Mat & inp, Mat & outRe, Mat & outIm, bool swap_quads = true);
	Mat fft_reim_cplx(Mat & inp, bool swap_quads = false);
	Mat ifft_reim_cplx(Mat & fdIn, bool swap_quads = false);
	void fft_magnitude_phase(Mat & inp, Mat & outMag, Mat & outPha, bool swap_quads = true);
	void ifft_magnitude_phase(Mat & magI, Mat & phaI, Mat & outp, bool swap_quads = true);
	void ifft_real_imag(Mat & re, Mat & im, Mat & outp, bool swap_quads = true);
}

template <class T>
T ll_lirp_at(Mat & im, T xf, T yf)
{
	int width = im.size().width;
	int height = im.size().height;
	if(xf < 0.0 || yf < 0.0 || yf >= (T) height || xf >= (T)width)
	{
		return 0.0;
	}

	T p1, p2, p3, p4;
	int x,y;
	x = (int)floor(xf);
	y = (int)floor(yf);
	p1 = im.at<T>(Point(x,y));
	p2 = im.at<T>(Point(x+1,y));
	p3 = im.at<T>(Point(x,y+1));
	p4 = im.at<T>(Point(x+1,y+1));
	
	T xb = (T)x;
	T xa = (T)(x+1);
	T scaler1 = (xf - xb);
	T pint1 = scaler1*p2 + (1.0-scaler1)*p1;

	T pint2 = scaler1*p4 + (1.0-scaler1)*p3;

	T yb = (T)y;
	T ya = (T)(y+1);
	T scaler2 = (yf - yb);
	T rv = scaler2*pint2 + (1.0f-scaler2)*pint1;
	return rv;
}

void ll_log_polar(Mat & imIn, Mat & outp, double scalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);

template <class PixelType>
double ll_mse(Mat & im1, Mat & im2)
	{
		if(im1.size() != im2.size()) -1.0;
		double rv = 0.0, tmp = 0.0;
		Point2i p(0,0);
		unsigned int w = static_cast<unsigned int>(im1.size().width);
		unsigned int h = static_cast<unsigned int>(im1.size().height);
		for(unsigned int y = 0; y < h; y++)
		{
			p.y = y;
			for(unsigned int x = 0; x < w; x++)
			{
				p.x = x;
				tmp = static_cast<double>(im1.at<PixelType>(p)) - static_cast<double>(im2.at<PixelType>(p));
				rv += tmp*tmp;
			}
		}
		rv /= static_cast<double>(w*h);
		return rv;
	}

template <class T>
T & ll_center_pixel(Mat & im)
{
	Point2i p(static_cast<int>(im.size().width)/2, static_cast<int>(im.size().height)/2);
	return im.at<T>(p);
}

Mat ll_hamming_window(Size s);
Mat ll_hanning_window(Size s);
void ll_imshow_32FC1(string name, Mat & im);

Point2d ll_phase_correlate(Mat & im1, Mat & im2);
void ll_phase_correlate_rst(Mat & im1i, Mat & im2i, double & rotation, double & scale, double & transX, double & transY, double logTransformScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR, int filter = 1, bool showIms = false);
void ll_phase_correlate_rs(Mat & im1i, Mat & im2i, double & rotation, double & scale, double logTransformScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR, int filter = 1);
void ll_phase_correlate_rst(Mat & im1, Mat im2);
bool ll_phase_correlate_rst_rectify(Mat & im1, Mat im2); //returns whether images should be flipped!

void ll_pad2(Mat & input, Mat & output);

void ll_find_image_intersection(Point2d p, Point2d v, Size image_size, Point2d & p1, Point2d & p2);

Mat ll_edgeDetection(Mat & im);

vector<string> ll_split(string & s, char delim = ' ');

Mat ll_surfRegister(Mat & im1, Mat & im2, bool sort = true, int getTopXFeatures = 50, double allowedError = 0.1);

Mat ll_siftRegister(Mat & im1, Mat & im2, bool sort = true, int getTopXFeatures = 50, double allowedError = 0.1);

class SuperFast2DPC
{
public:
	static double FastPC(Mat & im1, Mat & im2, Point2d & translation);

	static double lukeDimentionReduce1D(Mat & im, Mat & ret, float thresh = 0.1f, float minRad = 10.0f);

	static double FastPCR(Mat & a, Mat & b, double & rotation, float thresh = 0.01f, float minRad = 10.0f);

	static double phase_correlate_rt(Mat & a, Mat & b, double & rotation, Point2d & translation);

	static double phase_correlate_rt2(Mat & a, Mat & b, double & rotation, Point2d & translation);

	static double phase_correlate_rtOptimal(Mat & a, Mat & b, double & rotation, Point2d & translation);
	static double phase_correlate_rtOptimal_ed(Mat & a, Mat & b, double & rotation, Point2d & translation);
	static double PC1D(Mat & n, Mat & n2);
};

void ll_pad(Mat & image, Mat & output, Size size);