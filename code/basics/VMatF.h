/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description: 
		The .h file for VMatF, which is a float, single channel only 3D version of opencv's Mat object

	depends on: Pixel3DSet, R3

	*requires GPU
*/

#pragma once



#include "Pixel3DSet.h"
#include "R3.h"
#include <vector>
#include <string>
#include <functional>
using namespace std;
using namespace ll_R3;
using namespace ll_pix3d;
using namespace ll_siobj;

class VMat;


class VMat
{
public:
	int s;
	float * data;
	int s2, s3;
	VMat(int sIn = 8);
	VMat(int sIn, vector<R3> l, float offValue, float onValue);
	VMat(int sIn, Pixel3DSet l, float offValue, bool l_is_scaled = false);
	~VMat();
	VMat(const VMat & cp);
	VMat(string fn);
	VMat & operator = (const VMat & cp);
	void copyInVM(const VMat & cp);
	//sets up everything once s is set, assumes no memory is allocated
	void setup_init();
	void free();
	inline float & VMat::at(int x, int y, int z)
	{
		return data[z*s2 + y*s + x];
	}
	inline float & VMat::operator ()(int x, int y, int z)
	{
		return data[z*s2 + y*s + x];
	}
	inline float VMat::at(R3 & r)
	{
		float pix1, pix2, tmp, tmp2;
		Point3i p1((int)r.x, (int)r.y, (int)r.z);
		
		float a = (inbounds(p1.x,p1.y, p1.z))? (float) at(p1.x, p1.y, p1.z): 0.0f;
		float b = (inbounds(p1.x+1,p1.y, p1.z))? (float) at(p1.x+1, p1.y, p1.z): 0.0f;
		float c = (inbounds(p1.x, p1.y+1, p1.z))? (float) at(p1.x, p1.y+1, p1.z): 0.0f;
		float d = (inbounds(p1.x+1,p1.y+1, p1.z))? (float) at(p1.x+1, p1.y+1, p1.z): 0.0f;
		p1.z++;
		float e = (inbounds(p1.x,p1.y, p1.z))? (float) at(p1.x, p1.y, p1.z): 0.0f;
		float f = (inbounds(p1.x+1,p1.y, p1.z))? (float) at(p1.x+1, p1.y, p1.z): 0.0f;
		float g = (inbounds(p1.x,p1.y+1, p1.z))? (float) at(p1.x, p1.y+1, p1.z): 0.0f;
		float h = (inbounds(p1.x+1,p1.y+1, p1.z))? (float) at(p1.x+1, p1.y+1, p1.z): 0.0f;
		p1.z--;
		float dx = r.x - (float) p1.x;
		float dy = r.y - (float) p1.y;
		float dz = r.z - (float) p1.z;

		tmp = (1.0f-dx)*a + dx*b;
		tmp2 = (1.0f-dx)*c + dx*d;
		pix1 = (1.0f-dy)*tmp + dy*tmp2;

		tmp = (1.0f-dx)*e + dx*f;
		tmp2 = (1.0f-dx)*g + dx*h;
		pix2 = (1.0f-dy)*tmp + dy*tmp2;

		return (1.0f-dz)*pix1 + dz*pix2;

	}
	inline float VMat::operator () (R3 & r)
	{
		return at(r);
	}
	void operator *= (float s);
	void operator /= (float s);
	void operator += (float s);
	void operator -= (float s);
	double mean();
	double variance();
	double variance(double mean);
	double stddev();
	double stddev(double mean);
	void noise(double mean, double stddev);
	void add_noise(VMat & ns);
	void noise2(double mean, double stddev);
	double noise(double range); //returns SNR, takes in range and adds random values to volume from -range/2 to range/2
	float average_pixel();
	float average_pixel_non_zeros();
	inline bool inbounds(int x, int y, int z);
	inline Point3i round_point(R3 & r);
	void setAll(float value);
	int count(float threshold);
	vector<ll_R3::R3> collect_points(float th);
	ll_pix3d::Pixel3DSet pixel3dset(float zero = 0.0f);
	SIObj siobj(int block_wid, float threshold);
	void save(string fname);
	void open(string fname);
	
	static Mat translation_matrix(float x, float y, float z);
	static Mat rotation_matrix_x(float angle);
	static Mat rotation_matrix_y(float angle);
	static Mat rotation_matrix_z(float angle);
	static Mat rotation_matrix(float angleX, float angleY, float angleZ);
	static Mat scale_matrix(float sx, float sy, float sz);
	static  Mat center_origin_matrix_inv(int volume_width);
	static  Mat center_origin_matrix(int volume_width);
	static Mat rotation_matrix_center(int volume_width, float rx, float ry, float rz);
	static Mat scale_matrix_center(int volume_width, float sc);
	static Mat scale_matrix_center(int volume_width, R3 sc);
	static Mat transformation_matrix(int volume_width, float rx, float ry, float rz, float sc, float tx, float ty, float tz);
	static Mat scale_matrix(float us);
	static Mat getRYSMat(int volume_width, float sca, float ry);
	static Mat VMat::getRYSMat_gpu(int volume_width, float sca, float ry);

	VMat clone();
	static inline void transform_point(Mat & m, R3 & inp);
	void transform_volume(Mat & m, float nothing = 0.0f);
	void VMat::transform_volume_forward(Mat & m, float nothing = 0.0f);
	void transform_volume(float rx, float ry, float rz, float sc, float tx, float ty, float tz);
	void transform_volume_forward(float rx, float ry, float rz, float sc, float tx, float ty, float tz);
	
	
	
	void save_obj(string fname, int objWid, float threshold);
	void max_min_loc(float & minimum, float & maximum, Point3i * minimumLoc = NULL, Point3i * maximumLoc = NULL);
	Mat slice(int z);
	double mse(VMat & mIn);
	double correlate(VMat & mIn, double scale = 1.0);
	double correlate2(VMat & mIn, double scale = 1.0);
	double meandifference(VMat & mIn);
	double meandifference2(VMat & mIn);
	void box_filter(int am = 1);
	void normalize();
	void threshold(float th);

	//reduces the dimension of a 3d volume to 2d image, by performing the spherical transform, 
	//each pixel at (x,y) is an average value of a ray's intersection with different voxels,
	//this ray is defined by rotating [1,0,0] by y (about the x axis), then by x (along the y axis)
	Mat luke_dimensionsion_reduce_1a(float threshold = 0.01f, Size s = Size(512, 512), float min_radius = 30.0f);
	//this finds the y-axis rotation directly
	float fast_y_rotation_estimation_1a(VMat & signal_2, Size s = Size(512, 512), float min_radius = 10.0f);

	

	//reduces the dimension of a 3d volume to a 2d image, by taking 
	//photographs along a particular axis (specified by axis) and uses the average value down said axis
	Mat luke_dimension_reduce_2a(float threshold = 0.01f, int axis = 0, Size s = Size(512, 512));
	R3 luke_fast_translation_estimation_2a(VMat & signal_2, Size s = Size(512, 512));

	//reduces the dimension of a 3d volume to a 2d image, by taking
	//a photograph of the voxels along a specified axis and using the voxel value,
	//if there are no values, 0 is set, uses the first intersection
	Mat luke_dimension_reduce_3a(float threshold = 0.01f, int axis = 0, Size s = Size(512, 512));

	Mat luke_dimension_reduce_4a(float threshold = 0.01f, int axis = 0, Size s = Size(512, 512));

	//does not work
	//void phase_correlate_rst(VMat & v2, float & yrotation, float & scale, R3 & translation, Size s = Size(512, 512));

	//clean sets all points with a distance from 0 smaller than dist to 0.0f
	void clean(float dist = 0.001f);

	void edge_detect(float scalar = 1.0f, bool normalize_after = false);


	string bin_string(int num_bins);

	R3 center_location();

	//static methods:

	static Mat pca_lukes_pc_t(VMat & v1, VMat & v2, bool edge_detect = false, Size s = Size(512,512));

	//compute the registration matrix between two volumes using pca
	static Mat pca(VMat & v1, VMat & v2, bool edge_detect = false, float clean_amount = 0.2f);

	//double compute pca: can find better results lol
	static Mat pca_double(VMat & v1, VMat & v2, bool edge_detect = false, float clean_amount = 0.1f);

	//performs a max per voxel like v1(x,y,z) = (v1(x,y,z)>=threshold && v2(x,y,z)>=threshold)? max(v1(x,y,z),v2(x,y,z)) : 0
	static void or(VMat & v1, VMat & v2, float threshold);

	//swaps quadrants so dc value is in the center
	static void swap_quadrants(VMat & v);

	//performs a log of each voxel element
	static void log_transform(VMat & v);

	//performs a filtering operation on a point to it can give the output x,y,z values of the translation from phase correlation peak
	static Point3i filter_pc(Point3i a, int s);
	static int filter_pc(int a, int s);

	//grabs the log polar scalar which is based on the volume width
	static float log_polar_scalar(float volume_width);

	//performs a filtering operation, turning the peak retreived during phase_correlate_rst into scale and rotation params
	static void filter_pc(Point3i pc, float & rotation, float & scale, int N);

	//performs a log_polar on a point, next function is inverse
	static  void log_polar(R3 & p, int s);
	static  void log_polar_inv(R3 & p, int s);

	//performs cpu version of log_polar transform, next function performs the inverse
	void log_polar(VMat & s);
	void log_polar_inv(VMat & s);

	//computes a correction matrix for two volumes both with 2 correlated points
	static Mat correction_matrix_up_axis(R3 p1a, R3 p1b, R3 p2a, R3 p2b);
	static Mat correct_volume_up_vector_rotation_matrix(R3 p1a, R3 p1b);

	static Mat correction_matrix_right_axis(R3 p1a, R3 p1b, R3 p2a, R3 p2b);
	static Mat correct_volume_right_vector_rotation_matrix(R3 p1a, R3 p1b);
	
	static Mat new_up_mat(R3 new_up, R3 center);
	static Mat new_right_mat(R3 new_up, R3 center);

	Mat pca_correct_up();

	Mat pca_correct_right();

	

};




