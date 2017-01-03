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
	VMat(int sIn, std::vector<R3> l, float offValue, float onValue);
	VMat(int sIn, Pixel3DSet l, float offValue, bool l_is_scaled = false);
	VMat(SIObj & ob, int sIn = 256, int pad = 20);
	~VMat();
	VMat(const VMat & cp);
	VMat(std::string fn);
	VMat & operator = (const VMat & cp);
	void copyInVM(const VMat & cp);
	//sets up everything once s is set, assumes no memory is allocated
	void setup_init();
	void free();
	inline float & at(int x, int y, int z)
	{
		return data[z*s2 + y*s + x];
	}
	inline float & operator ()(int x, int y, int z)
	{
		return data[z*s2 + y*s + x];
	}
	inline float at(R3 & r)
	{
		float pix1, pix2, tmp, tmp2;
		cv::Point3i p1((int)r.x, (int)r.y, (int)r.z);

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
	inline float operator () (R3 & r)
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
	bool inbounds(int x, int y, int z);
	inline cv::Point3i round_point(R3 & r);
	void setAll(float value);
	int count(float threshold);
	std::vector<ll_R3::R3> collect_points(float th);
	ll_pix3d::Pixel3DSet pixel3dset(float zero = 0.0f);
	SIObj siobj(int block_wid, float threshold);
	void save(std::string fname);
	void open(std::string fname);

	static cv::Mat translation_matrix(float x, float y, float z);
	static cv::Mat rotation_matrix_x(float angle);
	static cv::Mat rotation_matrix_y(float angle);
	static cv::Mat rotation_matrix_z(float angle);
	static cv::Mat rotation_matrix(float angleX, float angleY, float angleZ);
	static cv::Mat scale_matrix(float sx, float sy, float sz);
	static cv::Mat center_origin_matrix_inv(int volume_width);
	static cv::Mat center_origin_matrix(int volume_width);
	static cv::Mat rotation_matrix_center(int volume_width, float rx, float ry, float rz);
	static cv::Mat scale_matrix_center(int volume_width, float sc);
	static cv::Mat scale_matrix_center(int volume_width, R3 sc);
	static cv::Mat transformation_matrix(int volume_width, float rx, float ry, float rz, float sc, float tx, float ty, float tz);
	static cv::Mat scale_matrix(float us);
	static cv::Mat getRYSMat(int volume_width, float sca, float ry);
	static cv::Mat getRYSMat_gpu(int volume_width, float sca, float ry);

	VMat clone();
	static inline void transform_point(cv::Mat & m, R3 & inp);
	void transform_volume(cv::Mat & m, float nothing = 0.0f);
	void transform_volume_forward(cv::Mat & m, float nothing = 0.0f);
	void transform_volume(float rx, float ry, float rz, float sc, float tx, float ty, float tz);
	void transform_volume_forward(float rx, float ry, float rz, float sc, float tx, float ty, float tz);



	void save_obj(std::string fname, int objWid, float threshold);
	void max_min_loc(float & minimum, float & maximum, cv::Point3i * minimumLoc = NULL, cv::Point3i * maximumLoc = NULL);
	cv::Mat slice(int z);
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
	cv::Mat luke_dimensionsion_reduce_1a(float threshold = 0.01f, cv::Size s = cv::Size(512, 512), float min_radius = 30.0f);
	//this finds the y-axis rotation directly
	float fast_y_rotation_estimation_1a(VMat & signal_2, cv::Size s = cv::Size(512, 512), float min_radius = 10.0f);



	//reduces the dimension of a 3d volume to a 2d image, by taking
	//photographs along a particular axis (specified by axis) and uses the average value down said axis
	cv::Mat luke_dimension_reduce_2a(float threshold = 0.01f, int axis = 0, cv::Size s = cv::Size(512, 512));
	R3 luke_fast_translation_estimation_2a(VMat & signal_2, cv::Size s = cv::Size(512, 512));

	//reduces the dimension of a 3d volume to a 2d image, by taking
	//a photograph of the voxels along a specified axis and using the voxel value,
	//if there are no values, 0 is set, uses the first intersection
	cv::Mat luke_dimension_reduce_3a(float threshold = 0.01f, int axis = 0, cv::Size s = cv::Size(512, 512));

	cv::Mat luke_dimension_reduce_4a(float threshold = 0.01f, int axis = 0, cv::Size s = cv::Size(512, 512));

	//does not work
	//void phase_correlate_rst(VMat & v2, float & yrotation, float & scale, R3 & translation, Size s = Size(512, 512));

	//clean sets all points with a distance from 0 smaller than dist to 0.0f
	void clean(float dist = 0.001f);

	void edge_detect(float scalar = 1.0f, bool normalize_after = false);


	std::string bin_string(int num_bins);

	R3 center_location();

	//static methods:

	static cv::Mat pca_lukes_pc_t(VMat & v1, VMat & v2, bool edge_detect = false, cv::Size s = cv::Size(512,512));

	//compute the registration matrix between two volumes using pca
	static cv::Mat pca(VMat & v1, VMat & v2, bool edge_detect = false, float clean_amount = 0.2f);

	//double compute pca: can find better results lol
	static cv::Mat pca_double(VMat & v1, VMat & v2, bool edge_detect = false, float clean_amount = 0.1f);

	//performs a max per voxel like v1(x,y,z) = (v1(x,y,z)>=threshold && v2(x,y,z)>=threshold)? max(v1(x,y,z),v2(x,y,z)) : 0
	static void Or(VMat & v1, VMat & v2, float threshold);

	//swaps quadrants so dc value is in the center
	static void swap_quadrants(VMat & v);

	//performs a log of each voxel element
	static void log_transform(VMat & v);

	//performs a filtering operation on a point to it can give the output x,y,z values of the translation from phase correlation peak
	static cv::Point3i filter_pc(cv::Point3i a, int s);
	static int filter_pc(int a, int s);

	//grabs the log polar scalar which is based on the volume width
	static float log_polar_scalar(float volume_width);

	//performs a filtering operation, turning the peak retreived during phase_correlate_rst into scale and rotation params
	static void filter_pc(cv::Point3i pc, float & rotation, float & scale, int N);

	//performs a log_polar on a point, next function is inverse
	static  void log_polar(R3 & p, int s);
	static  void log_polar_inv(R3 & p, int s);

	//performs cpu version of log_polar transform, next function performs the inverse
	void log_polar(VMat & s);
	void log_polar_inv(VMat & s);

	//computes a correction matrix for two volumes both with 2 correlated points
	static cv::Mat correction_matrix_up_axis(R3 p1a, R3 p1b, R3 p2a, R3 p2b);
	static cv::Mat correct_volume_up_vector_rotation_matrix(R3 p1a, R3 p1b);

	static cv::Mat correction_matrix_right_axis(R3 p1a, R3 p1b, R3 p2a, R3 p2b);
	static cv::Mat correct_volume_right_vector_rotation_matrix(R3 p1a, R3 p1b);

	static cv::Mat new_up_mat(R3 new_up, R3 center);
	static cv::Mat new_right_mat(R3 new_up, R3 center);

	cv::Mat pca_correct_up();

	cv::Mat pca_correct_right();


	//correlate
    void filter(VMat & filter);
    VMat resize(int ns);
	VMat resizeFwd(int ns);


};

namespace LukeLincoln
{


    template <class T>
    class LVol{
    public:
        int width, height, depth;
        R3 corner;
        float resolution; //number of items per voxel
        T * data;
        int widthTimesHeight;

        //default constructor
        LVol()
        {
            width = height = depth = widthTimesHeight = 0;
            corner = R3();
            resolution = 1.0f;
            data = NULL;
        }
        //basic constructor
        LVol(int width, int height, int depth, T defaultValue = 0)
        {
            corner = R3();
            resolution = 1.0f;
            this->width = width;
            this->height = height;
            this->depth = depth;
            this->widthTimesHeight = width * height;
            data = new T[width * height * depth];
            setEach(defaultValue);
        }

        LVol(R3 & ncorner, int w, int h, int d, float res, T defaultVal = 0)
        {
            corner = ncorner;
            resolution = res;
            this->width = w;
            this->height = h;
            this->depth = d;
            this->widthTimesHeight = width * height;
            data = new T[width * height * depth];
            setEach(defaultVal);
        }

        LVol(const LVol & c) { copyIn(c); }
        LVol & operator = (const LVol & c) { if(this != &c) {copyIn(c);} return *this; }


        //destructor
        ~LVol() { free(); }

        void free()
        {
            if(width > 0 && height > 0 && depth > 0)
            {
                width = height = depth = 0;
                delete [] data;
            }
        }

        void copyIn(const LVol & c)
        {
            free();
            corner = c.corner;
            resolution = c.resolution;
            width = c.width;
            height = c.height;
            depth = c.depth;
            widthTimesHeight = width * height;
            int SS = width * height * depth;
            data = new T[SS];
            for(int i = 0; i < SS; i++) data[i]=c.data[i];


        }



        //gets the index into the volume given a real point
        cv::Point3i index(ll_R3::R3 & p)
        {
            ll_R3::R3 x = (p - corner) / resolution;
            return cv::Point3i(x.x + 0.5f, x.y + 0.5f, x.z + 0.5f);
        }

        //gets the real point from a given volume index
        ll_R3::R3 unIndex(cv::Point3i & p)
        {
            ll_R3::R3 ret((float)p.x, (float)p.y, (float)p.z);
            return ret*resolution + corner;
        }

        //returns true is a given index is within bounds
        bool inbounds(cv::Point3i & p)
        {
            return  p.x >= 0 && p.y >= 0 && p.z >= 0
            &&      p.x < width && p.y < height && p.z < depth;
        }

        ll_R3::R3 getMax()
        {
            cv::Point3i p(width, height, depth);
            return unIndex(p);
        }

        //returns the stats about a given set of points
        void stats(std::vector<R3> & points, R3 & cornerOut, R3 & maxCorn)
        {
            if(points.size() == 0) return;
            cornerOut = points[0];
            maxCorn = points[0];
            for(int i = 1; i < points.size(); i++)
            {
                cornerOut.min(points[i]);
                maxCorn.max(points[i]);
            }
            maxCorn;
            R3 one(2.0f, 2.0f, 2.0f);
            maxCorn += one;
            cornerOut -= one;
        }

        //returns a boolean whether the volume must be resized for the given list,
        //if true then the new corner and w,h,d are returned
        bool mustResize(std::vector<R3> & points, R3 & newCorner, cv::Point3i & newSizes)
        {
            R3 cr, mc;
            //get the stats on the new set of points
            stats(points, cr, mc);
            //std::cout << "got mc << " << mc << std::endl;
            //std::cout << "current is << " << getMax() << std::endl;

            //set the possible new corner and size
            newCorner = corner;
            R3 newMx = getMax();
            newCorner.min(cr);
            newMx.max(mc);


            //if they are the same as the old, no resize necessary
            if(newCorner == corner && newMx == getMax()) return false;

            //std::cout << "should change the newS to " << newMx << std::endl;
            //else compute the new sizes and return true
            newMx = (newMx-newCorner) / resolution;
            newSizes = cv::Point3i(newMx.x, newMx.y, newMx.z);

            return true;

        }

        //add vector<R3> and vector<T> without and with resize and without
        //returns the number of points added
        int addWithoutResize(std::vector<R3> & points, std::vector<T> & data)
        {
            int counter = 0;
            for(int i = 0; i < points.size(); i++)
            {
                R3 point = points[i];
                cv::Point3i ind = index(point);
                if(inbounds(ind))
                {
                    this->operator()(ind.x, ind.y, ind.z) = data[i];
                    counter++;
                }else
                {
                    //std::cout << "could not add: " << point << " in " << getMax() << std::endl;
                }
            }
            return counter;
        }

        int add(std::vector<R3> & points, std::vector<T> & data)
        {
            R3 nc; cv::Point3i ns;
            bool mr = mustResize(points, nc, ns);

            if(!mr) return addWithoutResize(points, data);
            else
            {
                double sizeMB = (sizeof(T) * ns.x * ns.y * ns.z);
                sizeMB /= (1024.0 * 1024.0);
                //std::cout << "sz: " << sizeMB << std::endl;
                if(sizeMB > 3000.0) return addWithoutResize(points, data);


                //std::cout << "changing size to " << ns << " and corner to " << nc << "\n";
                //std::cin.get();

                LVol<T> cp(nc, ns.x+10, ns.y+10, ns.z+10, resolution);

                for(int z = 0; z < depth; z++)
                {
                    for(int y=0; y < height; y++)
                    {
                        for(int x = 0; x < width; x++)
                        {
                            cv::Point3i from(x,y,z);
                            T d = this->operator()(x,y,z);
                            R3 to = unIndex(from);
                            cv::Point3i reIndex = cp.index(to);
                            if(cp.inbounds(reIndex))
                                cp(reIndex.x, reIndex.y, reIndex.z) = d;
                        }
                    }
                }
                int rv = cp.addWithoutResize(points, data);

                *this = cp;
                return rv;
            }
        }


        T & operator () (int x, int y, int z)
        {
            return data[z * widthTimesHeight + y * width + x];
        }

        //return how large it is in mb
        double numMB()
        {
            double scalar = 1024.0 * 1024.0; scalar = 1.0 / scalar;
            return scalar * (double)(sizeof(T) * width * height * depth);
        }

        //set each value to this
        void setEach(T & v)
        {
            int S = width * height * depth;
            for(int i = 0; i < S; i++) data[i] = v;
        }


    };


    Pixel3DSet makePixel3DSet(LVol<cv::Vec3b> & in);

}


