#pragma once
#include <OpenNI.h>
#include "locv3.h"

#include "Pixel3DSet.h"

//requires:
//	opencv 3.0
//	openni2

//forward declaration
namespace ll_openni { class LL_APXCapture; }

namespace ll_openni
{
	
	//maximum I have read is 443, will cap at 1000
	class LL_AXPCapture
	{
	public:
		VideoCapture cap;
		void init(); //initializes openni
		bool registration_on();//checks if registration is on
		
		bool ready(); //call before getting images
		void destroy(); //frees up and shuts down camera

		//functions to retrieve images and information
		bool read_rgb(Mat & m);
		bool read_gs(Mat & m);
		bool read_valid_depth(Mat & m);
		bool read_disparity(Mat & m);

		//depth retrieval:
		bool read_depth_raw(Mat & m);
		bool read_depth_8uc1(Mat & m, double scalar = 0.0255); // can range from .0255 (max depth range) to .05 half depth range()
		bool read_depth_32fc1(Mat & m, double scalar = 0.0001); //can range from 0.0001 (max depth range) to .0002 (half depth range)
		bool read_point_cloud(Mat & m);
		bool point_cloud_normalize(ll_pix3d::Pix3D & rv, float scalar = 384.0f, float maximum_axis_distance = 10.0f);
		void extract_point_cloud(Mat & color_image, Mat & valid_depth_image, Mat & point_cloud_image, vector<Vec3f> & points_out, vector<Vec3b> & colours_out);
		void extract_point_cloud_normalize(Mat & color_image, Mat & valid_depth_image, Mat & point_cloud_image, vector<Vec3f> & points_out, vector<Vec3b> & colours_out, float scalar = 384.0f, float maximum_axis_distance = 10.0f);
		void extract_mesh_normalize(Mat & color_image, Mat & valid_depth_image, Mat & point_cloud_image, vector<Vec3f> & points_out, vector<Vec3b> & colours_out, vector<Point3i> & triangle_list, float scalar = 384.0f, float maximum_axis_distance = 10.0f);
		void extract_mesh_normalize_smooth(Mat & color_image, Mat & valid_depth_image, Mat & point_cloud_image, vector<Vec3f> & points_out, vector<Vec3b> & colours_out, vector<Point3i> & triangle_list, float scalar = 384.0f, float maximum_axis_distance = 10.0f);
		void save_pix3d_obj(string fname, vector<Vec3f> & points, vector<Vec3b> & colours);
	};

}
