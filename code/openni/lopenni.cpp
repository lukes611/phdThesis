#include "lopenni.h"
#include <fstream>
#include <functional>
using namespace std;
using namespace ll_pix3d;

namespace ll_openni
{
	void LL_AXPCapture::init()
	{
		openni::Status rc = openni::STATUS_OK;
		openni::Device device;
		openni::VideoStream depth, color;
		const char* deviceURI = openni::ANY_DEVICE;
		rc = openni::OpenNI::initialize();
		if (rc != openni::STATUS_OK)
		{
			ll_error("failted to initialize xtion pro live: " + string(openni::OpenNI::getExtendedError()));
		}
		cap.open(CAP_OPENNI2);
		
		if(!cap.isOpened())
		{
			ll_error("failed to open the video capture suite");
		}
		bool mode_res = cap.set(CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CAP_OPENNI_QVGA_30HZ); //CAP_OPENNI_QVGA_60HZ = for faster?
		//cap.set(CAP_PROP_OPENNI2_MIRROR, 0);
		

		if(!mode_res)
		{
			ll_error("XTionCamera::init() failed at mode_res setting");
		}
	}

	bool LL_AXPCapture::registration_on()
	{
		return cap.get( CAP_PROP_OPENNI_REGISTRATION ) == 1;
	}

	bool LL_AXPCapture::ready()
	{
		return cap.grab();
	}

	void LL_AXPCapture::destroy()
	{
		cap.release();
		openni::OpenNI::shutdown();
	}

	bool LL_AXPCapture::read_rgb(Mat & m)
	{
		return cap.retrieve(m, CAP_OPENNI_BGR_IMAGE);
	}

	bool LL_AXPCapture::read_gs(Mat & m)
	{
		return cap.retrieve(m,CAP_OPENNI_GRAY_IMAGE);
	}

	bool LL_AXPCapture::read_valid_depth(Mat & m)
	{
		return cap.retrieve(m,CAP_OPENNI_VALID_DEPTH_MASK);
	}
	
	bool LL_AXPCapture::read_disparity(Mat & m)
	{
		return cap.retrieve(m,CAP_OPENNI_DISPARITY_MAP);
	}

	bool LL_AXPCapture::read_depth_raw(Mat & m)
	{
		return cap.retrieve(m,CAP_OPENNI_DEPTH_MAP);
	}

	bool LL_AXPCapture::read_depth_8uc1(Mat & m, double scalar)
	{
		Mat tmp;
		if(cap.retrieve(tmp,CAP_OPENNI_DEPTH_MAP))
		{
			tmp.convertTo(m, CV_8UC1,scalar);
			return true;
		}
		return false;
	}
	
	bool LL_AXPCapture::read_depth_32fc1(Mat & m, double scalar)
	{
		Mat tmp;
		if(cap.retrieve(m,CAP_OPENNI_DEPTH_MAP))
		{
			m.convertTo(tmp, CV_32FC1,scalar);
			threshold(tmp, m, 1.0, 1.0, CV_THRESH_TRUNC);
			return true;
		}
		return false;
	}

	bool LL_AXPCapture::read_point_cloud(Mat & m)
	{
		return cap.retrieve(m, CV_CAP_OPENNI_POINT_CLOUD_MAP);
	}

	void LL_AXPCapture::extract_point_cloud(Mat & color_image, Mat & valid_depth_image, Mat & point_cloud_image, vector<Vec3f> & points_out, vector<Vec3b> & colours_out)
	{
		int w = color_image.size().width;
		int h = color_image.size().height;
		int s = w*h;
		int count = 0;
		if(points_out.size() < s) points_out.resize(s);
		if(colours_out.size() < s) colours_out.resize(s);
		Vec3f * pt = (Vec3f *) point_cloud_image.data;
		Vec3b * cpt = (Vec3b *) color_image.data;
		unsigned char * vpt = (unsigned char *) color_image.data;
		for(unsigned int i = 0; i < s; i++, pt++, cpt++, vpt++)
		{
			if(*vpt != 0x00)
			{
				points_out[count] = *pt;
				colours_out[count] = *cpt;
				count++;
			}
		}
		points_out.resize(count);
		colours_out.resize(count);
	}

	void LL_AXPCapture::extract_point_cloud_normalize(Mat & color_image, Mat & valid_depth_image, Mat & point_cloud_image, vector<Vec3f> & points_out, vector<Vec3b> & colours_out, float scalar, float maximum_axis_distance)
	{
		int w = color_image.size().width;
		int h = color_image.size().height;
		int s = w*h;
		int count = 0;
		scalar /= maximum_axis_distance;
		if(points_out.size() < s) points_out.resize(s);
		if(colours_out.size() < s) colours_out.resize(s);
		Vec3f * pt = (Vec3f *) point_cloud_image.data;
		Vec3b * cpt = (Vec3b *) color_image.data;
		unsigned char * vpt = (unsigned char *) color_image.data;
		Vec3d minimum;
		Mat pc_channels[3];
		split(point_cloud_image, pc_channels);
		minMaxLoc(pc_channels[0], &minimum[0]);
		minMaxLoc(pc_channels[1], &minimum[1]);
		minMaxLoc(pc_channels[2], &minimum[2]);
		pc_channels[0] -= minimum[0];
		pc_channels[1] -= minimum[1];
		pc_channels[2] -= minimum[2];
		merge(pc_channels, 3, point_cloud_image);
		point_cloud_image *= scalar;
		for(unsigned int i = 0; i < s; i++, pt++, cpt++, vpt++)
		{
			if(*vpt != 0x00)
			{
				points_out[count] = *pt;
				colours_out[count] = *cpt;
				count++;
			}
		}
		points_out.resize(count);
		colours_out.resize(count);
	}

	bool LL_AXPCapture::point_cloud_normalize(Pix3D & rv, float scalar, float maximum_axis_distance)
	{
		
		int w = 640;
		int h = 480;
		Size size(640, 480);

		rv.free();
		rv.type = Pix3D::ImageType;
		rv.count = 640 * 480;
		rv.points = new ll_R3::R3[rv.count];
		rv.colors = new Vec3b[rv.count];
		rv.validDepth = new bool[rv.count];
		Mat colors, validDepth, pointCloud;
		
		//set up color image:
		if(this->read_rgb(colors))
			resize(colors, colors, size);
		else return false;

		memcpy(rv.colors, colors.data, sizeof(Vec3b) * rv.count);

		if(!this->read_valid_depth(validDepth)) return false;

		for(unsigned int i = 0; i < rv.count; i++) rv.validDepth[i] = validDepth.data[i] != 0x00;

		if(!this->read_point_cloud(pointCloud)) return false;

		scalar /= maximum_axis_distance;
		Vec3d minimum;
		Mat pc_channels[3];
		split(pointCloud, pc_channels);
		minMaxLoc(pc_channels[0], &minimum[0]);
		minMaxLoc(pc_channels[1], &minimum[1]);
		minMaxLoc(pc_channels[2], &minimum[2]);
		pc_channels[0] -= minimum[0];
		pc_channels[1] -= minimum[1];
		pc_channels[2] -= minimum[2];
		merge(pc_channels, 3, pointCloud);
		pointCloud *= scalar;
		
		Vec3f * ptr = (Vec3f *) pointCloud.data;
		for(unsigned int i = 0; i < rv.count; i++, ptr++) rv.points[i] = ll_R3::R3((*ptr)[0], (*ptr)[1], (*ptr)[2]);
		return true;
	}

	void LL_AXPCapture::extract_mesh_normalize(Mat & color_image, Mat & valid_depth_image, Mat & point_cloud_image, vector<Vec3f> & points_out, vector<Vec3b> & colours_out, vector<Point3i> & triangle_list, float scalar, float maximum_axis_distance)
	{
		int w = color_image.size().width;
		int h = color_image.size().height;
		int s = w*h;
		int tcount = 0;
		scalar /= maximum_axis_distance;
		Vec3d minimum;
		Mat pc_channels[3];
		split(point_cloud_image, pc_channels);
		minMaxLoc(pc_channels[0], &minimum[0]);
		minMaxLoc(pc_channels[1], &minimum[1]);
		minMaxLoc(pc_channels[2], &minimum[2]);
		pc_channels[0] -= minimum[0];
		pc_channels[1] -= minimum[1];
		pc_channels[2] -= minimum[2];
		merge(pc_channels, 3, point_cloud_image);
		point_cloud_image *= scalar;
		float max_dist = 5.0f;
		float d[4];
		for(int y = 0; y < h-1; y++)
		{
			for(int x = 0; x < w-1; x++)
			{
				if(valid_depth_image.at<unsigned char>(y,x) != 0x00
					&&
					valid_depth_image.at<unsigned char>(y,x+1) != 0x00
					&&
					valid_depth_image.at<unsigned char>(y+1,x) != 0x00
					&&
					valid_depth_image.at<unsigned char>(y+1,x+1) != 0x00)
				{
					d[0] = point_cloud_image.at<Vec3f>(y,x)[2];
					d[1] = point_cloud_image.at<Vec3f>(y,x+1)[2];
					d[2] = point_cloud_image.at<Vec3f>(y+1,x)[2];
					d[3] = point_cloud_image.at<Vec3f>(y+1,x+1)[2];
					if	(		abs(d[0]-d[1]) < max_dist
						&&
								abs(d[0]-d[2]) < max_dist
						&&
								abs(d[0]-d[3]) < max_dist
					)
					{
						points_out.push_back(point_cloud_image.at<Vec3f>(y,x));
						points_out.push_back(point_cloud_image.at<Vec3f>(y+1,x));
						points_out.push_back(point_cloud_image.at<Vec3f>(y+1,x+1));
						points_out.push_back(point_cloud_image.at<Vec3f>(y,x+1));
						colours_out.push_back(color_image.at<Vec3b>(y,x));
						colours_out.push_back(color_image.at<Vec3b>(y,x));
						triangle_list.push_back(Point3i(tcount, tcount+1, tcount+2));
						triangle_list.push_back(Point3i(tcount, tcount+2, tcount+3));
						tcount+=4;
					}
				}
			}
		}
	}

	void LL_AXPCapture::extract_mesh_normalize_smooth(Mat & color_image, Mat & valid_depth_image, Mat & point_cloud_image, vector<Vec3f> & points_out, vector<Vec3b> & colours_out, vector<Point3i> & triangle_list, float scalar, float maximum_axis_distance)
	{
		int w = color_image.size().width;
		int h = color_image.size().height;
		int s = w*h;
		int tcount = 0;
		scalar /= maximum_axis_distance;
		Vec3d minimum;
		Mat pc_channels[3];
		split(point_cloud_image, pc_channels);
		minMaxLoc(pc_channels[0], &minimum[0]);
		minMaxLoc(pc_channels[1], &minimum[1]);
		minMaxLoc(pc_channels[2], &minimum[2]);
		pc_channels[0] -= minimum[0];
		pc_channels[1] -= minimum[1];
		pc_channels[2] -= minimum[2];
		merge(pc_channels, 3, point_cloud_image);
		point_cloud_image *= scalar;
		float max_dist = 1.0f;
		float d[4];
		medianBlur(point_cloud_image, point_cloud_image, 3);
		for(int y = 0; y < h-1; y++)
		{
			for(int x = 0; x < w-1; x++)
			{
				if(valid_depth_image.at<unsigned char>(y,x) != 0x00
					&&
					valid_depth_image.at<unsigned char>(y,x+1) != 0x00
					&&
					valid_depth_image.at<unsigned char>(y+1,x) != 0x00
					&&
					valid_depth_image.at<unsigned char>(y+1,x+1) != 0x00)
				{
					d[0] = point_cloud_image.at<Vec3f>(y,x)[2];
					d[1] = point_cloud_image.at<Vec3f>(y,x+1)[2];
					d[2] = point_cloud_image.at<Vec3f>(y+1,x)[2];
					d[3] = point_cloud_image.at<Vec3f>(y+1,x+1)[2];
					if	(		abs(d[0]-d[1]) < max_dist
						&&
								abs(d[0]-d[2]) < max_dist
						&&
								abs(d[0]-d[3]) < max_dist
					)
					{
						points_out.push_back(point_cloud_image.at<Vec3f>(y,x));
						points_out.push_back(point_cloud_image.at<Vec3f>(y+1,x));
						points_out.push_back(point_cloud_image.at<Vec3f>(y+1,x+1));
						points_out.push_back(point_cloud_image.at<Vec3f>(y,x+1));
						colours_out.push_back(color_image.at<Vec3b>(y,x));
						colours_out.push_back(color_image.at<Vec3b>(y,x));
						triangle_list.push_back(Point3i(tcount, tcount+1, tcount+2));
						triangle_list.push_back(Point3i(tcount, tcount+2, tcount+3));
						tcount+=4;
					}
				}
			}
		}
	}

	void LL_AXPCapture::save_pix3d_obj(string fname, vector<Vec3f> & points, vector<Vec3b> & cols)
	{
		ofstream fi(fname, ios::out);
		string rv = "";
		fi << points.size() << endl;
		for(int i = 0; i < points.size(); i++)
		{
			rv += to_string(points[i][0]) + string(" ") + to_string(points[i][1]) + string(" ") + to_string(points[i][2]) + string(" ");
			rv += to_string(cols[i][0]) + string(" ") + to_string(cols[i][1]) + string(" ") + to_string(cols[i][2]) + string("\n");
		}
		fi << rv;
		fi.close();
	}

}