#include "Pixel3DSet.h"
#include <time.h>
#include <fstream>
#include "BitReaderWriter.h"
#include <stack>
using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_siobj;

namespace ll_pix3d
{

	Pixel3DSet::Pixel3DSet()
	{
	}

	Pixel3DSet::Pixel3DSet(const Pixel3DSet & a)
	{
		copyFrom(a);
	}
	Pixel3DSet & Pixel3DSet::operator = (const Pixel3DSet & a)
	{
		if(this == &a) return *this;
		copyFrom(a);
		return *this;
	}
	Pixel3DSet::Pixel3DSet(const vector<R3> & pointz, const vector<Vec3b> colorz)
	{
		points = pointz;
		colors = colorz;
	}
	Pixel3DSet::Pixel3DSet(const vector<R3> & pointz, Vec3b on_color)
	{
		points = pointz;
		for(int i = 0; i < points.size(); i++) colors.push_back(on_color);

	}
	Pixel3DSet::Pixel3DSet(const vector<Vec3f> & pointz, const vector<Vec3b> colorz)
	{
		points = vector<R3>(pointz.size());
		int _s = (int)points.size();
		for(int i = 0; i < _s; i++) points[i] = R3(pointz[i][0], pointz[i][1], pointz[i][2]);
		colors = colorz;
	}
	Pixel3DSet::Pixel3DSet(R3 * l, Vec3b * color, int n)
	{
		points.clear();
		colors.clear();
		for(int i = 0; i < n; i++)
		{
			points.push_back(l[i]);
			colors.push_back(color[i]);
		}
	}
	Pixel3DSet::Pixel3DSet(const Pix3D & p)
	{
		points.clear();
		colors.clear();
		for(int i = 0; i < p.count; i++)
		{
			if(!p.hasVd() || p.validDepth[i])
			{
				points.push_back(p.points[i]);
				colors.push_back(p.colors[i]);
			}
		}
	}

	void Pixel3DSet::normalize(int s)
	{
		R3 mn, mx;
		min_max_R3(mn, mx);

		float fx = mx.max();
		float fn = mn.min();
		fx -= fn;
		int points_size = (int)points.size();
		for(int i = 0; i < points_size; i++)
		{
			R3 c = points[i];
			c -= mn;
			c *= (((float)s) / fx);
			int xx = static_cast<int>(round(c.x));
			int yy = static_cast<int>(round(c.y));
			int zz = static_cast<int>(round(c.z));
			points[i] = R3((float)xx, (float)yy, (float)zz);
			points[i].x += 0.5f;
			points[i].y += 0.5f;
			points[i].z += 0.5f;
		}
	}

	void Pixel3DSet::round_points()
	{
		int points_size = static_cast<int>(points.size());
		R3 half(0.5f, 0.5f, 0.5f);
		for(int i = 0; i < points_size; i++)
		{
			R3 c = points[i];
			int xx = static_cast<int>(round(c.x));
			int yy = static_cast<int>(round(c.y));
			int zz = static_cast<int>(round(c.z));
			points[i] = R3((float)xx, (float)yy, (float)zz);
			points[i] += half;
		}
	}

	Pix3D Pixel3DSet::pix3d() const
	{
		return Pix3D(points, colors);
	}


	void Pixel3DSet::clear()
	{
		colors.clear();
		points.clear();
	}
	void Pixel3DSet::save(string fn)
	{
		Pix3DC t = pix3d().compress();
		t.save(fn);
		t.free();
	}
	void Pixel3DSet::open(string fn)
	{
		Pix3DC t;
		t.open(fn);
		Pix3D t1;
		t1.decompress(t);
		*this = Pixel3DSet(t1);
		t.free();
	}
	void Pixel3DSet::copyFrom(const Pixel3DSet & a)
	{
		points = a.points;
		colors = a.colors;
	}
	Pixel3DSet Pixel3DSet::clone()
	{
		Pixel3DSet rv;
		rv.copyFrom(*this);
		return rv;
	}
	inline int Pixel3DSet::size()
	{
		return (int)points.size();
	}
	inline R3 & Pixel3DSet::operator [] (int index)
	{
		return points[index];
	}
	void Pixel3DSet::push_back(R3 & a, Vec3b c)
	{
		points.push_back(a);
		colors.push_back(c);
	}
	void Pixel3DSet::UNION(Pixel3DSet & a)
	{
		for(int i = 0; i < a.size(); i++)
		{
			points.push_back(a[i]);
			colors.push_back(a.colors[i]);
		}
	}
	Pixel3DSet Pixel3DSet::operator + (Pixel3DSet & a)
	{
		Pixel3DSet rv = *this;
		rv.UNION(a);
		return rv;
	}
	Pixel3DSet & Pixel3DSet::operator += (Pixel3DSet & a)
	{
		UNION(a);
		return *this;
	}
	Mat Pixel3DSet::translation_matrix(float x, float y, float z)
	{
		float md[16] = {
		1.0f, 0.0f, 0.0f, x,
		0.0f, 1.0f, 0.0f, y,
		0.0f, 0.0f, 1.0f, z,
		0.0f, 0.0f, 0.0f, 1.0f,
		};
		return Mat(4, 4, CV_32FC1, md).clone();
	}
	Mat Pixel3DSet::rotation_matrix_x(float angle)
	{
		float cs = cos(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
		float sn = sin(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
		float md[16] = {
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, cs, -sn, 0.0f,
		0.0f, sn, cs, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f,
		};
		return Mat(4, 4, CV_32FC1, md).clone();
	}
	Mat Pixel3DSet::rotation_matrix_y(float angle)
	{
		float cs = cos(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
		float sn = sin(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
		float md[16] = {
		cs, 0.0f, sn, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		-sn, 0.0f, cs, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f,
		};
		return Mat(4, 4, CV_32FC1, md).clone();
	}
	Mat Pixel3DSet::rotation_matrix_z(float angle)
	{
		float cs = cos(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
		float sn = sin(angle / ll_R3::ll_R3_C::ll_R3_rad2deg);
		float md[16] = {
		cs, -sn, 0.0f, 0.0f,
		sn, cs, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f,
		};
		return Mat(4, 4, CV_32FC1, md).clone();
	}
	Mat Pixel3DSet::rotation_matrix(float angleX, float angleY, float angleZ)
	{
		return rotation_matrix_z(angleZ) * rotation_matrix_y(angleY) * rotation_matrix_x(angleX);
	}
	Mat Pixel3DSet::scale_matrix(float sx, float sy, float sz)
	{
		float md[16] = {
		sx, 0.0f, 0.0f, 0.0f,
		0.0f, sy, 0.0f, 0.0f,
		0.0f, 0.0f, sz, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f,
		};
		return Mat(4, 4, CV_32FC1, md).clone();
	}

	inline Mat Pixel3DSet::center_origin_matrix_inv(R3 p)
	{
		return translation_matrix(-p.x, -p.y, -p.z);
	}
	inline Mat Pixel3DSet::center_origin_matrix(R3 p)
	{
		return translation_matrix(p.x, p.y, p.z);
	}
	R3 Pixel3DSet::getAvg()
	{
		R3 rv;
		for(int i = 0; i < size(); i++)
		{
			rv += points[i];
		}
		if(size() > 0)
		{
			rv *= (1.0f / (float)size());
		}
		return rv;
	}

	void Pixel3DSet::reduce(int size_)
	{
		float size__ = (float)size_;
		int _s2 = size_ * size_;
		int _s3 = _s2 * size_;

		vector<Vec3b> colList(_s3, Vec3b(0,0,0));
		vector<int> counts(_s3, 0);
		R3 mn, mx;
		min_max_R3(mn, mx);
		mx -= mn;
		float maxScalar = (mx.x > mx.y) ? mx.x : mx.y;
		maxScalar = (mx.z > maxScalar)? mx.z : maxScalar;
		for(int i = 0; i < size(); i++)
		{
			R3 p = points[i];
			p -= mn;
			p *= (size__ / maxScalar);
			Point3i index(static_cast<int>(round(p.x)), static_cast<int>(round(p.y)), static_cast<int>(round(p.z)));
			int iindex = index.z * _s2 + index.y * size_ + index.x;
			if(index.x<0 || index.y<0 || index.z<0 || index.x>=size_ || index.y>=size_ || index.z>=size_) continue;
			int currentCount = counts[iindex];
			double currentScalar = currentCount / static_cast<double>(currentCount+1);
			double nextScalar = 1.0 / static_cast<double>(currentCount+1);
			colList[iindex] = ll_color_blend(colList[iindex],colors[i], currentScalar, nextScalar);
			counts[iindex] = counts[iindex] + 1;
		}
		points.clear();
		colors.clear();
		for(int z = 0; z < size_; z++)
		{
			for(int y = 0; y < size_; y++)
			{
				for(int x = 0; x < size_; x++)
				{
					int iindex = z * _s2 + y * size_ + x;
					if(x<0 || y<0 || z<0 || x>=size_ || y>=size_ || z>=size_) continue;

					if(counts[iindex] <= 0) continue;
					R3 p((float)x, (float)y, (float)z);
					p *= (maxScalar / (float)size_);
					p += mn;
					Vec3b color = colList[iindex];
					points.push_back(p);
					colors.push_back(color);
				}
			}
		}
	}
	void Pixel3DSet::min_max_R3(R3 & mn, R3 & mx)
	{
		if(size() <= 0) return;
		mn = points[0];
		mx = points[0];
		for(int i = 1; i < size(); i++)
		{
			mn.min(points[i]);
			mx.max(points[i]);
		}
	}

	Mat Pixel3DSet::rotation_matrix_center(float rx, float ry, float rz, R3 center)
	{
		return center_origin_matrix(center) * rotation_matrix(rx, ry, rz) * center_origin_matrix_inv(center);
	}

	Mat Pixel3DSet::scale_matrix_center(float sc, R3 center)
	{
		return center_origin_matrix(center) * scale_matrix(sc) * center_origin_matrix_inv(center);
	}

	Mat Pixel3DSet::transformation_matrix(float rx, float ry, float rz, float sc, float tx, float ty, float tz, R3 center)
	{
		return translation_matrix(tx, ty, tz) * scale_matrix_center(sc, center) * rotation_matrix_center(rx, ry, rz, center);
	}

	Mat Pixel3DSet::scale_matrix(float us)
	{
		float md[16] = {
		us, 0.0f, 0.0f, 0.0f,
		0.0f, us, 0.0f, 0.0f,
		0.0f, 0.0f, us, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f,
		};
		return Mat(4, 4, CV_32FC1, md).clone();
	}

	inline void Pixel3DSet::transform_point(Mat & m, R3 & inp)
	{
		R3 x = inp;
		inp.x = x.x*m.at<float>(0,0) + x.y*m.at<float>(0,1) + x.z*m.at<float>(0,2) + m.at<float>(0,3);
		inp.y = x.x*m.at<float>(1,0) + x.y*m.at<float>(1,1) + x.z*m.at<float>(1,2) + m.at<float>(1,3);
		inp.z = x.x*m.at<float>(2,0) + x.y*m.at<float>(2,1) + x.z*m.at<float>(2,2) + m.at<float>(2,3);
	}

	void Pixel3DSet::transform_set(Mat & m)
	{
		for(int i = 0; i < size(); i++)
		{
			transform_point(m, points[i]);
		}
	}

	void Pixel3DSet::transform_set(float rx, float ry, float rz, float sc, float tx, float ty, float tz, R3 center)
	{
		Mat m = transformation_matrix(rx,ry,rz,sc,tx,ty,tz, center);
		transform_set(m);
	}

	SIObj Pixel3DSet::siobj(int block_wid, int s)
	{
		SIObj rv(size(), 0);
		float is = 1.0f / (float) s;
		for(int i = 0; i < size(); i++)
		{
			rv._points[i] = (points[i]*is)*(float)block_wid;
		}
		return rv;
	}

	SIObj Pixel3DSet::siobj()
	{
        SIObj rv(size(), 0);
		for(int i = 0; i < size(); i++)
		{
			rv._points[i] = points[i];
		}
		return rv;
	}

	void Pixel3DSet::save_obj(string fname, int objWid)
	{
		SIObj ob = siobj(objWid);
		ob.saveOBJ(fname);
	}

	void Pixel3DSet::save_obj(string fname)
	{
		SIObj ob = siobj();
		ob.saveOBJ(fname);
	}

	float Pixel3DSet::gsNPixel(int index)
	{
		R3 tmp((float)colors[index][0], (float)colors[index][1], (float)colors[index][2]);
		tmp *= (1.0f / 255.0f);
		return ((tmp.x+tmp.y+tmp.z)/3.0f);
	}

	R3 Pixel3DSet::color_as_r3(int index) const
	{
		return R3((float)colors[index][0], (float)colors[index][1], (float)colors[index][2]);
	}

	R3 Pixel3DSet::NPixel(int index)
	{
		R3 tmp((float)colors[index][2], (float)colors[index][1], (float)colors[index][0]);
		tmp *= (1.0f / 255.0f);
		return tmp;
	}

	void Pixel3DSet::white_all_colors()
	{
		for(int i = 0;i < this->colors.size();i++)
		{
			colors[i] = Vec3b(255,255,255);
		}
	}

	void Pixel3DSet::to_Center(int central_location)
	{
		R3 mn, mx;
		min_max_R3(mn, mx);
		R3 central_location_r3((float)central_location, (float)central_location, (float)central_location);
		for(int i = 0; i < size(); i++)
		{
			points[i] = (points[i]-mn) + central_location_r3;
		}
	}

	void Pixel3DSet::to_origin()
	{
		R3 mn, mx;
		min_max_R3(mn, mx);
		for(int i = 0; i < size(); i++)
		{
			points[i] -= mn;
		}
	}

	void Pixel3DSet::to_center_of_MinMax()
	{
		R3 mn, mx;
		min_max_R3(mn, mx);
		R3 t = (mx - mn) * 0.5f;
		mn += t;
		for(int i = 0; i < size(); i++)
		{
			points[i] -= mn;
		}
	}

	void Pixel3DSet::noise(double mean, double stddev)
	{
		srand((unsigned int)time(NULL));
		double sum = 0.0;
		double scalar = 1.0 / RAND_MAX;
		for(int i = 0; i < colors.size(); i++)
		{
			sum = 0.0;
			for(int j = 0; j < 12; j++)
			{
				sum += scalar * (double)rand();
			}
			sum -= 6.0;
			sum *= stddev;
			sum += mean;
			colors[i][0] += (unsigned char)sum;
			colors[i][1] += (unsigned char)sum;
			colors[i][2] += (unsigned char)sum;
		}
	}

	float Pixel3DSet::mean_gs()
	{
		float rv = 0.0f;
		for(int i = 0; i < colors.size(); i++)
		{
			rv += (float)colors[i][0];
			rv += (float)colors[i][1];
			rv += (float)colors[i][2];
		}
		return rv / (float)(3*colors.size());
	}

	void Pixel3DSet::mutate_points(function<void(R3&)> f)
	{
		for(int i = 0; i < points.size(); i++) f(points[i]);
	}

	void Pixel3DSet::basicMinFilter(float distanceThreshold, float colorDifferenceThreshold)
	{
		int count = size();
		bool * keepers = new bool[count];
		//float * _colors = new float[count];

		vector<R3> npoints;
		vector<Vec3b> ncolors;

		for (int i = 0; i < count; i++)
		{
			keepers[i] = true;
			//_colors[i] = gsNPixel(i);
		}

		for (int i = 0; i < count; i++)
		{
			if (!keepers[i]) continue;
			//float icol = _colors[i];
			R3 ip = points[i];
			for (int j = i+1; j < count; j++)
			{
				if (!keepers[j]) continue;
				float dist = ip.dist(points[j]);
				//float cdist = abs(icol - _colors[j]);
				if (dist < distanceThreshold)// && cdist < colorDifferenceThreshold)
				{
					keepers[j] = false;
				}
			}
			npoints.push_back(ip);
			ncolors.push_back(colors[i]);
		}



		delete[] keepers;
		//delete[] _colors;

		*this = Pixel3DSet(npoints, ncolors);
	}

	void Pixel3DSet::unionFilter(Pixel3DSet & o, float distThreshold)
	{
		auto hashFunction = [distThreshold](R3 & p) -> int {
			R3 x = p / (distThreshold * 10.0f);
			return ((int)(x.x)) + ((int)(x.y))*1024 + ((int)(x.z))*1024*1024;
		};
		int count = size();
		int count2 = o.size();

		int hmSize = 5000;
		vector<int> * hashmap = new vector<int>[hmSize];

		//add this.points to hash map
		cout << "hashing...";
		for (int i = 0; i < count; i++)
		{
			int index = hashFunction(points[i]) % hmSize;
			hashmap[index].push_back(i);
		}
		cout << "done\n";
		//for each point in other, if not in hash map -> add this
		int _numKept = 0;
		for (int i = 0; i < count2; i++)
		{
			R3 p = o[i];
			int index = hashFunction(p) % hmSize;
			bool inHm = false;
			for (int j = 0; j < hashmap[index].size(); j++)
			{
				float dist = p.dist(points[hashmap[index][j]]);
				if (dist < distThreshold) //if already a close by point there: do not add
				{
					inHm = true;
					break;
				}
			}
			if (!inHm) //if no similar point : unionize!
			{
				points.push_back(p);
				colors.push_back(o.colors[i]);
				_numKept++;
			}

		}
		delete[] hashmap;
		cout << "keeping " << _numKept << " out of " << o.size() << endl;

	}

	Pixel3DSet Pixel3DSet::openDepthMap(Mat & depthImage, float maxDepth, float cutOff)
	{
		/*
			-= half way from each point
			depth is d * 10,000
			x is (x-hw) * d * 10,000 / 525
			~x is (x-hw) * d * (10,000/525)
		*/


		float minX = (-319.5f) * (10000.0f/525.0f);
		float maxX = ((639.0f-319.5f) * (10000.0f/525.0f)) - minX;
		float minZ = 0.0f;
		float maxZ = 10000.0f;
		float minY = (-239.5f) * (10000.0f/525.0f);
		float maxY = ((479.0f-239.5f) * (10000.0f/525.0f)) - minX;
		float maxi = maxDepth / max(max(maxX, maxY), maxZ);

		vector<R3> points;
		Size depthImageSize = depthImage.size();
		for(int y = 0; y < depthImageSize.height; y++)
		{
			for(int x = 0; x < depthImageSize.width; x++)
			{
				float d = depthImage.at<float>(y, x);
				if(d >= cutOff)
				{
					//d = 1.0f-d;
					//d += 0.3f;
					d *= 10000.0f;
					R3 point(
						((float)x - 319.5f) * (d / 525.0f),
						((float)y - 239.5f) * (d / 525.0f),
						d
					);
					//cout << d << endl;
					point.x = maxDepth - (point.x-minX) * maxi;
					point.y = maxDepth - (point.y-minY) * maxi;
					point.z = maxDepth - (point.z-minZ) * maxi;
					point.x = -point.x;
					points.push_back(point);
				}
			}
		}
		return Pixel3DSet(points);
	}

	Pixel3DSet Pixel3DSet::openDepthMap(Mat & colorImage, Mat & depthImage, float maxDepth, float cutOff, float offset, float range)
	{
		/*
			-= half way from each point
			depth is d * 10,000
			x is (x-hw) * d * 10,000 / 525
			~x is (x-hw) * d * (10,000/525)
		*/


		float minX = (-319.5f) * (10000.0f/525.0f);
		float maxX = ((639.0f-319.5f) * (10000.0f/525.0f)) - minX;
		float minZ = 0.0f;
		float maxZ = 10000.0f;
		float minY = (-239.5f) * (10000.0f/525.0f);
		float maxY = ((479.0f-239.5f) * (10000.0f/525.0f)) - minX;
		float maxi = maxDepth / max(max(maxX, maxY), maxZ);

		vector<R3> points;
		vector<Vec3b> colors;
		Size depthImageSize = depthImage.size();
		for(int y = 0; y < depthImageSize.height; y++)
		{
			for(int x = 0; x < depthImageSize.width; x++)
			{
				float d = depthImage.at<float>(y, x);
				if(d > cutOff )// && x <= (1.0f - cutOff))
				{
					//d = 1.0f-d;
					//d *= range;
					//d += offset;


					d *= 10000.0f;
					R3 point(
						((float)x - 319.5f) * (d / 525.0f),
						((float)y - 239.5f) * (d / 525.0f),
						d
					);
					point.x = maxDepth - (point.x-minX) * maxi;
					point.y = maxDepth - (point.y-minY) * maxi;
					point.z = maxDepth - (point.z-minZ) * maxi;
					point.x = -point.x;
					points.push_back(point);
					Vec3b color = colorImage.at<Vec3b>(y,x);

					colors.push_back(color);
				}
			}
		}
		return Pixel3DSet(points, colors);
	}

	Pixel3DSet Pixel3DSet::DirectProject(Mat & colorImage, Mat depthImage, float size, float threshold)
	{
		vector<R3> points;
		vector<Vec3b> colors;
		R3 point;
		Size depthImageSize = depthImage.size();
		Size hws = depthImageSize / 2;
		for(int y = 0; y < depthImageSize.height; y++)
		{
			for(int x = 0; x < depthImageSize.width; x++)
			{
				float d = depthImage.at<float>(y, x);
				if(d > 0.01f && d < (1.0f-threshold))
				{
					point.x = (x-hws.width)*d; point.y = (y-hws.height)*d;
					//point.x = x, point.y = y;
					point.z = d * size;
					points.push_back(point);
					Vec3b color = colorImage.at<Vec3b>(y,x);
					colors.push_back(color);
				}
			}
		}
		return Pixel3DSet(points, colors);
	}

	//Pixel3DSetWriter

	Pixel3DSetWriter::Pixel3DSetWriter(string dname, int max_frames)
	{
		directory_name = dname;
		maximum_frames = max_frames;
		frames_written_so_far = 0;

		ifstream fi("Pixel3DVideoInfo", ios::in);
		if(!fi.is_open()) ll_error("error in Pixel3DSetWriter::constructor, opening Pixel3DVideoInfo file");
		fi >> full_path;
		full_path += "/" + dname;
		fi.close();
		//cout << "fp: " << full_path << endl;
		clean();
	}

	Pixel3DSetWriter::~Pixel3DSetWriter()
	{
		close();
	}

	void Pixel3DSetWriter::clean()
	{
		system(string("node Pixel3DVideoManage.js " + directory_name).c_str());
	}

	void Pixel3DSetWriter::update_info_file()
	{
		ofstream fi(full_path + "/info", ios::out);
		if(!fi.is_open()) ll_error("error in Pixel3DSetWriter::update_info_file, could not open file");
		fi << frames_written_so_far << endl;
		fi.close();
	}

	void Pixel3DSetWriter::save()
	{
		while(!frames.empty())
		{
			Pix3D p = frames.front(); frames.pop();
			p.compress().save(full_path + "/f" + to_string(frames_written_so_far));
			frames_written_so_far++;
		}
		update_info_file();
	}

	void Pixel3DSetWriter::dump(unsigned int n)
	{
		unsigned int i = 0;
		while(!frames.empty() && i < n)
		{
			Pix3D p = frames.front(); frames.pop();
			p.compress().save(full_path + "/f" + to_string(frames_written_so_far));
			frames_written_so_far++;
			i++;
		}
		update_info_file();
	}

	void Pixel3DSetWriter::close()
	{
		save();
	}

	void Pixel3DSetWriter::push_back(const Pixel3DSet & p)
	{
		frames.push(p.pix3d());
		if(frames.size() > maximum_frames) save();
	}

	void Pixel3DSetWriter::operator << (const Pixel3DSet & p)
	{
		frames.push(p.pix3d());
		if(frames.size() > maximum_frames) save();
	}

	void Pixel3DSetWriter::push_back(const Pix3D & p)
	{
		frames.push(p);
		if(frames.size() > maximum_frames) save();
	}

	void Pixel3DSetWriter::operator << (const Pix3D & p)
	{
		frames.push(p);
		if(frames.size() > maximum_frames) save();
	}

	unsigned int Pixel3DSetWriter::size()
	{
		return (unsigned int)frames_written_so_far + (unsigned int)frames.size();
	}


	//capture object

	CapturePixel3DSet::CapturePixel3DSet(string dname, int buffer_size_in)
	{
		directory_name = dname;
		buffer_size = buffer_size_in;
		index = 0;
		num_frames_read = 0;
		ifstream fi("Pixel3DVideoInfo", ios::in);
		if(!fi.is_open()) ll_error("error in CapturePixel3DSet::constructor, opening Pixel3DVideoInfo file");
		fi >> full_path;
		full_path += "/" + dname;
		fi.close();
		reset();
	}
	CapturePixel3DSet::CapturePixel3DSet(int zero)
	{

	}
	CapturePixel3DSet CapturePixel3DSet::openCustom(string full_path_in, string dname, int buffer_size_in)
	{
        CapturePixel3DSet ret = 0;
		ret.directory_name = dname;
		ret.buffer_size = buffer_size_in;
		ret.index = 0;
		ret.num_frames_read = 0;
		ret.full_path = full_path_in;
		ret.full_path += "/" + dname;
		ret.reset();
		return ret;
	}

	CapturePixel3DSet::~CapturePixel3DSet()
	{
		while(!frames.empty()) frames.pop();
	}

	void CapturePixel3DSet::reset()
	{
        cout << full_path + "/info" << endl;
		ifstream fi(full_path + "/info", ios::in);
		if(!fi.is_open()) ll_error("error in CapturePixel3DSet::update_info_file, could not open file");
		fi >> total_number_of_frames;
		fi.close();
		index = 0;
		num_frames_read = 0;
		while(!frames.empty()) frames.pop();
		load();
	}

	void CapturePixel3DSet::load()
	{
		while(index < total_number_of_frames && frames.size() < buffer_size) load_single_frame();
	}

	bool CapturePixel3DSet::load_single_frame()
	{
		if(index >= total_number_of_frames) return false;
		Pix3DC _compressedVersion(full_path + string("/f") + to_string(index));
		Pix3D frame(_compressedVersion);
		frames.push(frame);
		index++;
		return true;
	}

	bool CapturePixel3DSet::read_frame(Pixel3DSet & p, unsigned int index_in)
	{
		if(index_in >= total_number_of_frames) return false;
		p.open(full_path + string("/f") + to_string(index_in));
		return true;
	}

	bool CapturePixel3DSet::read_frame(Pix3D & p, unsigned int index_in)
	{
		if(index_in >= total_number_of_frames) return false;
		Pix3DC tmp(full_path + string("/f") + to_string(index_in));
		p = Pix3D(tmp);
		return true;
	}

	int CapturePixel3DSet::size()
	{
		return total_number_of_frames;
	}

	bool CapturePixel3DSet::read(Pixel3DSet & p)
	{
		if(frames.empty()) load();
		if(frames.empty()) return false;
		p = Pixel3DSet(frames.front());
		num_frames_read++;
		frames.pop();
		return true;
	}

	bool CapturePixel3DSet::read(Pix3D & p)
	{
		if(frames.empty()) load();
		if(frames.empty()) return false;
		p = frames.front();
		num_frames_read++;
		frames.pop();
		return true;
	}

	bool CapturePixel3DSet::front(Pixel3DSet & p)
	{
		if(frames.empty()) load();
		if(frames.empty()) return false;
		p = frames.front();
		return true;
	}

	Pixel3DSet CapturePixel3DSet::reduce_video_frames(string directory_name, int size_)
	{
		CapturePixel3DSet cap = directory_name;
		R3 minimum, maximum;
		CapturePixel3DSet::minMaxLoc(directory_name, minimum, maximum);

		float sizef = static_cast<float>(size_);
		int _s2 = size_ * size_;
		int _s3 = _s2 * size_;

		vector<Vec3b> colList(_s3, Vec3b(0,0,0));
		vector<int> counts(_s3, 0);
		maximum -= minimum;
		float maxScalar = maximum.max();

		float sizef_div_maxScalar = sizef / maxScalar;

		if(maxScalar == 0.0f) ll_error("eventual division by 0 in CapturePixel3DSet::reduce_video_frames");

		Pixel3DSet frame;

		R3 p;

		while(cap.read(frame))
		{
			for(int i = 0; i < frame.size(); i++)
			{
				p = frame.points[i];
				p -= minimum;
				p *= sizef_div_maxScalar;
				Point3i index(static_cast<int>(round(p.x)), static_cast<int>(round(p.y)), static_cast<int>(round(p.z)));
				int iindex = index.z * _s2 + index.y * size_ + index.x;
				if(index.x<0 || index.y<0 || index.z<0 || index.x>=size_ || index.y>=size_ || index.z>=size_) continue;
				int currentCount = counts[iindex];
				double currentScalar = currentCount / static_cast<double>(currentCount+1);
				double nextScalar = 1.0 / static_cast<double>(currentCount+1);
				colList[iindex] = ll_color_blend(colList[iindex],frame.colors[i], currentScalar, nextScalar);
				counts[iindex] += 1;
			}
		}

		Pixel3DSet rv;
		float maxScalar_divide_sizef = maxScalar / sizef;


		for(int z = 0; z < size_; z++)
		{
			for(int y = 0; y < size_; y++)
			{
				for(int x = 0; x < size_; x++)
				{
					int iindex = z * _s2 + y * size_ + x;
					if(x<0 || y<0 || z<0 || x>=size_ || y>=size_ || z>=size_) continue;

					if(counts[iindex] <= 0) continue;
					//set p to [x,y,z]
					p.x = static_cast<float>(x);
					p.y = static_cast<float>(y);
					p.z = static_cast<float>(z);

					p *= maxScalar_divide_sizef;
					p += minimum;
					Vec3b color = colList[iindex];

					rv.points.push_back(p);

					rv.colors.push_back(color);
				}
			}
		}
		return rv;
	}

	Pixel3DSet CapturePixel3DSet::collect_video_frames(string directory_name, int size_)
	{
		CapturePixel3DSet cap = directory_name;
		R3 minimum, maximum;
		CapturePixel3DSet::minMaxLoc(directory_name, minimum, maximum);


		Pixel3DSet frame, db;

		R3 p;

		while(cap.read(frame))
		{
			db += frame;
		}


		return db;
	}

	void CapturePixel3DSet::minMaxLoc(string directory_name, R3 & minimum, R3 & maximum)
	{
		CapturePixel3DSet cap = directory_name;
		Pixel3DSet frame;
		if(cap.total_number_of_frames <= 0) ll_error("error in CapturePixel3DSet::minMaxLoc(), no frames at location");
		cap.front(frame);
		if(frame.points.size() <= 0) ll_error("error in CapturePixel3DSet::minMaxLoc(), first frame has no points");
		while(cap.read(frame))
		{
			if(frame.points.size() <= 0) continue;
			for(int i = 0; i < frame.size(); i++)
			{
				minimum.min(frame[i]);
				maximum.max(frame[i]);
			}
		}
	}



	//Pix3DC
	Pix3DC::Pix3DC(unsigned char * dataIn, int len)
	{
		setAs();
		clone(dataIn, len);
	}
	Pix3DC::Pix3DC(string fn)
	{
		setAs();
		open(fn);
	}
	Pix3DC::Pix3DC(const Pix3DC & p)
	{
		setAs();
		clone(p);
	}
	Pix3DC::~Pix3DC()
	{
		free();
	}
	Pix3DC & Pix3DC::operator = (const Pix3DC & p)
	{
		clone(p);
		return *this;
	}
	void Pix3DC::clone(const Pix3DC & p)
	{
		if(&p == this) return;
		free();
		clone(p.data, p.length);
	}
	void Pix3DC::setAs(unsigned char * dataIn, int len)
	{
		data = dataIn;
		length = len;
	}
	bool Pix3DC::empty()
	{
		if(length == 0 || data == NULL) return true;
		return false;
	}
	void Pix3DC::free()
	{
		if(!empty())
		{
			delete [] data;
			length = 0;
			data = NULL;
		}
	}
	Pix3DC Pix3DC::clone()
	{
		Pix3DC rv;
		rv.clone(data, length);
		return rv;
	}
	void Pix3DC::clone(unsigned char * dataIn, int len)
	{
		free();
		if(len == 0 || dataIn == NULL) return;
		data = new unsigned char[len];
		length = len;
		memcpy(data, dataIn, length);
	}
	void Pix3DC::clone(vector<unsigned char> * dataIn)
	{
		free();
		if(dataIn == NULL || dataIn->size() == 0) return;
		length = (int)dataIn->size();
		data = new unsigned char[length];
		std::copy(dataIn->begin(), dataIn->end(), data);
	}
	void Pix3DC::save(string fn)
	{
		int bytesPerWrite = 100, index = 0;
		ofstream fi(fn.c_str(), ios::binary);
		while(index < length)
		{
			int nextIndex = index + bytesPerWrite;
			nextIndex = (nextIndex > length) ? length : nextIndex;
			fi.write((const char *)&data[index], (nextIndex - index));
			index += (nextIndex-index);
		}
		fi.close();
	}
	void Pix3DC::open(string fn)
	{
		ifstream fi(fn.c_str(), ios::binary | ios::ate);
		streamsize size = fi.tellg();
		length = size;
		data = new unsigned char[length];

		fi.seekg(0, ios::beg);
		int index = 0, bytesPerRead = 2000;

		while(index < length)
		{

			//read in that many bytes, increment index by gcount (the number of bytes read
			fi.read((char *) data + index, bytesPerRead);
			int numBytesRead = fi.gcount();
			index += numBytesRead;
		}
		fi.close();
	}

	//Pix3D
	Pix3D::Pix3D()
	{
		setAs(); //set to default values
	}
	Pix3D::Pix3D(Pix3DC & p)
	{
		setAs();
		decompress(p);
	}
	Pix3D::Pix3D(int count, R3 * points, Vec3b * colors, bool * validDepth)
	{
		this->count = count;
		this->points = new R3[count];
		this->colors = new Vec3b[count];
		this->validDepth = new bool[count];
		this->type = ImageType;
		memcpy(this->points, points, sizeof(R3) * count);
		memcpy(this->colors, colors, sizeof(Vec3b) * count);
		memcpy(this->validDepth, validDepth, sizeof(bool) * count);
	}
	Pix3D::Pix3D(std::vector<ll_R3::R3> pointsIn, std::vector<Vec3b> colorsIn)
	{
		assert(pointsIn.size() == colorsIn.size());
		count = (unsigned int) pointsIn.size();
		type = Pix3D::NonImageType;
		points = new R3[count];
		colors = new Vec3b[count];
		for(unsigned int i = 0; i < count; i++)
		{
			points[i] = pointsIn[i];
			colors[i] = colorsIn[i];
		}
		validDepth = NULL;
	}
	Pix3D::Pix3D(const Pix3D & p)
	{
		setAs();
		clone(p);
	}
	Pix3D & Pix3D::operator = (const Pix3D & p)
	{
		clone(p);
		return *this;
	}
	Pix3D::~Pix3D()
	{
		free();
	}
	void Pix3D::setAs(ll_R3::R3 * points, Vec3b * colors, bool * validDepth, int count)
	{
		this->points = points;
		this->colors = colors;
		this->validDepth = validDepth;
		this->count = count;
		if(count == 0) type = Pix3D::NoDataType;
		else if(this->validDepth != NULL) type = Pix3D::ImageType;
		else type = Pix3D::NonImageType;
	}
	void Pix3D::clone(const Pix3D & p)
	{
		if(this == &p) return;
		free();
		if(p.empty()) return;
		type = p.type;
		count = p.count;
		points = new R3[count];
		colors = new Vec3b[count];
		if(hasVd())
			validDepth = new bool[count];
		else validDepth = NULL;
		bool _vd = hasVd();
		for(unsigned int i = 0; i < count; i++)
		{
			if(_vd) validDepth[i] = p.validDepth[i];
			points[i] = p.points[i];
			colors[i] = p.colors[i];
		}
	}
	void Pix3D::minMaxR3(ll_R3::R3 & minimum, ll_R3::R3 & maximum) const
	{
		minimum = R3(FLT_MAX, FLT_MAX, FLT_MAX);
		maximum = R3(FLT_MIN, FLT_MIN, FLT_MIN);
		for(unsigned int i = 0; i < count; i++)
		{
			minimum.min(points[i]);
			maximum.max(points[i]);
		}
	}
	unsigned short Pix3D::compress(float v, float minimum, float maximum) const
	{
		v -= minimum;
		v *= ((float)USHRT_MAX) / maximum;
		return (unsigned short) v;
	}
	float Pix3D::decompress(unsigned short v, float minimum, float maximum)
	{
		float rv = (float) v;
		rv *= maximum / (float)USHRT_MAX;
		rv += minimum;
		return rv;
	}
	Pix3DC Pix3D::compressImageType() const
	{
		Pix3DC rv;
		if(!hasVd()) return rv;
		BIT_MANIPULATOR::BitWriter vfi;
		vfi.addBit(1); //type is image type

		//pixel count
		vfi.addBytes(sizeof(unsigned int), (unsigned char *)(&count));

		//need to: find min and max or each point coord
		//save mins and maxs, then scale them as unsigned shorts
		R3 minimum, maximum;
		minMaxR3(minimum, maximum);
		vfi.addBytes(sizeof(float), (unsigned char *) &minimum.x);
		vfi.addBytes(sizeof(float), (unsigned char *) &minimum.y);
		vfi.addBytes(sizeof(float), (unsigned char *) &minimum.z);

		vfi.addBytes(sizeof(float), (unsigned char *) &maximum.x);
		vfi.addBytes(sizeof(float), (unsigned char *) &maximum.y);
		vfi.addBytes(sizeof(float), (unsigned char *) &maximum.z);


		for(unsigned int i = 0; i < count; i++)
		{
			//add valid depth bit
			int hasVD = validDepth[i] == 0x00 ? 0 : 1;
			vfi.addBit(hasVD);
			//save color
			vfi.addByte(colors[i][0]);
			vfi.addByte(colors[i][1]);
			vfi.addByte(colors[i][2]);
			//if depth valid: save point
			if(hasVD == 1)
			{
				unsigned short x = compress(points[i].x, minimum.x, maximum.x);
				unsigned short y = compress(points[i].y, minimum.y, maximum.y);
				unsigned short z = compress(points[i].z, minimum.z, maximum.z);
				vfi.addBytes(sizeof(unsigned short), (unsigned char *)&x);
				vfi.addBytes(sizeof(unsigned short), (unsigned char *)&y);
				vfi.addBytes(sizeof(unsigned short), (unsigned char *)&z);
			}
		}
		rv.clone(vfi.data);
		return rv;
	}
	Pix3DC Pix3D::compressNonImageType() const
	{
		Pix3DC rv;
		if(type != NonImageType) return rv;
		BIT_MANIPULATOR::BitWriter vfi;
		vfi.addBit(0); //type is non image type

		//pixel count
		vfi.addBytes(sizeof(unsigned int), (unsigned char *)(&count));

		//need to: find min and max or each point coord
		//save mins and maxs, then scale them as unsigned shorts
		R3 minimum, maximum;
		minMaxR3(minimum, maximum);
		vfi.addBytes(sizeof(float), (unsigned char *) &minimum.x);
		vfi.addBytes(sizeof(float), (unsigned char *) &minimum.y);
		vfi.addBytes(sizeof(float), (unsigned char *) &minimum.z);

		vfi.addBytes(sizeof(float), (unsigned char *) &maximum.x);
		vfi.addBytes(sizeof(float), (unsigned char *) &maximum.y);
		vfi.addBytes(sizeof(float), (unsigned char *) &maximum.z);

		//cout << count << " " << type << " " << minimum << ", " << maximum << endl;

		for(unsigned int i = 0; i < count; i++)
		{
			//save color
			vfi.addByte(colors[i][0]);
			vfi.addByte(colors[i][1]);
			vfi.addByte(colors[i][2]);
			//save point
			unsigned short x = compress(points[i].x, minimum.x, maximum.x);
			unsigned short y = compress(points[i].y, minimum.y, maximum.y);
			unsigned short z = compress(points[i].z, minimum.z, maximum.z);
			vfi.addBytes(sizeof(unsigned short), (unsigned char *)&x);
			vfi.addBytes(sizeof(unsigned short), (unsigned char *)&y);
			vfi.addBytes(sizeof(unsigned short), (unsigned char *)&z);
		}
		rv.clone(vfi.data);
		return rv;
	}
	void Pix3D::freeImageType()
	{
		if(empty()) return;
		delete [] validDepth;
		delete [] points;
		delete [] colors;
		setAs();
	}
	void Pix3D::freeNonImageType()
	{
		if(empty()) return;
		delete [] points;
		delete [] colors;
		setAs();
	}
	void Pix3D::free()
	{
		if(empty()) return;
		else if(hasVd()) freeImageType();
		else freeNonImageType();
	}
	Pix3DC Pix3D::compress() const
	{
		Pix3DC rv;
		rv.setAs();
		if(empty()) return rv;
		if(type == ImageType) return compressImageType();
		else return compressNonImageType();
	}
	void Pix3D::decompress(Pix3DC & d)
	{
		free();


		BIT_MANIPULATOR::BitReader vfi(d.data, d.length);
		type = vfi.readBit(); //type is image type

		//pixel count
		vfi.readBytes(sizeof(unsigned int), (unsigned char *)(&count));

		points = new R3[count];
		colors = new Vec3b[count];
		if(type == Pix3D::ImageType) validDepth = new bool[count];
		else validDepth = NULL;

		//need to: find min and max or each point coord
		//save mins and maxs, then scale them as unsigned shorts
		R3 minimum, maximum;
		vfi.readBytes(sizeof(float), (unsigned char *) &minimum.x);
		vfi.readBytes(sizeof(float), (unsigned char *) &minimum.y);
		vfi.readBytes(sizeof(float), (unsigned char *) &minimum.z);

		vfi.readBytes(sizeof(float), (unsigned char *) &maximum.x);
		vfi.readBytes(sizeof(float), (unsigned char *) &maximum.y);
		vfi.readBytes(sizeof(float), (unsigned char *) &maximum.z);

		//cout << count << " " << type << " " << minimum << ", " << maximum << endl;

		for(unsigned int i = 0; i < count; i++)
		{
			//add valid depth bit
			bool readData = true;
			if(type == Pix3D::ImageType)
			{
				readData = vfi.readBit() == 0x01;
				validDepth[i] = readData;
			}
			//save color
			colors[i][0] = vfi.readByte();
			colors[i][1] = vfi.readByte();
			colors[i][2] = vfi.readByte();
			//if depth valid: save point
			if(readData)
			{
				unsigned short x, y, z;
				vfi.readBytes(sizeof(unsigned short), (unsigned char *)&x);
				vfi.readBytes(sizeof(unsigned short), (unsigned char *)&y);
				vfi.readBytes(sizeof(unsigned short), (unsigned char *)&z);
				points[i].x = decompress(x, minimum.x, maximum.x);
				points[i].y = decompress(y, minimum.y, maximum.y);
				points[i].z = decompress(z, minimum.z, maximum.z);
			}
		}
	}
	bool Pix3D::hasVd() const
	{
		return type == Pix3D::ImageType;
	}
	bool Pix3D::empty() const
	{
		return type == NoDataType;
	}
	bool Pix3D::colorImage(Mat & m)
	{
		if(type != ImageType) return false;
		m = Mat(480, 640, CV_8UC3, colors);
		return true;
	}
	bool Pix3D::depthImage(Mat & m)
	{
		if(type != ImageType) return false;
		m = Mat::zeros(Size(640, 480), CV_32FC1);
		float * ptr = (float *) m.data;
		for(unsigned int i = 0; i < count; i++, ptr++) *ptr = points[i].z;
		m /= 384.0f;
		return true;
	}
	bool Pix3D::vdImage(Mat & m)
	{
		if(type != ImageType) return false;
		m = Mat::zeros(Size(640, 480), CV_8UC1);
		unsigned char * ptr = (unsigned char *) m.data;
		for(unsigned int i = 0; i < count; i++, ptr++) *ptr = validDepth[i] ? 0xFF : 0x00;
		return true;
	}
	Pix3D Pix3D::withoutWarping(int margin)
	{
		if(!hasVd()) return *this;

		vector<R3> pts;
		vector<Vec3b> cols;
		for(int y = margin; y < 640 - margin; y++)
		{
			for(int x = margin; x < 480 - margin; x++)
			{
				int index = y * 480 + x;
				pts.push_back(points[index]);
				cols.push_back(colors[index]);
			}
		}
		return Pix3D(pts, cols);
	}


    //KD-Tree Implementation
    LKDNode::LKDNode()
    {
        axis = 0;
        value = 0.0f;
        _p = R3();
        left = right = NULL;
    }
    LKDNode::LKDNode(int ax, float val, R3 & pIn, LKDNode * l, LKDNode * r)
    {
        axis = ax;
        value = val;
        _p = pIn;
        left = l;
        right = r;
    }
    LKDNode::LKDNode(const LKDNode & input)
    {
        copyIn(input);
    }
    LKDNode & LKDNode::operator = (const LKDNode & input)
    {
        if(&input == this) return *this;
        copyIn(input);
        return *this;
    }
    void LKDNode::copyIn(const LKDNode & input)
    {
        axis = input.axis;
        value = input.value;
        _p = input.value;
        left = input.left;
        right = input.right;
        indices = input.indices;
    }

    void LKDNode::free()
    {
        if(left){ left->free(); delete left; }
        if(right){ right->free(); delete right; }
    }

    void LKDNode::init(Pixel3DSet & pset)
    {
        indices.clear(); for(int i = 0; i < pset.size(); i++) indices.push_back(i);
    }
    void LKDNode::split(int chosenAxis, Pixel3DSet & pset)
    {
        if(indices.size() == 1) return;
        axis = chosenAxis;

        //set value and _p here
        vector<R3*> tmp; for(int i = 0; i < indices.size(); i++) tmp.push_back(&pset[indices[i]]);
        int ax = axis;
        sort(tmp.begin(), tmp.end(), [ax](const R3 * p1, const R3 * p2)->bool{
            R3 t1 = *p1, t2 = *p2;
            return t1[ax] < t2[ax];
        });

        _p = *tmp[tmp.size()/2];
        value = _p[axis];

        vector<int> ls, rs;
        for(int i = 0; i < indices.size(); i++)
        {
            R3 p = pset[indices[i]];
            if(p[axis] <= value) ls.push_back(indices[i]);
            else rs.push_back(indices[i]);
        }

        if(ls.size() > 1 && rs.size() > 1)
        {
            left = new LKDNode();
            right = new LKDNode();
            left->indices = ls;
            right->indices = rs;

            indices.clear();
        }
    }

    bool LKDNode::isLeaf()
    {
        return left == NULL && right == NULL;
    }

    void LKDNode::split(Pixel3DSet & pset, int startAxis, int maxDepth)
    {
        if(maxDepth <= 0) return;
        split(startAxis, pset);
        if(!isLeaf())
        {
            int nextAxis = (startAxis + 1) % 3;
            left->split(pset, nextAxis, maxDepth - 1);
            right->split(pset, nextAxis, maxDepth - 1);
        }
    }

    void LKDNode::forEach(std::function<void(LKDNode *)> f)
    {
        stack<LKDNode *> s;
        s.push(this);

        while(!s.empty())
        {
            LKDNode * c = s.top(); s.pop();
            if(c == NULL) continue;
            f(c);
            if(!c->isLeaf())
            {
                s.push(c->left);
                s.push(c->right);
            }
        }

    }

    double LKDNode::averageLeafSize()
    {
        double avg = 0.0, c = 0.0;
        forEach([&avg, &c](LKDNode * ptr)->void{
            if(ptr->isLeaf())
            {
                avg += (double)ptr->indices.size();
                c += 1.0;
            }
        });
        if(c == 0.0) return 0.0;
        return avg / c;
    }

    bool LKDNode::NN(Pixel3DSet & pset, R3 & q, int & index, R3 & w)
    {
        float dist = DBL_MAX;
        R3 p = _p;
        NN(pset, q, index, p, dist);
        w = pset[index];
        return true;
    }
    void LKDNode::NN(Pixel3DSet & pset, R3 & q, int & index, R3 & p, float & w)
    {
        if(isLeaf())
        {
            for(int i = 0; i < indices.size(); i++)
            {
                R3 pnt = pset[indices[i]];
                float _w = pnt.dist(q);
                if(_w < w)
                {
                    w = _w;
                    p = _p;
                    index = indices[i];
                    //isFound = true;
                }
            }
        }else
        {
            if(q[axis] <= value) //search left first
            {
                //if closest distance is less than distance from q to the pivot
                if(q[axis]-w <= value) left->NN(pset, q, index, p, w);
                if(q[axis]+w > value) right->NN(pset, q, index, p, w);
            }else //search right first
            {
                if(q[axis]+w > value) right->NN(pset, q, index, p, w);
                if(q[axis]-w <= value) left->NN(pset, q, index, p, w);;
            }
        }
    }
}
