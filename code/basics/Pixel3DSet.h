#pragma once
#include "locv3.h"
#include <vector>
#include <queue>
#include <string>
#include "R3.h"
#include "SIObj.h"
#include <functional>

//depends on ll_r3, ll_siobj and opencv3 (locv3), bit-reader/writer

//forward declaration:
namespace ll_pix3d
{
	class Pix3DC; class Pix3D; class Pixel3DSet; class Pixel3DSetWriter; class CapturePixel3DSet;
}


namespace ll_pix3d
{

	//a basic container for an array of binary data
	class Pix3DC
	{
	public:
		//DATA:
		unsigned char * data; //raw compressed binary data
		int length; //the length of the data

		Pix3DC(unsigned char * dataIn = NULL, int len = 0);
		Pix3DC(std::string fn); //opens a file based on fn
		Pix3DC(const Pix3DC & p);
		Pix3DC & operator = (const Pix3DC & p);
		~Pix3DC();
		void clone(const Pix3DC & p); //copies p as this (deep copy)

		void setAs(unsigned char * dataIn = NULL, int len = 0); // sets the data to these valuds in a raw way
		bool empty(); //returns true if there is no data owned in the heap
		void free(); //frees memory
		Pix3DC clone(); //returns a clone of this object
		void clone(unsigned char * dataIn, int len); //performs a deep copy using a raw c array
		void clone(std::vector<unsigned char> * dataIn); //performs a deep copy using c++ vector array
		void save(std::string fn); //saves to a file
		void open(std::string fn); //opens from a file

	};

	//a new 3d format for pix3dc data
	//all sizes 640 x 480 unless type is NonImage
	class Pix3D
	{
	public:
		//DATA:
		ll_R3::R3 * points; //3d points making up a point cloud
		cv::Vec3b * colors; // colors
		bool * validDepth; //valid depth : used if type is imagetype
		unsigned int count; //the length of the data vector [(color, point, vd : if used), (color, point, vd : if used), ... ()]
		unsigned char type; //the type of data: imagetype, nonimagetype and nodatatype

		Pix3D();
		Pix3D(Pix3DC & p);
		Pix3D(int count, ll_R3::R3 * points, cv::Vec3b * colors, bool * validDepths);
		Pix3D(std::vector<ll_R3::R3> points, std::vector<cv::Vec3b> colors);
		~Pix3D();
		Pix3D(const Pix3D & p);
		void setAs(ll_R3::R3 * points = NULL, cv::Vec3b * colors = NULL, bool * validDepth = NULL, int count = 0);
		Pix3D & operator = (const Pix3D & p);
		void clone(const Pix3D & p);
		Pix3DC compressImageType() const;
		Pix3DC compressNonImageType() const;
		void freeImageType();
		void freeNonImageType();
		void free();
		Pix3DC compress() const;
		void decompress(Pix3DC & d);
		void minMaxR3(ll_R3::R3 & minimum, ll_R3::R3 & maximum) const;
		unsigned short compress(float v, float minimum, float maximum) const;
		float decompress(unsigned short v, float minimum, float maximum);
		bool hasVd() const;
		bool empty() const;
		bool colorImage(cv::Mat & m);
		bool depthImage(cv::Mat & m);
		bool vdImage(cv::Mat & m);

		Pix3D withoutWarping(int margin = 20);

		const static unsigned char NonImageType = 0x00;
		const static unsigned char ImageType = 0x01;
		const static unsigned char NoDataType = 0x02;
	};


	class Pixel3DSet
	{
	public:
		std::vector<ll_R3::R3> points;
		std::vector<cv::Vec3b> colors;
		Pixel3DSet();

		Pixel3DSet(const Pixel3DSet & a);
		Pixel3DSet & operator = (const Pixel3DSet & a);
		Pixel3DSet(const std::vector<ll_R3::R3> & pointz, const std::vector<cv::Vec3b> colorz);
		Pixel3DSet(const std::vector<ll_R3::R3> & pointz, cv::Vec3b on_color = cv::Vec3b(255, 255, 255));
		Pixel3DSet(const std::vector<cv::Vec3f> & pointz, const std::vector<cv::Vec3b> colorz);
		Pixel3DSet(ll_R3::R3 * l, cv::Vec3b * color, int n);
		Pixel3DSet(const Pix3D & p);

		//normalization functions
		void normalize(int s = 384); //was n() / n(int s). Just normalizes all the points
		void round_points(); //rounds all the points, was n2()

		//i/o
		void save(std::string fn);
		void open(std::string fn);
		Pix3D pix3d() const;


		void clear();
		void copyFrom(const Pixel3DSet & a);
		Pixel3DSet clone();
		inline int size();
		inline ll_R3::R3 & operator [] (int index);
		void push_back(ll_R3::R3 & a, cv::Vec3b c);
		void UNION(Pixel3DSet & a);
		Pixel3DSet operator + (Pixel3DSet & a);
		Pixel3DSet & operator += (Pixel3DSet & a);

		ll_R3::R3 getAvg();
		void reduce(int size_);
		void min_max_R3(ll_R3::R3 & mn, ll_R3::R3 & mx);

		//static functionality for matrix generation
		static cv::Mat rotation_matrix_center(float rx, float ry, float rz, ll_R3::R3 center);
		static cv::Mat scale_matrix_center(float sc, ll_R3::R3 center);
		static cv::Mat transformation_matrix(float rx, float ry, float rz, float sc, float tx, float ty, float tz, ll_R3::R3 center);
		static cv::Mat scale_matrix(float us);
		static inline void transform_point(cv::Mat & m, ll_R3::R3 & inp);
		static cv::Mat translation_matrix(float x, float y, float z);
		static cv::Mat rotation_matrix_x(float angle);
		static cv::Mat rotation_matrix_y(float angle);
		static cv::Mat rotation_matrix_z(float angle);
		static cv::Mat rotation_matrix(float angleX, float angleY, float angleZ);
		static cv::Mat scale_matrix(float sx, float sy, float sz);
		static cv::Mat center_origin_matrix_inv(ll_R3::R3 center);
		static cv::Mat center_origin_matrix(ll_R3::R3 center);

		void transform_set(cv::Mat & m);
		void transform_set(float rx, float ry, float rz, float sc, float tx, float ty, float tz, ll_R3::R3 center);
		ll_siobj::SIObj siobj(int block_wid, int s = 384);
		ll_siobj::SIObj siobj();
		void save_obj(std::string fname, int objWid);
		void save_obj(std::string fname);
		float gsNPixel(int index);
		ll_R3::R3 color_as_r3(int index) const;
		ll_R3::R3 NPixel(int index);
		void white_all_colors();
		void to_Center(int central_location = 0);
		void to_origin();
		void to_center_of_MinMax();
		void noise(double mean, double stddev);
		void positional_noise(double scalar);
		float mean_gs();

		void mutate_points(std::function<void(ll_R3::R3&)> f);

		void basicMinFilter(float distanceThreshold = 0.6f, float colorDifferenceThreshold = 0.6f);
		void unionFilter(Pixel3DSet & o, float distThreshold = 0.6f);

		static Pixel3DSet openDepthMap(cv::Mat & depthImage, float maxDepth, float cutOff = 0.1f);
		static Pixel3DSet openDepthMap(cv::Mat & colorImage, cv::Mat & depthImage, float maxDepth, float cutOff = 0.1f, float offset = 0.4f, float range = 0.7f);
		static Pixel3DSet DirectProject(cv::Mat & colorImage, cv::Mat depthImage, float size, float threshold);


	};


	//Pixel3DSetWriter
	class Pixel3DSetWriter
	{
	public:
		std::queue<Pix3D> frames;
		std::string directory_name;
		std::string full_path;
		int maximum_frames;
		int frames_written_so_far;

		Pixel3DSetWriter(std::string directory_name = "default", int max_frames = 50);
		~Pixel3DSetWriter();

		//clean -> performs the Pixel3DSetManage node.js process clean. is called in the constructor
		void clean();

		//saves the current queue-set to the file system
		void save();

		//updates the info file
		void update_info_file();

		void dump(unsigned int n); // dumps n in the queue to the file system, if n>frames.size(), then is equivelent to save

		//adds a file to the queue, if there are too many files, as specified by maximum_frames, save is called
		void push_back(const Pixel3DSet & p);

		//adds a file to the queue, if there are too many files, as specified by maximum_frames, save is called
		void operator << (const Pixel3DSet & p);

		//adds a file to the queue, if there are too many files, as specified by maximum_frames, save is called
		void push_back(const Pix3D & p);

		//adds a file to the queue, if there are too many files, as specified by maximum_frames, save is called
		void operator << (const Pix3D & p);

		void close();

		unsigned int size(); //outputs the total number of frames, eg. the frames written so far plus the number in the buffer


	};


	//CapturePixel3DSet
	class CapturePixel3DSet
	{
	public:
		std::queue<Pix3D> frames;
		unsigned int index;
		unsigned int total_number_of_frames;
		unsigned int buffer_size;
		unsigned int num_frames_read; //records the number of successful CapturePixel3DSet.read() functions were called
		std::string directory_name;
		std::string full_path;
		CapturePixel3DSet(std::string directory_name = "default", int buffer_size = 30);
		CapturePixel3DSet(int zero); //null created CapturePixel3DSet
		static CapturePixel3DSet openCustom(std::string fullPathName, std::string directory_name, int buffer_size = 30);
		~CapturePixel3DSet();

		void reset(); //loads in some frames, gets the number of frames
		void load(); //loads in frames unti index == total_number_of_frames or frames.size() == buffer_size
		bool load_single_frame(); //loads a single frame, returns true/false if process was completed
		bool read(Pixel3DSet & p); // returns true or false whether the read was successful
		bool read(Pix3D & p); // returns true or false whether the read was successful
		bool front(Pixel3DSet & p); // similar to read but keeps a copy of the frame
		int size(); //returns the total number of frames possible to read
		bool read_frame(Pixel3DSet & p, unsigned int index_in); //reads a specific frame, but does not check if it is buffered
		bool read_frame(Pix3D & p, unsigned int index_in); //reads a specific frame, but does not check if it is buffered

		static Pixel3DSet reduce_video_frames(std::string directory_name, int size_);
		static Pixel3DSet collect_video_frames(std::string directory_name, int size_);
		static void minMaxLoc(std::string directory_name, ll_R3::R3 & minimum, ll_R3::R3 & maximum);
	};


	class LKDNode
	{
	public:
        int axis; //0-x,1-y,2-z
        float value; //the value of axis to split on
        LKDNode * left, * right;
        ll_R3::R3 _p;
        std::vector<int> indices;

        LKDNode();
        LKDNode(int ax, float val, ll_R3::R3 & pIn, LKDNode * l = NULL, LKDNode * r = NULL);
        LKDNode(const LKDNode & input);
        LKDNode & operator = (const LKDNode & input);
        void copyIn(const LKDNode & input);
        void free();

        void init(Pixel3DSet & pset);
        void split(int chosenAxis, Pixel3DSet & pset);
        bool isLeaf();

        void split(Pixel3DSet & pset, int startAxis = 0, int maxDepth = 15);
        void forEach(std::function<void(LKDNode *)> f);
        double averageLeafSize();

        bool NN(Pixel3DSet & pset, ll_R3::R3 & q, int & index, ll_R3::R3 & w);
        void NN(Pixel3DSet & pset, ll_R3::R3 & q, int & index, ll_R3::R3 & p, float & w); //q=query point, w=closest point

	};

}
