/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description: 
		Contains some objects and algorithms which are are written by me, which perform computer vision algorithms 

	depends on: opencv 3.0 and ll_R3, Pixel3DSet
*/
#pragma once
#include <stack>
#include "locv3.h"
#include "R3.h"
#include "Pixel3DSet.h"

//requires only opencv 3.0
//forward declaration
namespace ll_algorithms
{
	namespace ll_pca_3d { class LPCA; }
	namespace ll_ann { class LNN_Set; }
}

namespace ll_algorithms
{

	namespace ll_semiglobal_matcher
	{
		void disparity_errors(Mat & errors, int row, Mat & left_image, Mat & right_image, int ndisparities, int sad_size);
		void optimize(Mat & errors, Mat & previous_positions, Mat & running_costs, Mat & depth_out, Mat & errors_out, double smoothness, Point2i & shift_amount);
		void compute(Mat & im1, Mat & im2, Mat & depth, Mat & errors, int ndisparities, int sad_size, double smoothness, bool fliplr = false);
		Mat ll_sgdisparity(Mat & im1, Mat & im2, int ndisparities, int sad_size, double smoothness);
	}

	namespace ll_pca_3d
	{
		class LPCA
		{
		public:
			ll_R3::R3 mean;
			ll_R3::R3 eigenvecs[3];
			float eigenvals[3];
			const static unsigned char COMPUTE_ORIGINAL = 0x00;
			const static unsigned char COMPUTE_2 = 0x01;

			LPCA();
			LPCA(vector<ll_R3::R3> & points, unsigned char type_of_compute = COMPUTE_ORIGINAL);

			//the threshold decides whether point p (with color c) is included in the computation, where the threshold must 
			//be met as R3(c.red,c.green,c.blue).mag() >= threshold for p to be included
			LPCA(ll_pix3d::Pixel3DSet & p, float threshold, unsigned char type_of_compute = COMPUTE_ORIGINAL);

			LPCA(const LPCA & input);

			LPCA & operator = (const LPCA & input);

			void copyIn(const LPCA & input);

			//a basic compute of the pca
			void compute(std::vector<ll_R3::R3> & pts);

			//a compute of the pca, which attempts to re-arrange the directions
			void compute2(std::vector<ll_R3::R3> & pts);

			//generate the matrix which transforms from the x-axis to the eigen-vecs of pca
			Mat mat();
			//generate the matrix which transforms from the y-axis to the eigen-vecs of pca
			Mat mat_y();
			//generate the matrix which transforms from the axes defined by the eigen-vecs of pca, to the primary x,y,z axes
			Mat matinv();

			ll_R3::R3 best_eigen_vec();

			//performs matrix multiplication on a point, __p, based on a matrix produced by this algorithm
			static ll_R3::R3 mult(Mat & R, ll_R3::R3 __p);

			//generates a tralslation matrix based on the translation params tr.x, tr.y, tr.z
			static Mat translation_matrix(ll_R3::R3 tr);

			//generates the matrix which transforms the first set of points (which this was generated from) 
			//to align with the points of the second: b points*
			Mat full_matrix(LPCA & b);

			//this corrects the object so the primary axis is vertical
			Mat full_matrix_correct_vert(LPCA & b);

			static Mat compute_transform_between(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2);
			static Mat compute_transform_between_for_pc(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2);
			
			static Mat compute_alignment_for_pc(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2);
			static Mat compute_alignment_for_pc_small_change(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2);

			static void compute_axis(ll_pix3d::Pixel3DSet & p, ll_R3::R3 & center, ll_R3::R3 & a1, ll_R3::R3 & a2, ll_R3::R3 & a3);

			//fixes pca2's main vector to align more with pca1
			static void fix_eigenvectors(LPCA & pca1, LPCA & pca2);



		};
	}

	namespace ll_ann
	{
		class LNN_Set
		{
		public:
			/*
				you can get open a file with info like:
				x - total set
				y - training vector size
				z - training label vector size
				data row 0
				data row 1
				data row 2...
			*/
			cv::Ptr<cv::ml::ANN_MLP> ann;
			vector<vector<float>> data;
			vector<vector<float>> labels;
			double param1, param2, weight_scale;
			int num_iterations;
			LNN_Set(int num_iters = 1000, double wscale = 0.2); //initialize object
			LNN_Set(const LNN_Set & lnnset);
			LNN_Set & operator = (const LNN_Set & lnnset);
			void open(string fname); //open a dataset
			void open_weights(string fname); //load neural network using weights
			void save_weights(string fname); //save the weights for use later
			int training_set_size(); //retrieve the number of training instances
			int label_vector_size(); //retrieve the vector size of a label
			int training_vector_size(); //retrieve the vector size of a training instance
			Mat training_set(); //get the Mat version of the training data
			Mat training_set(int i); //get a single training instance in Mat format
			Mat label_set(); //get the labels for training in Mat format
			static string str(const vector<float> & list); //converts a vector of floats to a string
			string to_string(); //retrieves all the training data as a string
			void add_set(vector<float> data_, vector<float> & labels_); //adds a row of training data
			int train_it(int middle_layer_mult = 10); //trains the ANN
			vector<float> predict_it(vector<float> input); //after training, this predicts output based on input
		};

	}


	namespace ll_optical_flow
	{
		Mat denseOpticalFlow(Mat & previousFrame, Mat & currentFrame, double pyramidLevelScalar = 0.5, int numberOfPyramidLevels = 3, int windowSize = 15, int numberOfIterations = 3, int smoothness = 5, double smoothnessConstraint = 1.2, bool improvePerformance = false);
		Mat denseOpticalFlowUSE(Mat & previousFrame, Mat & currentFrame, Mat & of, double pyramidLevelScalar = 0.5, int numberOfPyramidLevels = 3, int windowSize = 15, int numberOfIterations = 3, int smoothness = 5, double smoothnessConstraint = 1.2);
		void drawDenseOpticalFlow(Mat & optical_flow_input, Mat & input, int stepSize = 10, Vec3b color = Vec3b(255, 255, 255), bool drawCircleOnPrevious = false);
		Mat formOFMask(Mat & ofin, double threshold = 0.5);
		Mat depthMapFromOF(const Mat & flow);
		Mat bestOFDepth(Mat & previous, Mat & current, Mat & tmp, int sad = 15, int bsize = 7);
	}

	class IGHM {
	public:
		IGHM();
		IGHM(Mat imIn);
		static Point2f differential(Mat & im, int y, int x);
		static Point2f differential2(Mat & im, int y, int x);
		vector<Point2i> & at(int y, int x);
		static bool ib(int y, int x);
		Size size();
		int total();
		Size sizeOfOriginal();
		Mat countImage();
		static Mat symIm(IGHM & gm);
		static Mat GenCountImage(Mat & im);
		static double computeRotation(Mat & im1, Mat & im2);
		static void phase_correlate_rt(Mat & im1, Mat & im2, double & rotationOut, Point2d & translationOut);
		static void phase_correlate_rt(Mat & im1, Mat & im2);
	private:
		vector<vector<Point2i>> hm;
		int w, h;
	public:
		static const float pif;
		static const float r2d;
	};

	class IPolarHM {
	public:
		IPolarHM();
		IPolarHM(Mat imIn, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		IPolarHM(Mat imIn, int w, int h, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		void compute(Mat & im, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		void compute(Mat & im, int w, int h, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static float logMag(int hw, int hh, int x, int y);
		//static float getScalar(int w);
		//static float getScalar(int w, int nw);
		vector<Point2i> & at(int y, int x);
		bool ib(int y, int x);
		Size size();
		int total();
		Size sizeOfOriginal();
		Mat countImage();
		Mat magImage();
		Mat hpImage();
		void clear(); //clears hash map
		static Mat GenCountImage(Mat & im, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static void phaseCorrelateRS(Mat & image1, Mat & image2, double & rotation, double & scale, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static void phaseCorrelateRST(Mat & image1, Mat & image2, double & rotation, double & scale, Point2d & trans, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static void phaseCorrelateRST(Mat & image1, Mat & image2, long long & ms, double & rotation, double & scale, Point2d & trans, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		//static void phaseCorrelateRST_NG(Mat & image1, Mat & image2, long long & ms, double & rotation, double & scale, Point2d & trans);
		static void phaseCorrelateRSTf(Mat & image1, Mat & image2, long long & ms, double & rotation, double & scale, Point2d & trans, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static Mat IPolarHM::HPImage(Mat & input, long long & ms, double scalar);
	private:
		vector<vector<Point2i>> hm;
		Mat originalCopy;
		void possibleResize(int w, int h); //resizes and clears hash map if required, else just  clears
		int w, h;
	};
	

}