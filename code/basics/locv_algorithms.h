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
		void disparity_errors(cv::Mat & errors, int row, cv::Mat & left_image, cv::Mat & right_image, int ndisparities, int sad_size);
		void optimize(cv::Mat & errors, cv::Mat & previous_positions, cv::Mat & running_costs, cv::Mat & depth_out, cv::Mat & errors_out, double smoothness, cv::Point2i & shift_amount);
		void compute(cv::Mat & im1, cv::Mat & im2, cv::Mat & depth, cv::Mat & errors, int ndisparities, int sad_size, double smoothness, bool fliplr = false);
		cv::Mat ll_sgdisparity(cv::Mat & im1, cv::Mat & im2, int ndisparities, int sad_size, double smoothness);
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
			LPCA(std::vector<ll_R3::R3> & points, unsigned char type_of_compute = COMPUTE_ORIGINAL);

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
			cv::Mat mat();
			//generate the matrix which transforms from the y-axis to the eigen-vecs of pca
			cv::Mat mat_y();
			//generate the matrix which transforms from the axes defined by the eigen-vecs of pca, to the primary x,y,z axes
			cv::Mat matinv();

			ll_R3::R3 best_eigen_vec();

			//performs matrix multiplication on a point, __p, based on a matrix produced by this algorithm
			static ll_R3::R3 mult(cv::Mat & R, ll_R3::R3 __p);

			//generates a tralslation matrix based on the translation params tr.x, tr.y, tr.z
			static cv::Mat translation_matrix(ll_R3::R3 tr);

			//generates the matrix which transforms the first set of points (which this was generated from) 
			//to align with the points of the second: b points*
			cv::Mat full_matrix(LPCA & b);

			//this corrects the object so the primary axis is vertical
			cv::Mat full_matrix_correct_vert(LPCA & b);

			static cv::Mat compute_transform_between(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2);
			static cv::Mat compute_transform_between_for_pc(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2);
			
			static cv::Mat compute_alignment_for_pc(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2);
			static cv::Mat compute_alignment_for_pc_small_change(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2);

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
			std::vector<std::vector<float>> data;
			std::vector<std::vector<float>> labels;
			double param1, param2, weight_scale;
			int num_iterations;
			LNN_Set(int num_iters = 1000, double wscale = 0.2); //initialize object
			LNN_Set(const LNN_Set & lnnset);
			LNN_Set & operator = (const LNN_Set & lnnset);
			void open(std::string fname); //open a dataset
			void open_weights(std::string fname); //load neural network using weights
			void save_weights(std::string fname); //save the weights for use later
			int training_set_size(); //retrieve the number of training instances
			int label_vector_size(); //retrieve the vector size of a label
			int training_vector_size(); //retrieve the vector size of a training instance
			cv::Mat training_set(); //get the Mat version of the training data
			cv::Mat training_set(int i); //get a single training instance in Mat format
			cv::Mat label_set(); //get the labels for training in Mat format
			static std::string str(const std::vector<float> & list); //converts a vector of floats to a string
			std::string to_string(); //retrieves all the training data as a string
			void add_set(std::vector<float> data_, std::vector<float> & labels_); //adds a row of training data
			int train_it(int middle_layer_mult = 10); //trains the ANN
			std::vector<float> predict_it(std::vector<float> input); //after training, this predicts output based on input
		};

	}


	namespace ll_optical_flow
	{
		cv::Mat denseOpticalFlow(cv::Mat & previousFrame, cv::Mat & currentFrame, double pyramidLevelScalar = 0.5, int numberOfPyramidLevels = 3, int windowSize = 15, int numberOfIterations = 3, int smoothness = 5, double smoothnessConstraint = 1.2, bool improvePerformance = false);
		cv::Mat denseOpticalFlowUSE(cv::Mat & previousFrame, cv::Mat & currentFrame, cv::Mat & of, double pyramidLevelScalar = 0.5, int numberOfPyramidLevels = 3, int windowSize = 15, int numberOfIterations = 3, int smoothness = 5, double smoothnessConstraint = 1.2);
		void drawDenseOpticalFlow(cv::Mat & optical_flow_input, cv::Mat & input, int stepSize = 10, cv::Vec3b color = cv::Vec3b(255, 255, 255), bool drawCircleOnPrevious = false);
		cv::Mat formOFMask(cv::Mat & ofin, double threshold = 0.5);
		cv::Mat depthMapFromOF(const cv::Mat & flow);
		cv::Mat bestOFDepth(cv::Mat & previous, cv::Mat & current, cv::Mat & tmp, int sad = 15, int bsize = 7);
	}

	class IGHM {
	public:
		IGHM();
		IGHM(cv::Mat imIn);
		static cv::Point2f differential(cv::Mat & im, int y, int x);
		static cv::Point2f differential2(cv::Mat & im, int y, int x);
		std::vector<cv::Point2i> & at(int y, int x);
		static bool ib(int y, int x);
		cv::Size size();
		int total();
		cv::Size sizeOfOriginal();
		cv::Mat countImage();
		static cv::Mat symIm(IGHM & gm);
		static cv::Mat GenCountImage(cv::Mat & im);
		static double computeRotation(cv::Mat & im1, cv::Mat & im2);
		static void phase_correlate_rt(cv::Mat & im1, cv::Mat & im2, double & rotationOut, cv::Point2d & translationOut);
		static void phase_correlate_rt(cv::Mat & im1, cv::Mat & im2);
	private:
		std::vector<std::vector<cv::Point2i>> hm;
		int w, h;
	public:
		static const float pif;
		static const float r2d;
	};

	class IPolarHM {
	public:
		IPolarHM();
		IPolarHM(cv::Mat imIn, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		IPolarHM(cv::Mat imIn, int w, int h, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		void compute(cv::Mat & im, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		void compute(cv::Mat & im, int w, int h, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static float logMag(int hw, int hh, int x, int y);
		//static float getScalar(int w);
		//static float getScalar(int w, int nw);
		std::vector<cv::Point2i> & at(int y, int x);
		bool ib(int y, int x);
		cv::Size size();
		int total();
		cv::Size sizeOfOriginal();
		cv::Mat countImage();
		cv::Mat magImage();
		cv::Mat hpImage();
		void clear(); //clears hash map
		static cv::Mat GenCountImage(cv::Mat & im, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static void phaseCorrelateRS(cv::Mat & image1, cv::Mat & image2, double & rotation, double & scale, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static void phaseCorrelateRST(cv::Mat & image1, cv::Mat & image2, double & rotation, double & scale, cv::Point2d & trans, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static void phaseCorrelateRST(cv::Mat & image1, cv::Mat & image2, long long & ms, double & rotation, double & scale, cv::Point2d & trans, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		//static void phaseCorrelateRST_NG(Mat & image1, Mat & image2, long long & ms, double & rotation, double & scale, Point2d & trans);
		static void phaseCorrelateRSTf(cv::Mat & image1, cv::Mat & image2, long long & ms, double & rotation, double & scale, cv::Point2d & trans, double lpScalar = LL_DEFAULT_LOG_TRANSFORM_SCALAR);
		static cv::Mat IPolarHM::HPImage(cv::Mat & input, long long & ms, double scalar);
	private:
		std::vector<std::vector<cv::Point2i>> hm;
		cv::Mat originalCopy;
		void possibleResize(int w, int h); //resizes and clears hash map if required, else just  clears
		int w, h;
	};
	

}