/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description:
		The .cpp file acompanying locv_algorithms

	depends on: opencv 3.0 and ll_R3, Pixel3DSet
*/
#include "locv_algorithms.h"
#include <fstream>
#include <chrono>
#include "VMatF.h"
using namespace ll_R3;
using namespace ll_pix3d;
using namespace std;
using namespace cv;

namespace ll_algorithms
{
	namespace ll_semiglobal_matcher
	{
		void disparity_errors(Mat & errors, int row, Mat & left_image, Mat & right_image, int ndispairities, int sad_size)
		{
			int from_ = sad_size;
			int to_ = left_image.size().width - (sad_size + ndispairities);
			if(errors.size() != Size(ndispairities, (1+to_-from_))) ll_error("ll_semiglobal_matcher::disparity_errors from locv_algorithms=> did not set up errors image to be correct size");

			double running_error = 0.0;
			double window_size = static_cast<double>((sad_size + sad_size + 1) * (sad_size + sad_size + 1));
			for(int x = from_, x_0 = 0; x <= to_; x++, x_0++)
			{
				for(int disparity = 0; disparity < ndispairities; disparity++)
				{
					running_error = 0.0;
					for(int _y = -sad_size; _y <= sad_size; _y++)
					{
						for(int _x = -sad_size; _x <= sad_size; _x++)
						{
							Vec3d pix1 = Vec3d(left_image.at<Vec3b>(_y + row, _x + x));
							Vec3d pix2 = Vec3d(right_image.at<Vec3b>(_y + row, _x + x + disparity));
							running_error += static_cast<double>((pix1-pix2).dot((pix1-pix2)));
						}
					}
					errors.at<double>(x_0,disparity) = running_error / window_size;

				}
			}
		}
		void optimize(Mat & errors, Mat & previous_positions, Mat & running_costs, Mat & depth_out, Mat & errors_out, double smoothness, Point2i & shift_amount)
		{
			int row_width = errors.size().height;
			int ndisparities = errors.size().width;
			for(int disparity = 0; disparity < ndisparities; disparity++) running_costs.at<double>(0, disparity) = errors.at<double>(0, disparity);

			//setup values:
			double lowest_cost = DBL_MAX;
			int previous_path = 0;
			for(int disparity = 0; disparity < ndisparities; disparity++)
			{
				for(int pixel = 1; pixel < row_width; pixel++)
				{
					lowest_cost = DBL_MAX;
					previous_path = 0;
					double current_cost = errors.at<double>(pixel, disparity);
					for(int disp = 0; disp < ndisparities; disp++)
					{
						double distance = static_cast<double>(abs(disp-disparity));
						double cost = errors.at<double>(pixel-1, disp) + current_cost + smoothness*distance;
						if(cost < lowest_cost)
						{
							lowest_cost = cost;
							previous_path = disp;
						}
					}
					previous_positions.at<int>(pixel, disparity) = previous_path;
					running_costs.at<double>(pixel, disparity) = lowest_cost;
				}
			}

			int end = row_width-1;
			lowest_cost = running_costs.at<double>(end, 0);
			int path_from = 0;
			for(int disparity = 1; disparity < ndisparities; disparity++)
			{
				double cost = running_costs.at<double>(end, disparity);
				if(cost < lowest_cost)
				{
					lowest_cost = cost;
					path_from = disparity;
				}
			}

			for(int pixel = end, disparity = path_from, pixelfw = 0; pixel >= 0; pixel--, pixelfw++)
			{
				double error = running_costs.at<double>(pixel, disparity);
				int depth = disparity;
				Point2i p(shift_amount.x + pixelfw, shift_amount.y);
				p.x = (end+shift_amount.x)-p.x;
				depth_out.at<unsigned char>(p.y, p.x) = static_cast<unsigned char>(depth);
				errors_out.at<double>(p.y, p.x) = error;
				disparity = previous_positions.at<int>(pixel, disparity);
			}

		}
		void compute(Mat & im1, Mat & im2, Mat & depth, Mat & errors, int ndisparities, int sad_size, double smoothness, bool fliplr)
		{
			int from_ = sad_size;
			int to_ = im1.size().width - (sad_size + ndisparities);
			int to_y_ = im1.size().height - sad_size;
			depth = Mat::ones(im1.size(), CV_8UC1);
			errors = Mat::ones(im1.size(), CV_64FC1);
			depth *= 255;
			Size optimize_size(ndisparities, 1+to_-from_);
			Mat error = Mat::zeros(optimize_size, CV_64FC1);
			Mat running_error = Mat::zeros(optimize_size, CV_64FC1);
			Mat previous_positions = Mat::zeros(optimize_size, CV_32SC1);
			for(int y = from_; y <= to_y_; y++)
			{
				if(!fliplr) disparity_errors(error, y, im1, im2, ndisparities, sad_size);
				else disparity_errors(error, y, im2, im1, ndisparities, sad_size);
				Point2i pnt(from_, from_+y-1);
				optimize(error, previous_positions, running_error, depth, errors, smoothness, pnt);
			}
		}
		Mat ll_sgdisparity(Mat & im1, Mat & im2, int ndisparities, int sad_size, double smoothness)
		{
			Mat d, e;
			compute(im1, im2, d, e, ndisparities, sad_size, smoothness);
			return d.clone();
		}
	}

	namespace ll_pca_3d
	{
		LPCA::LPCA(){}
		LPCA::LPCA(vector<R3> & points, unsigned char type_of_compute)
		{
			if(type_of_compute == LPCA::COMPUTE_ORIGINAL) compute(points);
			else compute2(points);
		}
		LPCA::LPCA(Pixel3DSet & p, float threshold, unsigned char type_of_compute)
		{
			vector<R3> points;
			for(int i = 0; i < p.points.size(); i++)
			{
				R3 color(static_cast<float>(p.colors[i][0]), static_cast<float>(p.colors[i][1]), static_cast<float>(p.colors[i][2]));
				if(color.mag() >= threshold) points.push_back(
					p.points[i]
					);
			}
			if(type_of_compute == LPCA::COMPUTE_ORIGINAL) compute(points);
			else compute2(points);
		}
		LPCA::LPCA(const LPCA & input)
		{
			copyIn(input);
		}

		LPCA & LPCA::operator = (const LPCA & input)
		{
			if(this == &input) return *this;
			copyIn(input);
			return *this;
		}

		void LPCA::copyIn(const LPCA & input)
		{
			for(int i = 0; i < 3; i++)
			{
				eigenvecs[i] = input.eigenvecs[i];
				eigenvals[i] = input.eigenvals[i];
			}
			mean = input.mean;
		}

		void LPCA::compute(vector<R3> & pts)
		{
			//Construct a buffer used by the pca analysis
			Mat data_pts = Mat(pts.size(), 3, CV_64FC1);
			for (int i = 0; i < data_pts.rows; ++i)
			{
				data_pts.at<double>(i, 0) = (double)pts[i].x;
				data_pts.at<double>(i, 1) = (double)pts[i].y;
				data_pts.at<double>(i, 2) = (double)pts[i].z;
			}

			//Perform PCA analysis
			PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
			

			//Store the position of the object
			R3 pos = R3(static_cast<float>(pca_analysis.mean.at<double>(0, 0)),
								static_cast<float>(pca_analysis.mean.at<double>(0, 1)),
								static_cast<float>(pca_analysis.mean.at<double>(0, 2))
						);

			cout << "got mean " << pos << endl;

			//Store the eigenvalues and eigenvectors
			vector<R3> eigen_vecs(3);
			vector<double> eigen_val(3);
			for (int i = 0; i < 3; ++i)
			{
				eigen_vecs[i] = R3(pca_analysis.eigenvectors.at<double>(i, 0),
										pca_analysis.eigenvectors.at<double>(i, 1),
										pca_analysis.eigenvectors.at<double>(i, 2));

				eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
			}
			eigenvals[0] = eigen_val[0];
			eigenvals[1] = eigen_val[1];
			eigenvals[2] = eigen_val[2];

 			eigenvecs[0] = eigen_vecs[0].unit();
			eigenvecs[1] = eigen_vecs[1].unit();
			eigenvecs[2] = eigen_vecs[2].unit();
			mean = pos;
		}

		void LPCA::compute2(vector<R3> & pts)
		{

			if(pts.size() < 3)
			{
				eigenvecs[0] = R3(0.0f, 1.0f, 0.0f);
				eigenvecs[1] = R3(1.0f, 0.0f, 0.0f);
				eigenvecs[2] = R3(0.0f, 0.0f, 1.0f);
				mean = R3();

				return;
			}

			//Construct a buffer used by the pca analysis
			Mat data_pts = Mat(pts.size(), 3, CV_64FC1);
			for (int i = 0; i < data_pts.rows; ++i)
			{
				data_pts.at<double>(i, 0) = (double)pts[i].x;
				data_pts.at<double>(i, 1) = (double)pts[i].y;
				data_pts.at<double>(i, 2) = (double)pts[i].z;
			}


			//Perform PCA analysis
			PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
			

			//Store the position of the object
			R3 pos = R3(pca_analysis.mean.at<double>(0, 0),
								pca_analysis.mean.at<double>(0, 1),
								pca_analysis.mean.at<double>(0, 2));
			//cout << pos << endl;
			//Store the eigenvalues and eigenvectors
			//vector<R3> eigen_vecs(3);
			//vector<double> eigen_val(3);
			//cout << pca_analysis.eigenvectors << endl;
			//cout << pca_analysis.eigenvalues << endl;
			for (int i = 0; i < 3; ++i)
			{
				//eigen vecs are the cols
				//eigenvecs[i] = R3(pca_analysis.eigenvectors.at<double>(0, i),pca_analysis.eigenvectors.at<double>(1, i),pca_analysis.eigenvectors.at<double>(2, i));
				//eigen vecs are the rows
				eigenvecs[i] = R3(pca_analysis.eigenvectors.at<double>(i, 0),pca_analysis.eigenvectors.at<double>(i, 1),pca_analysis.eigenvectors.at<double>(i, 2));

				eigenvals[i] = pca_analysis.eigenvalues.at<double>(0, i);
			}

			//eigenvals[0] = eigen_val[0];
			//eigenvals[1] = eigen_val[1];
			//eigenvals[2] = eigen_val[2];
			

 			//eigenvecs[0] = eigen_vecs[0].unit();
			//eigenvecs[1] = R3(1.0f, 0.0f, 0.0f);
			//eigenvecs[2] = eigenvecs[1] ^ eigenvecs[0];
			//eigenvecs[1] = eigenvecs[0] ^ eigenvecs[2];
			//eigenvecs[0].normalize();
			//eigenvecs[1].normalize();
			//eigenvecs[2].normalize();
			mean = pos;
		}

		Mat LPCA::mat()
		{
			R3 a = eigenvecs[0];
			R3 b = eigenvecs[1];
			R3 c = eigenvecs[2];
			a.normalize();
			b.normalize();
			c.normalize();
			float t[] =
			{
				a.x, b.x, c.x, 0.0f,
				a.y, b.y, c.y, 0.0f,
				a.z, b.z, c.z, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};
			Mat rv(4, 4, CV_32FC1, t);
			rv = rv.t();
			return rv.clone();
		}

		Mat LPCA::mat_y()
		{
			R3 a = eigenvecs[0];
			R3 b = eigenvecs[1];
			R3 c = eigenvecs[2];
			a.normalize();
			b.normalize();
			c.normalize();
			float t[] =
			{
				b.x,	a.x,	c.x,	0.0f,
				b.y,	a.y,	c.y,	0.0f,
				b.z,	a.z,	c.z,	0.0f,
				0.0f,	0.0f,	0.0f,	1.0f
			};
			Mat rv(4, 4, CV_32FC1, t);
			rv = rv.inv();
			return rv.clone();
		}

		Mat LPCA::matinv()
		{
			Mat m = mat();
			m = m.t();
			return m.clone();
		}

		R3 LPCA::mult(Mat & R, R3 __p)
		{
			float _p[] = {__p.x, __p.y, __p.z, 1.0f};
			Mat p = Mat(4, 1, CV_32FC1, _p);
			p = R * p;
			return R3(p.at<float>(Point2i(0,0)), p.at<float>(Point2i(0,1)), p.at<float>(Point2i(0,2)));
		}

		Mat LPCA::translation_matrix(R3 tr)
		{
			float t[] =
			{
				1.0f, 0.0f, 0.0f, tr.x,
				0.0f, 1.0f, 0.0f, tr.y,
				0.0f, 0.0f, 1.0f, tr.z,
				0.0f, 0.0f, 0.0f, 1.0f
			};
			Mat rv(4, 4, CV_32FC1, t);
			return rv.clone();
		}

		Mat LPCA::full_matrix(LPCA & b)
		{
			Mat m = translation_matrix(b.mean) * b.matinv() * this->mat() * translation_matrix(-mean);
			return m.clone();
		}

		Mat LPCA::full_matrix_correct_vert(LPCA & b)
		{
			int index = 0;
			R3 e1 = -eigenvecs[index];
			R3 e2 = b.eigenvecs[index];
			Mat rv = VMat::correction_matrix_up_axis(mean, mean + e1 * 1.0f, b.mean, b.mean + e2 * 1.0f);
			return rv.clone();
			//Mat bmatyi = b.mat_y();
			//bmatyi = bmatyi.inv();
			//Mat m = translation_matrix(mean) * bmatyi * mat_y() * translation_matrix(-mean);
			//return m.clone();
		}

		Mat LPCA::compute_transform_between(Pixel3DSet & p, Pixel3DSet & p2)
		{
			ll_algorithms::ll_pca_3d::LPCA pc1(p, 0.5f);
			ll_algorithms::ll_pca_3d::LPCA pc2(p2, 0.5f);

			Mat m = pc1.full_matrix(pc2);
			return m.clone();
		}

		Mat LPCA::compute_transform_between_for_pc(Pixel3DSet & p, Pixel3DSet & p2)
		{
			ll_algorithms::ll_pca_3d::LPCA pc1(p, 0.2f, LPCA::COMPUTE_2);
			ll_algorithms::ll_pca_3d::LPCA pc2(p2, 0.2f, LPCA::COMPUTE_2);

			Mat m = pc1.full_matrix_correct_vert(pc2);
			return m.clone();
		}


		Mat LPCA::compute_alignment_for_pc(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2)
		{
			Mat m;
			cout << "here : " << endl;
			ll_algorithms::ll_pca_3d::LPCA pc1(p, 0.2f, LPCA::COMPUTE_2);
			ll_algorithms::ll_pca_3d::LPCA pc2(p2, 0.2f, LPCA::COMPUTE_2);

			m = pc1.full_matrix_correct_vert(pc2);

			return m.clone();
		}

		Mat LPCA::pca_register(ll_pix3d::Pixel3DSet & p1, ll_pix3d::Pixel3DSet & p2)
		{
			ll_algorithms::ll_pca_3d::LPCA pc1(p1.points, LPCA::COMPUTE_2);
			ll_algorithms::ll_pca_3d::LPCA pc2(p2.points, LPCA::COMPUTE_2);

			float datapc1[16] = {
				pc1.eigenvecs[0].x, pc1.eigenvecs[1].x, pc1.eigenvecs[2].x, 0.0f,
				pc1.eigenvecs[0].y, pc1.eigenvecs[1].y, pc1.eigenvecs[2].y, 0.0f,
				pc1.eigenvecs[0].z, pc1.eigenvecs[1].z, pc1.eigenvecs[2].z, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};

			/*cout << pc1.eigenvecs[0] << " -> " << pc2.eigenvecs[0] << endl;
			cout << pc1.eigenvecs[1] << " -> " << pc2.eigenvecs[1] << endl;
			cout << pc1.eigenvecs[2] << " -> " << pc2.eigenvecs[2] << endl;

			cout << pc1.eigenvals[0] << " -> " << pc2.eigenvals[0] << endl;
			cout << pc1.eigenvals[1] << " -> " << pc2.eigenvals[1] << endl;
			cout << pc1.eigenvals[2] << " -> " << pc2.eigenvals[2] << endl;
			cout << "()()()()()()" << endl;*/

			float datapc2[16] = {
				pc2.eigenvecs[0].x, pc2.eigenvecs[1].x, pc2.eigenvecs[2].x, 0.0f,
				pc2.eigenvecs[0].y, pc2.eigenvecs[1].y, pc2.eigenvecs[2].y, 0.0f,
				pc2.eigenvecs[0].z, pc2.eigenvecs[1].z, pc2.eigenvecs[2].z, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};

			Mat pc1Rm(Size(4,4), CV_32FC1, datapc1);
			Mat pc2Rm(Size(4,4), CV_32FC1, datapc2);
			Mat pc1RmT; transpose(pc1Rm, pc1RmT);
			Mat pc2RmT; transpose(pc2Rm, pc2RmT);


			
			Mat ret = Mat::eye(Size(4,4), CV_32FC1);

			/*
				um mean, un-rotate by self, rotate by other, add other's mean
			*/
			Mat unMean = Pixel3DSet::transformation_matrix(0.0f, 0.0f, 0.0f, 1.0f, -pc1.mean.x, -pc1.mean.y, -pc1.mean.z, R3());
			Mat meanIt = Pixel3DSet::transformation_matrix(0.0f, 0.0f, 0.0f, 1.0f, pc2.mean.x, pc2.mean.y, pc2.mean.z, R3());
			//cout << unMean << endl << meanIt << endl;
			//cout << pc1.mean << endl;
			//cout << pc2.mean << endl;
			//cout << unMean << endl;
			//cout << meanIt << endl;
			//cout << pc1Rm << endl << pc2Rm << endl;// << (pc1RmT*pc1Rm) << endl;
			//cout << pc2Rm << endl;

			ret = meanIt * pc2Rm * pc1RmT * unMean;

			//cout << ret << endl;

			return ret.clone();
		}



		R3 LPCA::best_eigen_vec()
		{
			if(eigenvals[0] >= eigenvals[1] && eigenvals[0] >= eigenvals[2]) return eigenvecs[0];
			else if(eigenvals[1] >= eigenvals[2]) return eigenvecs[1];
			return eigenvecs[2];
		}


		void LPCA::fix_eigenvectors(LPCA & pca1, LPCA & pca2)
		{
			auto a = pca1.eigenvecs[0].unit();
			auto b = pca2.eigenvecs[0].unit();
			if(a.dist(-b) < a.dist(b)) pca2.eigenvecs[0].inv();
		}

		Mat LPCA::compute_alignment_for_pc_small_change(ll_pix3d::Pixel3DSet & p, ll_pix3d::Pixel3DSet & p2)
		{
			Mat m;

			ll_algorithms::ll_pca_3d::LPCA pc1(p, 0.2f, LPCA::COMPUTE_2);
			ll_algorithms::ll_pca_3d::LPCA pc2(p2, 0.2f, LPCA::COMPUTE_2);
			LPCA::fix_eigenvectors(pc2, pc1);
			m = pc1.full_matrix_correct_vert(pc2);

			return m.clone();
		}

		void LPCA::compute_axis(ll_pix3d::Pixel3DSet & p, ll_R3::R3 & center, ll_R3::R3 & a1, ll_R3::R3 & a2, ll_R3::R3 & a3)
		{
			ll_algorithms::ll_pca_3d::LPCA pc1(p, 0.5f);
			center = pc1.mean;
			a1 = pc1.eigenvecs[0];
			a2 = pc1.eigenvecs[1];
			a3 = pc1.eigenvecs[2];
		}
	}

	namespace ll_ann
	{
		LNN_Set::LNN_Set(int num_iters , double wscale)
		{
			param1 = param2 = 1.0;
			weight_scale = wscale;
			num_iterations = num_iters;
		}
		LNN_Set::LNN_Set(const LNN_Set & lnnset)
		{
			ann = lnnset.ann;
			data = lnnset.data;
			labels = lnnset.labels;
			param1 = lnnset.param1;
			param2 = lnnset.param2;
			weight_scale = lnnset.weight_scale;
			num_iterations = lnnset.num_iterations;
		}
		LNN_Set & LNN_Set::operator = (const LNN_Set & lnnset)
		{
			if(this == &lnnset) return *this;
			ann = lnnset.ann;
			data = lnnset.data;
			labels = lnnset.labels;
			param1 = lnnset.param1;
			param2 = lnnset.param2;
			weight_scale = lnnset.weight_scale;
			num_iterations = lnnset.num_iterations;
			return *this;
		}
		void LNN_Set::open(string fname)
		{
			data.clear();
			labels.clear();
			ifstream fi(fname.c_str(), ios::in);
			if(fi.is_open())
			{
				int rows = 0, cols = 0, lcols;
				float tmp;
				fi >> rows;
				fi >> cols;
				fi >> lcols;
				for(int i = 0; i < rows; i++)
				{
					vector<float> l(cols);
					for(int j = 0; j < cols; j++)
					{
						fi >> l[j];
					}
					vector<float> l2(lcols);
					for(int j = 0; j < lcols; j++)
					{
						fi >> l2[j];
					}
					data.push_back(l);
					labels.push_back(l2);
				}
				fi.close();
			}


		}
		void LNN_Set::open_weights(string fname)
		{
			cv::String st(fname);
#ifdef _WIN32
			ann = cv::ml::ANN_MLP::load<cv::ml::ANN_MLP>(fname);
#else
			ann = cv::ml::ANN_MLP::load(fname);
#endif
			Mat tmp = ann->getLayerSizes();
			int tvecsize = tmp.at<int>(0,0);
			data.push_back(vector<float>(tvecsize, 0.0f));
		}
		void LNN_Set::save_weights(string fname)
		{
			ann->save(fname);
		}
		int LNN_Set::training_set_size()
		{
			return data.size();
		}
		int LNN_Set::label_vector_size()
		{
			return labels[0].size();
		}
		int LNN_Set::training_vector_size()
		{
			return data[0].size();
		}
		Mat LNN_Set::training_set()
		{
			Mat rv = Mat::zeros(Size(training_vector_size(),training_set_size()), CV_32FC1);
			for(int i = 0; i < training_set_size(); i++)
			{
				vector<float> l = data[i];
				for(int j = 0; j < l.size(); j++) rv.at<float>(Point2i(j, i)) = l[j];
			}
			return rv.clone();
		}
		Mat LNN_Set::training_set(int i)
		{
			Mat rv = Mat::zeros(Size(training_vector_size(),1), CV_32FC1);
			vector<float> l = data[i];
			for(int j = 0; j < l.size(); j++) rv.at<float>(Point2i(j, 0)) = l[j];
			return rv.clone();
		}
		Mat LNN_Set::label_set()
		{
			Mat rv = Mat::zeros(Size(label_vector_size(),training_set_size()), CV_32FC1);
			for(int i = 0; i < training_set_size(); i++)
			{
				vector<float> l = labels[i];
				for(int j = 0; j < l.size(); j++) rv.at<float>(Point2i(j, i)) = l[j];
			}
			return rv.clone();
		}
		string LNN_Set::str(const vector<float> & list)
		{
			string rv = "";
			for(int i = 0; i < list.size(); i++)
			{
				rv += std::to_string(list[i]);
				if(i < list.size()-1) rv += ", ";
			}
			return rv;
		}
		string LNN_Set::to_string()
		{
			string rv = "";
			for(int i = 0; i < data.size(); i++)
			{
				rv += "d[" + std::to_string(i) + "] [" + str(data[i]) + "] = {" + str(labels[i]) + "}\n";
			}
			return rv;
		}
		void LNN_Set::add_set(vector<float> data_, vector<float> & labels_)
		{
			if(data.empty() || (data_.size() == training_vector_size() && labels_.size() == label_vector_size()))
			{
				data.push_back(data_);
				labels.push_back(labels_);
			}
		}
		int LNN_Set::train_it(int middle_layer_mult)
		{
			ann = cv::ml::ANN_MLP::create();
			Mat layers = Mat::zeros(Size(1, 3), CV_32SC1);
			layers.at<int>(0, 0) = training_vector_size();
			layers.at<int>(1, 0) = training_vector_size()*middle_layer_mult;
			layers.at<int>(2, 0) = label_vector_size();
			ann->setLayerSizes(layers);
			ann->setActivationFunction(cv::ml::ANN_MLP::SIGMOID_SYM, param1, param2);
			ann->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, num_iterations, FLT_EPSILON));
			ann->setTrainMethod(cv::ml::ANN_MLP::BACKPROP, weight_scale);

			Mat tset = this->training_set();
			Mat lset = this->label_set();
			return ann->train(tset,ml::SampleTypes::ROW_SAMPLE,lset);
		}
		vector<float> LNN_Set::predict_it(vector<float> input)
		{
			vector<float> rv;
			if(input.size() != training_vector_size()) return rv;
			Mat in = Mat::zeros(Size(input.size(), 1), CV_32FC1);
			for(int i = 0; i < input.size(); i++) in.at<float>(Point2i(i, 0)) = input[i];
			Mat op;
			ann->predict(in, op);
			for(int i = 0; i < op.size().width; i++) rv.push_back(op.at<float>(Point2i(i,0)));
			return rv;
		}
	}

	namespace ll_optical_flow
	{
		Mat denseOpticalFlow(Mat & previousFrame, Mat & currentFrame, double pyramidLevelScalar, int numberOfPyramidLevels,
			int windowSize, int numberOfIterations, int smoothness, double smoothnessConstraint, bool improvePerformance)
		{
			Mat rv;
			if(!improvePerformance)
				calcOpticalFlowFarneback(previousFrame, currentFrame, rv, pyramidLevelScalar, numberOfPyramidLevels, windowSize, numberOfIterations, smoothness, smoothnessConstraint, 0);
			else calcOpticalFlowFarneback(previousFrame, currentFrame, rv, pyramidLevelScalar, numberOfPyramidLevels, windowSize, numberOfIterations, smoothness, smoothnessConstraint, OPTFLOW_FARNEBACK_GAUSSIAN);
			return rv.clone();
		}
		Mat denseOpticalFlowUSE(Mat & previousFrame, Mat & currentFrame, Mat & of, double pyramidLevelScalar, int numberOfPyramidLevels,
			int windowSize, int numberOfIterations, int smoothness, double smoothnessConstraint)
		{
			calcOpticalFlowFarneback(previousFrame, currentFrame, of, pyramidLevelScalar, numberOfPyramidLevels, windowSize, numberOfIterations, smoothness, smoothnessConstraint, OPTFLOW_USE_INITIAL_FLOW);
			return of.clone();
		}
		void drawDenseOpticalFlow(Mat & ofin, Mat & im, int step, Vec3b color, bool drawCircPrev)
		{
			for(int y = 0; y < ofin.rows; y += step)
			{
				for(int x = 0; x < ofin.cols; x += step)
				{
					const Point2f& fxy = ofin.at< Point2f>(y, x);
					line(im, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
							color);
					if(drawCircPrev) circle(im, Point(cvRound(x), cvRound(y)), 1, color, -1);
					else circle(im, Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), 1, color, -1);
				}
			}
		}
		Mat formOFMask(Mat & ofin, double threshold)
		{
			Mat rv = Mat::zeros(ofin.size(), CV_8UC1);
			for(int y = 0; y < rv.rows; y++)
			{
				for(int x = 0; x < rv.cols; x++)
				{
					Point2f & p = ofin.at<Point2f>(y,x);
					double dist = sqrt(p.dot(p));
					if(dist >= threshold) rv.at<unsigned char>(y,x) = 0xFF;
				}
			}
			return rv.clone();
		}
		Mat depthMapFromOF(const Mat & flow)
		{
			Mat rv = Mat::zeros(flow.size(), CV_32FC1);
			for(int y = 0; y < rv.rows; y++)
			{
				for(int x = 0; x < rv.cols; x++)
				{
					const Point2f & p = flow.at<Point2f>(y,x);
					rv.at<float>(y,x) = sqrt(p.dot(p));
				}
			}
			return rv.clone();
		}
		Mat bestOFDepth(Mat & previous, Mat & current, Mat & tmp, int sad, int bs)
		{
			Mat of;
			if(tmp.size() != Size(0,0))
			{
				//cout << "doing it" << endl;
				of = denseOpticalFlowUSE(previous, current, tmp, 0.5, 3, sad, 15, 7, 1.2);
				//of = denseOpticalFlow(  previous, current,       0.5, 2, sad, 10, 7, 1.2, true);
				//of = tmp*.4f + of*0.6f;
				tmp = of.clone();
			}else
			{
				of = denseOpticalFlow(  previous, current,       0.5, 3, sad, 10, 7, 1.2, true);
				tmp = of.clone();
			}
			Mat depth = depthMapFromOF(of);
			Mat dm = formOFMask(of, 0.6);
			ll_normalize(depth, dm, 0.0);
			threshold(depth, depth, 0.5, 1.0, CV_THRESH_TOZERO_INV);
			ll_normalize(depth);
			ll_32F1_to_UCF1(depth);
			//equalizeHist(depth, depth);
			return depth.clone();
		}

	}
	IGHM::IGHM()
	{
		w = 360, h = 256;
		hm = vector<vector<Point2i>>(w*h);
	}
	IGHM::IGHM(Mat imIn)
	{
		w = 360, h = 256;
		Mat im;
		Size imInSize = imIn.size();
		GaussianBlur(imIn, im, Size(7,7), 0, 0, BORDER_DEFAULT );
		hm = vector<vector<Point2i>>(w*h);
		for (int y = 1; y < imInSize.height-1; y++) {
			for (int x = 1; x < imInSize.width-1; x++) {
				Point2f d = differential2(im, y, x);
				float mag = sqrt(d.x*d.x + d.y*d.y) * 256.0f;
				float ori = atan2(d.x, d.y) * r2d + 180.f;
				int magi = (int)mag;
				int orii = (int)ori;
				if (orii >= 0 && orii < 360 && magi >= 1 && magi < 255)
				{
					at(magi, orii).push_back(Point2i(x, y));
				}
			}
		}
	}
	Point2f IGHM::differential(Mat & im, int y, int x)
	{
		Point2f rv;
		Size ms = im.size();
		if((x-1) < 0 || (x+1) >= ms.width || (y-1) < 0 || (y+1) >= ms.height) return rv;
		rv.x = -im.at<float>(y-1,x-1) + im.at<float>(y-1,x+1) + -2.f*im.at<float>(y,x-1) + 2.f*im.at<float>(y,x+1) + -im.at<float>(y+1,x-1) + im.at<float>(y+1,x+1);
		rv.y = -im.at<float>(y-1,x-1) + -2.f*im.at<float>(y-1,x) + -im.at<float>(y-1,x+1) + im.at<float>(y+1,x-1) + 2.f*im.at<float>(y+1,x) + im.at<float>(y+1,x+1);
		return rv;
	}
	Point2f IGHM::differential2(Mat & im, int y, int x)
	{
		Point2f rv;
		Size ms = im.size();
		if((x) < 0 || (x+1) >= ms.width || (y) < 0 || (y+1) >= ms.height) return rv;
		rv.x = im.at<float>(y,x+1) - im.at<float>(y,x);
		rv.y = im.at<float>(y+1,x) - im.at<float>(y,x);
		return rv;
	}
	vector<Point2i> & IGHM::at(int y, int x)
	{
		return hm[y * w + x];
	}
	bool IGHM::ib(int y, int x)
	{
		return y >= 0 && x >= 0 && x < 360 && y < 256;
	}
	Size IGHM::size()
	{
		return Size(w, h);
	}
	int IGHM::total()
	{
		int counter = 0;
		int si = w*h;
		for (int i = 0; i < si; i++)
		{
			counter += hm[i].size();
		}
		return counter;
	}
	Size IGHM::sizeOfOriginal()
	{
		Size rv(1,1);
		int plsize;
		int px, py;
		for(int y = 0; y < h; y++)
		{
			for(int x = 0; x < w; x++)
			{
				vector<Point2i> & pl = at(y,x);
				plsize = pl.size();
				for(int i = 0; i < plsize; i++)
				{
					px = pl[i].x;
					py = pl[i].y;
					rv.width = px > rv.width ? px : rv.width;
					rv.height = py > rv.height ? py : rv.height;
				}
			}
		}
		return rv + Size(2,2);
	}
	Mat IGHM::countImage()
	{
		Mat rv = Mat::zeros(size(), CV_32FC1);
		int t = total();
		if (t == 0) return rv.clone();
		for (int y = 0; y < h; y++)
		{
			for (int x = 0; x < w; x++)
			{
				rv.at<float>(y, x) = at(y, x).size();
			}
		}
		ll_normalize(rv);
		return rv.clone();
	}

	Mat IGHM::symIm(IGHM & gm)
	{
		Size gmSize = gm.size();
		Size ns = gm.sizeOfOriginal();
		//cout << ns << endl;
		Mat rv = Mat::zeros(ns, CV_32FC1);
		int magnitudeRange = 2;
		int angleRange = 10;
		for(int y = 0; y < gmSize.height; y++)
		{
			for(int x = 0; x < gmSize.width; x++)
			{
				int sangle = (x+180) % 360;
				vector<Point2i> & pl1 = gm.at(y,x);
				int pl1Size = pl1.size();
				if(pl1.size() == 0) continue;
				for(int y2 = y-magnitudeRange; y2 <= y+magnitudeRange; y2++)
				{
					if(!gm.ib(y2,1)) continue;
					for(int x2 = sangle-angleRange; x2 <= sangle+angleRange; x2++)
					{
						if(!gm.ib(y2,x2)) continue;
						//for each point pair
						vector<Point2i> & pl2 = gm.at(y2,x2);
						int pl2Size = pl2.size();
						for(int i = 0; i < pl1Size; i++)
						{
							for(int j = 0; j < pl2Size; j++)
							{
								Point2i pi = pl1[i];
								Point2i pj = pl2[j];
								//if(pj.x == pi.x) continue;
								Point2i cp = (pj + pi) / 2;
								float aos = (float)atan2((double)(pi.y - pj.y), (double)(pj.x - pi.x));
								float orii = x;
								float magi = y;
								float orij = x2;
								float magj = y2;
								float adifi = abs(ll_algorithms::IGHM::pif - (abs(aos - orii) - ll_algorithms::IGHM::pif));
								float adifj = abs(ll_algorithms::IGHM::pif - (abs(aos - orij) - ll_algorithms::IGHM::pif));
								float contribution = (magi * magj) / (1.f + abs(adifi - adifj));
								//cout << contribution << endl;
								if(cp.x >= 0 && cp.y >= 0 && cp.x < ns.width && cp.y < ns.height)
								{
									rv.at<float>(cp) += contribution;
								}
							}
						}
					}
				}
			}
		}
		ll_normalize(rv);
		return rv.clone();
		}
	Mat IGHM::GenCountImage(Mat & imIn)
	{
		int w = 360, h = 256;
		Mat im;
		bool toNormalize = false;
		Mat rv = Mat::zeros(Size(360, 256), CV_32FC1);
		Size imInSize = imIn.size();
		GaussianBlur(imIn, im, Size(7,7), 0, 0, BORDER_DEFAULT );
		for (int y = 1; y < imInSize.height-1; y++) {
			for (int x = 1; x < imInSize.width-1; x++) {
				Point2f d = differential2(im, y, x);
				float mag = sqrt(d.x*d.x + d.y*d.y) * 256.0f;
				float ori = atan2(d.x, d.y) * r2d + 180.f;
				int magi = (int)mag;
				int orii = (int)ori;
				if (orii >= 0 && orii < 360 && magi >= 1 && magi < 255)
				{
					toNormalize = true;
					rv.at<float>(magi, orii) += 1.f;
				}
			}
		}
		if(toNormalize)
			ll_normalize(rv);
		return rv.clone();
	}
	double IGHM::computeRotation(Mat & im1, Mat & im2)
	{
		Mat a = ll_algorithms::IGHM::GenCountImage(im1);
		Mat b = ll_algorithms::IGHM::GenCountImage(im2);
		Point2d p = ll_phase_correlate(a, b);
		return p.x < 0.0 ? -p.x : 360-p.x;
	}
	void IGHM::phase_correlate_rt(Mat & im1, Mat & im2, double & rotationOut, Point2d & translationOut)
	{
		rotationOut = computeRotation(im1, im2);
		Mat tmp;
		ll_transform_image(im1, tmp, rotationOut, 1.0, 0.0, 0.0);
		translationOut = ll_phase_correlate(tmp, im2);
	}

	void IGHM::phase_correlate_rt(Mat & im1, Mat & im2)
	{
		double rotationOut = computeRotation(im1, im2);
		Mat tmp;
		ll_transform_image(im1, tmp, rotationOut, 1.0, 0.0, 0.0);
		Point2d translationOut = ll_phase_correlate(tmp, im2);
		ll_transform_image(im1, im1, rotationOut, 1.0, translationOut.x, translationOut.y);
	}

	const float IGHM::pif = 3.14159265359f;
	const float IGHM::r2d = 57.2958f;

	/*
		IPolarHM implementation::
	*/


	IPolarHM::IPolarHM()
	{
		w = 360, h = 256;
		hm = vector<vector<Point2i>>(w*h);
	}

	void IPolarHM::possibleResize(int w, int h)
	{
		if (this->w != w || this->h != h)
		{
			//cout << "resizing" << endl;
			this->w = w;
			this->h = h;
			hm = vector<vector<Point2i>>(w*h);
			//cout << hm.size() << endl;
		}else clear();
	}

	void IPolarHM::clear()
	{
		int s = w * h;
		for (int i = 0; i < s; i++) hm[i].clear();
	}

	IPolarHM::IPolarHM(Mat imIn, double lpScalar)
	{
		compute(imIn, lpScalar);
	}

	IPolarHM::IPolarHM(Mat imIn, int w, int h, double lpScalar)
	{
		compute(imIn, w, h, lpScalar);
	}

	float IPolarHM::logMag(int hw, int hh, int x, int y)
	{
		return log(ll_distance<float>(x, y, hw, hh));
	}

	/*
	float IPolarHM::getScalar(int w)
	{
		return ((float)w) / log(((float)w) / 2.56f);
	}

	float IPolarHM::getScalar(int w, int nw)
	{
		return ((float)nw) / log(((float)w) / 2.56f);
	}*/

	//compute is complete
	void IPolarHM::compute(Mat & imIn, double lpScalar)
	{
		originalCopy = imIn.clone();
		Size imInSize = imIn.size();
		this->possibleResize(imInSize.width, imInSize.height); //resize if required
		Mat im;
		//cout << w << ", " << h << endl;
		int hw = w / 2, hh = h / 2;

		float logScalar = ll_lp_scalar(w, lpScalar);
		float oriScalar = h / 360.0f;

		GaussianBlur(imIn, im, Size(7, 7), 0, 0, BORDER_DEFAULT); //blur image
		for (int y = 1; y < imInSize.height - 1; y++) {
			for (int x = 1; x < imInSize.width - 1; x++) {
				Point2f d = IGHM::differential2(im, y, x);
				float mag = logMag(hw, hh, x, y) * logScalar;
				float ori = (atan2(d.y, d.x) * IGHM::r2d + 180.f) * oriScalar; //was x, y
				int magi = (int)mag;
				int orii = (int)ori;
				if (orii >= 0 && orii < w && magi >= 1 && magi < h)
				{
					if(ib(magi, orii)) at(magi, orii).push_back(Point2i(x, y));
				}
			}
		}
	}



	void IPolarHM::compute(Mat & imIn, int nw, int nh, double lpScalar)
	{
		originalCopy = imIn.clone();
		Size imInSize = imIn.size();
		this->possibleResize(nw, nh); //resize if required
		Mat im;

		int hw = w / 2, hh = h / 2;

		float logScalar = ll_lp_scalar(w, nw, lpScalar);
		float oriScalar = h / 360.0f;

		GaussianBlur(imIn, im, Size(7, 7), 0, 0, BORDER_DEFAULT); //blur image
		for (int y = 1; y < imInSize.height - 1; y++) {
			for (int x = 1; x < imInSize.width - 1; x++) {
				Point2f d = IGHM::differential2(im, y, x);
				float mag = logMag(hw, hh, x, y) * logScalar;
				float ori = (atan2(d.y, d.x) * IGHM::r2d + 180.f) * oriScalar;
				int magi = (int)mag;
				int orii = (int)ori;
				if (orii >= 0 && orii < w && magi >= 1 && magi < h)
				{
					at(magi, orii).push_back(Point2i(x, y));
				}
			}
		}
	}

	vector<Point2i> & IPolarHM::at(int y, int x)
	{
		return hm[y * w + x];
	}
	bool IPolarHM::ib(int y, int x)
	{
		return y >= 0 && x >= 0 && x < w && y < h;
	}
	Size IPolarHM::size()
	{
		return Size(w, h);
	}
	int IPolarHM::total()
	{
		int counter = 0;
		int si = w*h;
		for (int i = 0; i < si; i++)
		{
			counter += hm[i].size();
		}
		return counter;
	}
	Size IPolarHM::sizeOfOriginal()
	{
		return originalCopy.size();
	}
	Mat IPolarHM::countImage()
	{
		Mat rv = Mat::zeros(size(), CV_32FC1);
		int t = total();
		if (t == 0) return rv.clone();
		for (int y = 0; y < h; y++)
		{
			for (int x = 0; x < w; x++)
			{
				rv.at<float>(y, x) = at(y, x).size();
			}
		}
		ll_normalize(rv);
		return rv.clone();
	}

	Mat IPolarHM::hpImage()
	{
		Mat edgeIm = ll_edgeDetection(originalCopy);
		ll_normalize(edgeIm);
		Mat rv = Mat::zeros(size(), CV_32FC1);
		int t = total();
		if (t == 0) return rv.clone();
		for (int y = 0; y < h; y++)
		{
			for (int x = 0; x < w; x++)
			{
				vector<Point2i> & ref = at(y, x);
				for (int i = 0; i < ref.size(); i++)
				{
					rv.at<float>(y, x) += edgeIm.at<float>(ref[i]);
				}
			}
		}
		ll_normalize(rv);
		return rv.clone();
	}

	Mat IPolarHM::magImage()
	{
		Mat rv = Mat::zeros(size(), CV_32FC1);
		int t = total();
		if (t == 0) return rv.clone();
		for (int y = 0; y < h; y++)
		{
			for (int x = 0; x < w; x++)
			{
				vector<Point2i> & ref = at(y, x);
				for (int i = 0; i < ref.size(); i++)
				{
					rv.at<float>(y, x) += originalCopy.at<float>(ref[i]);
				}
			}
		}
		ll_normalize(rv);
		return rv.clone();
	}


	Mat IPolarHM::GenCountImage(Mat & imIn, double lpScalar)
	{

		Size imInSize = imIn.size();
		int w = imInSize.width, h = imInSize.height;

		int hw = w / 2, hh = h / 2;

		float logScalar = ll_lp_scalar(w, lpScalar);
		float oriScalar = h / 360.0f;
		Mat im;
		bool toNormalize = false;
		Mat rv = Mat::zeros(Size(360, 256), CV_32FC1);
		GaussianBlur(imIn, im, Size(7, 7), 0, 0, BORDER_DEFAULT);
		for (int y = 1; y < imInSize.height - 1; y++) {
			for (int x = 1; x < imInSize.width - 1; x++) {
				Point2f d = IGHM::differential2(im, y, x);
				float mag = logMag(hw, hh, x, y) * logScalar;
				float ori = (atan2(d.x, d.y) * IGHM::r2d + 180.f) * oriScalar;
				int magi = (int)mag;
				int orii = (int)ori;
				if (orii >= 0 && orii < w && magi >= 1 && magi < h)
				{
					toNormalize = true;
					rv.at<float>(magi, orii) += 1.f;
				}
			}
		}
		if (toNormalize)
			ll_normalize(rv);
		return rv.clone();
	}
	void IPolarHM::phaseCorrelateRS(Mat & image1, Mat & image2, double & rotation, double & scale, double lpScalar)
	{
		Mat a = image1.clone();
		Mat b = image2.clone();
		ll_UCF1_to_32F1(a);
		ll_UCF1_to_32F1(b);

		IPolarHM iphm1(a, lpScalar), iphm2(b, lpScalar);
		Mat magImage1 = iphm1.hpImage();
		Mat magImage2 = iphm2.hpImage();

		Mat h = ll_hanning_window(magImage1.size());
		magImage1 = magImage1.mul(h);
		magImage2 = magImage2.mul(h);


		Point2d peak = ll_phase_correlate(magImage1, magImage2);
		double w = (double)iphm1.size().width;
		peak.x = (peak.x) * (360.0 / w);
		peak.y /= ll_lp_scalar(iphm1.size().width, lpScalar);
		peak.y = exp(peak.y);
		rotation = peak.x;
		scale = peak.y;
	}

	void IPolarHM::phaseCorrelateRST(Mat & image1, Mat & image2, double & rotation, double & scale, Point2d & trans, double lpScalar)
	{
		Mat a = image1.clone();
		Mat b = image2.clone();
		ll_UCF1_to_32F1(a);
		ll_UCF1_to_32F1(b);

		IPolarHM iphm1(a, lpScalar), iphm2(b, lpScalar);
		Mat magImage1 = iphm1.hpImage();
		Mat magImage2 = iphm2.hpImage();

		Mat h = ll_hanning_window(magImage1.size());
		magImage1 = magImage1.mul(h);
		magImage2 = magImage2.mul(h);

		Point2d peak = ll_phase_correlate(magImage1, magImage2);
		double w = (double)iphm1.size().width;
		peak.x = (peak.x) * (360.0 / w);
		peak.y /= ll_lp_scalar(iphm1.size().width, lpScalar);
		peak.y = exp(peak.y);
		rotation = peak.x;
		scale = peak.y;

		ll_transform_image(a, a, rotation, scale, 0.0, 0.0);

		a = a.mul(h);
		b = b.mul(h);

		trans = ll_phase_correlate(a, b);
	}

	void IPolarHM::phaseCorrelateRST(Mat & image1, Mat & image2, long long & ms, double & rotation, double & scale, Point2d & trans, double lpScalar)
	{
		Mat a = image1.clone();
		Mat b = image2.clone();
		ll_UCF1_to_32F1(a);
		ll_UCF1_to_32F1(b);
		ms = 0;

		auto start = std::chrono::high_resolution_clock::now();
		IPolarHM iphm1(a, lpScalar), iphm2(b, lpScalar);
		Mat magImage1 = iphm1.hpImage();
		Mat magImage2 = iphm2.hpImage();

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		start = std::chrono::high_resolution_clock::now();

		Mat h = ll_hanning_window(magImage1.size());
		magImage1 = magImage1.mul(h);
		magImage2 = magImage2.mul(h);

		Point2d peak = ll_phase_correlate(magImage1, magImage2);
		elapsed = std::chrono::high_resolution_clock::now() - start;

		elapsed = std::chrono::high_resolution_clock::now() - start;
		ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();




		double w = (double)iphm1.size().width;
		peak.x = (peak.x) * (360.0 / w);
		peak.y /= ll_lp_scalar(magImage1.size().width, lpScalar);
		peak.y = exp(peak.y);
		rotation = peak.x;
		scale = peak.y;

		Mat bh = b.clone();
		bh = bh.mul(h);

		double error1, error2;
		Mat r1, r2;
		Point2d t1, t2;

		ll_transform_image(a, r1, rotation, scale, 0.0, 0.0);
		Mat hr1 = r1.clone();
		hr1 = hr1.mul(h);

		start = std::chrono::high_resolution_clock::now();
		t1 = ll_phase_correlate(hr1, bh);

		elapsed = std::chrono::high_resolution_clock::now() - start;
		ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

		ll_transform_image(r1, r1, 0.0, 1.0, t1.x, t1.y);
		error1 = ll_mse<float>(r1, b);

		ll_transform_image(a, r2, rotation + 180.0, scale, 0.0, 0.0);
		Mat hr2 = r2.clone();
		hr2 = hr2.mul(h);
		t2 = ll_phase_correlate(hr2, bh);
		ll_transform_image(r2, r2, 0.0, 1.0, t2.x, t2.y);
		error2 = ll_mse<float>(r2, b);

		if (error1 < error2)
		{
			trans = t1;
		}
		else
		{
			trans = t2;
			rotation += 180.0;
		}


	}

	/*void IPolarHM::phaseCorrelateRST_NG(Mat & image1, Mat & image2, long long & ms, double & rotation, double & scale, Point2d & trans, double lpScalar)
	{
		Mat h = ll_hanning_window(image1.size());
		Mat a = image1.clone();
		Mat b = image2.clone();
		ll_UCF1_to_32F1(a);
		ll_UCF1_to_32F1(b);
		ms = 0;

		auto start = std::chrono::high_resolution_clock::now();
		Mat ottoA, __a, ottoB, __b;
		ll_fft::fft_magnitude_phase(a, ottoA, __a);
		ll_fft::fft_magnitude_phase(b, ottoB, __b);
		log(ottoA, ottoA);
		log(ottoB, ottoB);
		ll_normalize(ottoA);
		ll_normalize(ottoB);
		IPolarHM iphm1(ottoA), iphm2(ottoB);
		Mat magImage1 = iphm1.hpImage();
		Mat magImage2 = iphm2.hpImage();

		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		start = std::chrono::high_resolution_clock::now();


		//magImage1 = magImage1.mul(h);
		//magImage2 = magImage2.mul(h);

		Point2d peak = ll_phase_correlate(magImage1, magImage2);
		elapsed = std::chrono::high_resolution_clock::now() - start;

		elapsed = std::chrono::high_resolution_clock::now() - start;
		ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();




		double w = (double)iphm1.size().width;
		peak.x = (-peak.x) * (360.0 / w);
		peak.y /= IPolarHM::getScalar(iphm1.size().width);
		peak.y = exp(peak.y);
		rotation = peak.x;
		scale = 1.0 / peak.y;
		//cout << rotation << " " << scale << endl;

		Mat bh = b.clone();
		bh = bh.mul(h);

		double error1, error2;
		Mat r1, r2;
		Point2d t1, t2;

		ll_transform_image(a, r1, rotation, scale, 0.0, 0.0);
		Mat hr1 = r1.clone();
		hr1 = hr1.mul(h);

		start = std::chrono::high_resolution_clock::now();
		t1 = ll_phase_correlate(hr1, bh);

		elapsed = std::chrono::high_resolution_clock::now() - start;
		ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

		ll_transform_image(r1, r1, 0.0, 1.0, t1.x, t1.y);
		error1 = ll_mse<float>(r1, b);

		ll_transform_image(a, r2, rotation + 180.0, scale, 0.0, 0.0);
		Mat hr2 = r2.clone();
		hr2 = hr2.mul(h);
		t2 = ll_phase_correlate(hr2, bh);
		ll_transform_image(r2, r2, 0.0, 1.0, t2.x, t2.y);
		error2 = ll_mse<float>(r2, b);

		if (error1 < error2)
		{
			trans = t1;
		}
		else
		{
			trans = t2;
			rotation += 180.0;
		}


	}*/

	Mat IPolarHM::HPImage(Mat & input, long long & ms, double scalar)
	{
		Size _size = input.size();
		int w = _size.width, h = _size.height;
		Mat ret = Mat::zeros(_size, CV_32FC1);
		ms = 0;
		Mat im;

		int hw = w / 2, hh = h / 2;

		float logScalar = ll_lp_scalar(w, scalar);
		float oriScalar = h / 360.0f;

		GaussianBlur(input, im, Size(7, 7), 0, 0, BORDER_DEFAULT); //blur image

		auto start = std::chrono::high_resolution_clock::now();

		for (int y = 1; y < h - 1; y++) {
			for (int x = 1; x < w - 1; x++) {
				Point2f d = IGHM::differential2(im, y, x);
				float mag = logMag(hw, hh, x, y);
				if (mag <= 5.0) continue;
				mag *= logScalar;
				float ori = (atan2(d.y, d.x) * IGHM::r2d + 180.f) * oriScalar;
				int magi = (int)mag;
				int orii = (int)ori;
				if (orii >= 0 && orii < w && magi >= 1 && magi < h)
				{
					ret.at<float>(magi, orii) += d.x*d.x + d.y*d.y;
				}
			}
		}
		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		ms = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		ll_normalize(ret);
		return ret.clone();
	}

	void IPolarHM::phaseCorrelateRSTf(Mat & image1, Mat & image2, long long & ms, double & rotation, double & scale, Point2d & trans, double lpScalar)
	{
		Mat a = image1.clone();
		Mat b = image2.clone();
		ll_UCF1_to_32F1(a);
		ll_UCF1_to_32F1(b);
		ms = 0;

		//auto start = std::chrono::high_resolution_clock::now();
		Mat magImage1 = HPImage(a, ms, lpScalar);
		Mat magImage2 = HPImage(b, ms, lpScalar);
		//IPolarHM iphm1(a), iphm2(b);
		//Mat magImage1 = iphm1.hpImage();
		//Mat magImage2 = iphm2.hpImage();

		//auto elapsed = std::chrono::high_resolution_clock::now() - start;
		//ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		auto start = std::chrono::high_resolution_clock::now();

		Mat h = ll_hanning_window(magImage1.size());
		magImage1 = magImage1.mul(h);
		magImage2 = magImage2.mul(h);

		Point2d peak = ll_phase_correlate(magImage1, magImage2);
		auto elapsed = std::chrono::high_resolution_clock::now() - start;

		elapsed = std::chrono::high_resolution_clock::now() - start;
		ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();




		double w = (double)magImage1.size().width;
		peak.x = (peak.x) * (360.0 / w);
		peak.y /= ll_lp_scalar(magImage1.size().width, lpScalar);
		peak.y = exp(peak.y);
		rotation = peak.x;
		scale = peak.y;

		Mat bh = b.clone();
		bh = bh.mul(h);

		double error1, error2;
		Mat r1, r2;
		Point2d t1, t2;

		ll_transform_image(a, r1, rotation, scale, 0.0, 0.0);
		Mat hr1 = r1.clone();
		hr1 = hr1.mul(h);

		start = std::chrono::high_resolution_clock::now();
		t1 = ll_phase_correlate(hr1, bh);

		elapsed = std::chrono::high_resolution_clock::now() - start;
		ms += std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

		ll_transform_image(r1, r1, 0.0, 1.0, t1.x, t1.y);
		error1 = ll_mse<float>(r1, b);

		ll_transform_image(a, r2, rotation + 180.0, scale, 0.0, 0.0);
		Mat hr2 = r2.clone();
		hr2 = hr2.mul(h);
		t2 = ll_phase_correlate(hr2, bh);
		ll_transform_image(r2, r2, 0.0, 1.0, t2.x, t2.y);
		error2 = ll_mse<float>(r2, b);

		if (error1 < error2)
		{
			trans = t1;
		}
		else
		{
			trans = t2;
			rotation += 180.0;
		}


	}
}

