#include "locv3.h"

namespace llcv3T
{

	template <class T>
	cv::Mat combine_images(cv::Mat & im1, cv::Mat & im2, bool horizontal = true)
	{
		if(horizontal)
		{
			cv::Size s(im1.size().width + im2.size().width, max(im1.size().height, im2.size().height));
			cv::Mat rv = Mat::zeros(s, im1.type());
			for(int y = 0; y < s.height; y++)
			{
				if(y < im1.size().height)
				{
					for(int x = 0; x < im1.size().width; x++)
						rv.at<T>(cv::Point2i(x,y)) = im1.at<T>(cv::Point2i(x,y));
				}

				if(y < im2.size().height)
				{
					for(int x = 0; x < im2.size().width; x++)
						rv.at<T>(cv::Point2i(x+im1.size().width,y)) = im2.at<T>(cv::Point2i(x,y));
				}
	
	
			}
			return rv.clone();
		}
		cv::Size s(max(im1.size().width, im2.size().width), im1.size().height + im2.size().height);
		cv::Mat rv = cv::Mat::zeros(s, im1.type());
		for(int y = 0; y < s.height; y++)
		{
			for(int x = 0; x < im1.size().width; x++)
					rv.at<T>(cv::Point2i(x,y)) = im1.at<T>(cv::Point2i(x,y));
			for(int x = 0; x < im2.size().width; x++)
				rv.at<T>(cv::Point2i(x,y + im1.size().height)) = im2.at<T>(cv::Point2i(x,y));
		}
		return rv.clone();
	}

	template <class T>
	void draw_matches(cv::Mat & im, cv::Size & left_offset, std::vector<T> & p1, std::vector<T> & p2, int limit = -1)
	{
		if(p1.size() != p2.size()) return;
		unsigned int len = 0;
		if(limit == -1)
		{
			len = static_cast<unsigned int>(p1.size());
		}else
		{
			len = min(static_cast<unsigned int>(p1.size()), static_cast<unsigned int>(limit));
		}
		for(unsigned int i = 0; i < len; i++)
		{
			double r = rand() / (double)RAND_MAX; r *= 255.0;
			double g = rand() / (double)RAND_MAX; g *= 255.0;
			double b = rand() / (double)RAND_MAX; b *= 255.0;
			cv::Point2i _p1((int)round(p1[i].x), (int)round(p1[i].y));
			cv::Point2i _p2((int)round(p2[i].x), (int)round(p2[i].y));
			cv::line(im, _p1, _p2 + cv::Point2i(left_offset.width,0), cv::Scalar(r,g,b));
		}
	}


	template <class T>
	cv::Mat draw_matchesOF(cv::Mat & im, std::vector<T> & p1, std::vector<T> & p2, int limit = -1)
	{
		cv::Mat rv = im.clone();
		if(p1.size() != p2.size()) return rv.clone();
		unsigned int len = 0;
		
		if(limit == -1)
		{
			len = static_cast<unsigned int>(p1.size());
		}else
		{
			len = min(static_cast<unsigned int>(p1.size()), static_cast<unsigned int>(limit));
		}
		for(unsigned int i = 0; i < len; i++)
		{
			double r = rand() / (double)RAND_MAX; r *= 255.0;
			double g = rand() / (double)RAND_MAX; g *= 255.0;
			double b = rand() / (double)RAND_MAX; b *= 255.0;
			cv::Point2i _p1((int)round(p1[i].x), (int)round(p1[i].y));
			cv::Point2i _p2((int)round(p2[i].x), (int)round(p2[i].y));
			cv::line(rv, _p1, _p2, cv::Scalar(r,g,b));
		}
		return rv.clone();
	}
}

