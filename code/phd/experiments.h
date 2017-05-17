#pragma once

#include <string>
#include <vector>
#include <map>
#include "../basics/Pixel3DSet.h"

#define AT_UNI //i am at uni

#ifdef _WIN32 //if on windows



#define HASGL //use openGL

#ifdef AT_UNI //if at uni and windows

#define LCPPDATA_DIR "C:/lcppdata"
#define DESKTOP_DIR "C:/Users/s2807774/Desktop"
#define EXPS_DIR "C:/Users/s2807774/Documents/Visual Studio 2015/Projects/PhD project 16/Ocv3 Basic with ll_ libraries/experiments"
#define HASFFTW

#else //on windows laptop

#define LCPPDATA_DIR "C:/lcppdata"
#define DESKTOP_DIR "C:/Users/luke/Desktop"
#define EXPS_DIR "C:/Users/luke/Documents/Visual Studio 2012/Projects/PhD 16 Sem2/PhD 16 Sem2/experiments"


#define HASCUDA

#endif


#else //using linux on my laptop

#define DESKTOP_DIR "/home/luke/Desktop"
#define LCPPDATA_DIR "/home/luke/lcppdata"
#define EXPS_DIR "/home/luke/gitProjects/phdThesis/experiments"

#define HASFFTW



#endif


















namespace ll_experiments
{



	class LLPointers{
		static std::map<std::string, void *> * access();
	public:
		static bool has(std::string key);
		template<class T>
		static T get(std::string key){
			std::map<std::string, void*> * ptr = LLPointers::access();
			T * dataPt = (T*)ptr->operator[](key);
			return *dataPt;
		}
		template<class T>
		static T * getPtr(std::string key){
			std::map<std::string, void*> * ptr = LLPointers::access();
			T * dataPt = (T*)ptr->operator[](key);
			return dataPt;
		}
		template<class T>
		static void setPtr(std::string key, T * dataPt){
			std::map<std::string, void*> * ptr = LLPointers::access();
			ptr->operator[](key) = (void*)dataPt;
		}
	};


	void viewVideo(std::string name, bool viewColor = true, bool viewDepth = false, bool viewVD = false, int wks = -1);

	std::vector<int> rng(int to);
	std::vector<int> rng(int from, int to);
	std::vector<int> rng(int from, int to, int inc);

	bool fileIsEmpty(std::string fileName);
    void appendData(std::string fileName, std::string header, std::string data, bool includeNewLine = true);

	ll_pix3d::CapturePixel3DSet openData(std::string name, int numFrames);

	/*
	A noise generator
	*/
	class NoiseGenerator{
	public:
		std::vector<cv::Point3f> signal, noise;

		NoiseGenerator(bool seedAtBeginning = true);
		~NoiseGenerator();
		double randomNumber(); //returns a random number between 0 and 1
		cv::Point3f randomPoint(); //returns a random point each coord is between 0 and 1
		void seed(); //seeds the randomNumbers
		double randomNumber(double range); //returns a random number between -.5x and .5x
		cv::Point3f randomPoint(double range); //returns a random point each coord is between -.5x and .5x

		cv::Point3f getNoise(cv::Point3f signal, double range); //returns noise to be added to the data
		ll_R3::R3 getNoise(ll_R3::R3 signal, double range); //returns noise to be added to the data

		cv::Point3f stdDev(std::vector<cv::Point3f> & data);
		cv::Point3f mean(std::vector<cv::Point3f> & data);
		cv::Point3f stdDevNoise();
		cv::Point3f stdDevSignal();

		void getSNR(cv::Point3f & out);
		void getSNR(double & out);


	};

	ll_pix3d::Pixel3DSet getNoisedVersion(ll_pix3d::Pixel3DSet & input, double noiseRange, double & snrOut);
	ll_pix3d::Pix3D getNoisedVersion(ll_pix3d::Pix3D & input, double noiseRange, double & snrOut);


#ifdef HASGL
	//set Pixel3DSet object in LLPointers by key "object"
	void viewPixel3DSet();
#endif
}
