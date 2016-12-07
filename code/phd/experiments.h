#pragma once

#include <string>
#include <vector>
#include <map>
#include "../basics/Pixel3DSet.h"

//#define AT_UNI //i am at uni

#ifdef _WIN32 //if on windows
#define HASGL //use openGL

#ifdef AT_UNI //if at uni and windows

#define LCPPDATA_DIR "C:/lcppdata"
#define DESKTOP_DIR "C:/Users/s2807774/Desktop"

#else //on windows laptop

#define LCPPDATA_DIR "C:/lcppdata"
#define DESKTOP_DIR "C:/Users/luke/Desktop"
#define EXPS_DIR "C:/Users/luke/Documents/Visual Studio 2012/Projects/PhD 16 Sem2/PhD 16 Sem2/experiments"


#define HASCUDA

#endif


#else //using linux on my laptop



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


#ifdef HASGL
	//set Pixel3DSet object in LLPointers by key "object"
	void viewPixel3DSet();
#endif
}
