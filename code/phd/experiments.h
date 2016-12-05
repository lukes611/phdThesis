#pragma once

#include <string>
#include <vector>
#include <map>

//#define HASGL


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


#ifdef HASGL
	//set Pixel3DSet object in LLPointers by key "object"
	void viewPixel3DSet();
#endif
}
