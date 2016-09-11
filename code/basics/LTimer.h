#pragma once
#include <chrono>

//linux&win32
//depends on modern c++

class LTimer {

public:
	static std::chrono::high_resolution_clock::time_point getTime();
	LTimer();
	void start();
	void stop();
	long long getMicroSeconds();
	double getSeconds();
	void reset();
private:
	std::chrono::high_resolution_clock::time_point _start, _end;
	long long _time;
	int _state;
};