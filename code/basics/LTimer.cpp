#include "LTimer.h"
using namespace std;
using namespace chrono;

high_resolution_clock::time_point LTimer::getTime()
{
	return high_resolution_clock::now();
}

LTimer::LTimer()
{
	reset();
}

void LTimer::reset()
{
	_state = 0;
}

void LTimer::start()
{
	if(_state != 0) return;
	_state = 1;
	_start = getTime();
}
void LTimer::stop()
{
	_end = getTime();
	if (_state != 1) return;
	_state = 2;
}
long long LTimer::getMicroSeconds()
{
	if (_state != 2) return 0;
	nanoseconds elapsed = _end - _start;
	return duration_cast<microseconds>(elapsed).count();
}
double LTimer::getSeconds()
{
	return getMicroSeconds() / 1000000.0;
}