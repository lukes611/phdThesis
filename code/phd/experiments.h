#pragma once

#include <string>
#include <vector>

namespace ll_experiments
{
	void viewVideo(std::string name, bool viewColor = true, bool viewDepth = false, bool viewVD = false, int wks = -1);

	std::vector<int> rng(int to);
	std::vector<int> rng(int from, int to);
	std::vector<int> rng(int from, int to, int inc);
}