#pragma once


/*

implementation of various measurment algorithms for 3D data
author : luke lincoln @ lukes611@gmail.com

requires: R3, VMat

*/
#include <vector>
#include "../basics/R3.h"
#include "../basics/VMatF.h"


namespace ll_measure
{
//to-do:
//Hausdorff distance
double hausdorff(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b);
//average error 
double avg(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b);
double avg(VMat & a, VMat & b, float threshold = 0.05f);
//mse
double mse(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b);
double mse_1way(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b);
double mse(VMat & a, VMat & b, float threshold = 0.05f);
//% matches
double percentMatch(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b, double minError = 0.5f);
double percentMatch_1way(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b, double minError = 0.5f);
double percentMatch(VMat & a, VMat & b, float threshold = 0.05f);
}