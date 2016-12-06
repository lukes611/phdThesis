#pragma once


/*

implementation of various measurment algorithms for 3D data
author : luke lincoln @ lukes611@gmail.com

requires: R3, VMat

*/
#include <vector>
#include "../basics/R3.h"
#include "../basics/VMatF.h"
#include "../basics/Pixel3DSet.h"


namespace ll_measure
{
//to-do:
//Hausdorff distance
double hausdorff(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b);
double hausdorff(ll_pix3d::Pixel3DSet & a, ll_pix3d::Pixel3DSet & b);
//average error
double avg(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b);
double avg(ll_pix3d::Pixel3DSet & a, ll_pix3d::Pixel3DSet & b);
double avg(VMat & a, VMat & b, float threshold = 0.05f);
//mse
double mse(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b);
double mse_1way(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b);
double mse(ll_pix3d::Pixel3DSet & a, ll_pix3d::Pixel3DSet & b);
double mse_1way(ll_pix3d::Pixel3DSet & a, ll_pix3d::Pixel3DSet & b);
double mse(VMat & a, VMat & b, float threshold = 0.05f);
//% matches
double percentMatch(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b, double minError = 0.8f);
double percentMatch_1way(std::vector<ll_R3::R3> & a, std::vector<ll_R3::R3> & b, double minError = 0.8f);

double percentMatch(ll_pix3d::Pixel3DSet & a, ll_pix3d::Pixel3DSet & b, double minError = 0.8f);
double percentMatch_1way(ll_pix3d::Pixel3DSet & a, ll_pix3d::Pixel3DSet & b, double minError = 0.8f);


double percentMatch(VMat & a, VMat & b, float threshold = 0.05f);


void error_metrics(ll_pix3d::Pixel3DSet & a, ll_pix3d::Pixel3DSet & b, double & hausdorff_error, double & mse_error, double & percent_match, double minError = 0.8f);
void error_metrics_1way(ll_pix3d::Pixel3DSet & a, ll_pix3d::Pixel3DSet & b, double & hausdorff_error, double & mse_error, double & percent_match, double minError = 0.8f);


Pix3D render(SIObj & objectIn, int volumeWidth, float focalDistance = 500.0f);
Pix3D render(ll_pix3d::Pixel3DSet & objectIn, int volumeWidth, float focalDistance);

}
