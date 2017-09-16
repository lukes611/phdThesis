#pragma once

/**
 * includes
 */

#include "../phd/experiments.h"
#ifdef HASGL
#include "../basics/ll_gl.h"
#endif
#include <iostream>
#include <string>
#include <vector>
#include "../basics/locv3.h"
#include "../phd/experiments.h"
#include "../basics/R3.h"
#include "../basics/llCamera.h"
#include "../basics/VMatF.h"
#include "../basics/LTimer.h"
#include "../phd/Licp.h"
#include "../phd/measurements.h"
#include "../phd/fmRansac.h"
#include "../pc/TheVolumePhaseCorrelator.h"
#include "../phd/Lpcr.h"
#include "../script/LScript.h"
#include "../phd/LSift.h"
#include "../phd/LCamExperiments.h"
#include "../basics/BitReaderWriter.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
using namespace ll_experiments;

#define HASFFTW

//single experiment, only tests one algorithm at a time
void exp1(string name, vector<int> frames);

//saves to version 2.0 data file
void saveV20(string data_name, string alg_name, int frame1, int frame2, float seconds, float hd);
void quantitativeExperiment20(string algorithm_name, string data_name, vector<int> frames);
void saveKitti(string data_name, string alg_name, int frame1, int frame2, float seconds, float hd);
void quantitativeExperimentKitti10(string algorithm_name, string data_name, vector<int> frames);

void qualitativeExperiment(string algorithm_name, string data_name, vector<int> frames);




