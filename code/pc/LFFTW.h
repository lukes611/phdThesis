/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description:
		The .h file for LFFTW which is an a set of functions for performing various actions relating to 3d-ffts using
		the CPU on a VMatF object with the fftw library. No GPU required :D

	depends on: Pixel3DSet, R3, VMatF

	requires FFTW
*/

#pragma once

#include "../phd/experiments.h"
#include "../basics/VMatF.h"
#include "../basics/R3.h"
#include "../basics/Pixel3DSet.h"



namespace llfftw
{
#ifdef HASFFTW


void fft3D(VMat & volume, VMat & realOut, VMat & imagOut);
void ifft3D(VMat & output, VMat & real, VMat & imag);

//log-polar function
void logPolar(VMat & v);

//fft-mag-swap-quads
void fftMagSwapQuads(VMat & in, VMat & mag);

void multiplySpectrums(VMat & re1, VMat & im1, VMat & re2, VMat & im2); //multiplies spectrums, output is in re1,im1
cv::Point3i phaseCorrelate(VMat & v1, VMat & v2);
void phaseCorrelate_rst(VMat & vol1, VMat & vol2, float & rotation, float & scale, cv::Point3i & translation, bool hanning_window_on = false);

#endif



}
