/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description: 
		Contains the extern references to the functionality on the GPU I wrote in the file matrixMul.cu

	depends on: VMat, R3
*/

#pragma once

#include "../basics/VMatF.h"
#include "../basics/R3.h"

extern
bool LLGPU_fft3d(VMat & v, VMat & re, VMat & im);
extern
bool LLGPU_ifft3d(VMat & re, VMat & im, VMat & out);
extern
bool LLGPU_fft3d_mag_swap_quads(VMat & v, VMat & output);
extern
bool LLGPU_transform(VMat & v, ll_R3::R3 rotation, float scale, ll_R3::R3 translation);
extern
bool LLGPU_transform(VMat & v, Mat & transformation_matrix);
extern
bool LLGPU_phase_correlate(VMat & v1, VMat & v2, Point3i & rv);
extern
bool LLGPU_phase_correlate_rst(VMat vol1, VMat vol2, float & rotation, float & scale, Point3i & translation, bool hanning_window_on = false);
extern
bool LLGPU_log(VMat & v);
extern
bool LLGPU_log_polar(VMat & v);
extern
bool LLGPU_log_only(VMat & v);
