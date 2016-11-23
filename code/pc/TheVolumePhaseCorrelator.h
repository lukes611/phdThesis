/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description:
		The .h file for TheVolumePhaseCorrelator which is an object enclosing a set of functions for performing various actions using
		the GPU on a VMatF object

	depends on: Pixel3DSet, R3, ExternGPUPrograms.h, VMatF, locv_algorithms.h

	*requires GPU
*/

#ifndef TheVolumePhaseCorrelator_H


#define TheVolumePhaseCorrelator_H

#include "../gpu/ExternGPUPrograms.h"
#include "../basics/VMatF.h"
#include "../basics/locv_algorithms.h"




namespace ll_volume_gpu
{
	//performs the transform based on m!
	void transform(VMat & vol, cv::Mat & m);

	//performs the transform based on m.inv()!
	void transform_volume(VMat & vol, float rx, float ry, float rz, float sc, float tx, float ty, float tz);

	void phase_correlate_rst(VMat & v1, VMat & v2, float & rotation, float & scale, R3 & translation, bool normalize_color = false);

	cv::Mat phase_correlate_rst(VMat & v1, VMat & v2, bool normalize_color = false);

	cv::Mat pca_phase_correlate_rst(VMat & v1, VMat & v2);

	//performs basic phase correlation giving translation params
	R3 phase_correlate(VMat & v1, VMat & v2);

	//performs fft on volume, and retrieves the magnitude volume from PC, where quadrants are swapped
	void fft_mag(VMat & v1, VMat & mag);

	//runs each voxel element through a log function, edits v
	void log(VMat & v);

	//performs log polar transform to s on the gpu
	void log_polar(VMat & s);

	cv::Mat pca_phase_correlate_rst(VMat & v1, VMat & v2);

	//use two registered points along with phase correlation to align two volumes according to scale, translation and rotation
	cv::Mat phase_correlate_rst_two_points(VMat & v1_in, VMat & v2_in, ll_R3::R3 match_1[2], ll_R3::R3 match_2[2]);

}


namespace ll_volume_partial
{
	//generates the magnitude of the fft of v1
	void fft_mag(VMat & v1, VMat & mag, bool swap_quads = true);

	//performs a partial gpu/cpu version of phase correlation against rotation, scale and translation
	void phase_correlate_rst(VMat & v1, VMat & v2, float & rotation, float & scale, R3 & translation);

	//my rotation estimation procedure, this procedure estimated the y-axis rotation between v1 and v2
	//despite the fact they can also be seperated by scale and translation
	float luke_rotation_estimation_method(VMat & v1, VMat & v2, cv::Size s = cv::Size(512, 512));



	//finds the rotation and translation params between two volumes using luke_rotation_estimation_method()
	void phase_correlate_rt_lrem(VMat & v1, VMat & v2, float & rotation, R3 & translation);

	//same as above function but works with rotation, translation AND scale
	void phase_correlate_rst_lrem(VMat & v1, VMat & v2, float & rotation, float & scale, R3 & translation);

	void phase_correlate_rt_luke(VMat & v1, VMat & v2, float & rotation, R3 & translation, cv::Size s = cv::Size(512, 512));

	cv::Mat phase_correlate_rt_luke(VMat & v1, VMat & v2, cv::Size s = cv::Size(512, 512));

	//coputes the registration matrix between v1 and v2 using my method and pca
	cv::Mat pca_lukes_pc_rt(VMat & v1, VMat & v2, cv::Size s = cv::Size(512, 512));

}




#endif
