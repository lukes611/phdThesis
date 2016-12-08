#include "TheVolumePhaseCorrelator.h"
#include <math.h>

using namespace cv;

namespace ll_volume_gpu
{
	
	void transform(VMat & vol, Mat & m)
	{
		LLGPU_transform(vol, m);
	}

	void transform_volume(VMat & vol, float rx, float ry, float rz, float sc, float tx, float ty, float tz)
	{
		LLGPU_transform(vol, R3(rx, ry, rz), sc, R3(tx, ty, tz));
	}

	void phase_correlate_rst(VMat & v1, VMat & v2, float & rotation, float & scale, R3 & translation, bool normalize_color)
	{
		if(normalize_color)
		{
			v1.normalize();
			v2.normalize();
		}
		Point3i t;
		LLGPU_phase_correlate_rst(v1, v2, rotation, scale, t, false);
		translation = R3((float)t.x, (float)t.y, (float)t.z);
	}

	Mat phase_correlate_rst(VMat & v1, VMat & v2, bool normalize_color)
	{
		R3 translation;
		float scale = 1.0f, rotation = 0.0f;
		if(normalize_color)
		{
			v1.normalize();
			v2.normalize();
		}
		Point3i t;
		LLGPU_phase_correlate_rst(v1, v2, rotation, scale, t, false); 
		translation = R3((float)t.x, (float)t.y, (float)t.z);
		return VMat::transformation_matrix(v1.s, 0.0f, rotation, 0.0f, scale, translation.x, translation.y, translation.z).clone();
	}

	Mat pca_phase_correlate_rst(VMat & v1_in, VMat & v2_in)
	{
		//set identity
		Mat rv = Mat::eye(Size(4,4), CV_32FC1);
		//copy both
		VMat v1 = v1_in;
		VMat v2 = v2_in;
		
		//transform both to proper up space
		Mat m1 = v1.pca_correct_up();
		Mat m2 = v2.pca_correct_up();
		Mat m2i = m2.inv();
		
		//pcrst
		Mat pcm; float rotation, scale; R3 translation;
		phase_correlate_rst(v1, v2, rotation, scale, translation); 
		pcm = VMat::transformation_matrix(v1.s, 0.0f, rotation, 0.0f, scale, translation.x, translation.y, translation.z);
		//un-transform v1 according to the return value of v2's up correction

		//return proper matrix
		return (m2i * pcm * m1);
	}

	Mat phase_correlate_rst_two_points(VMat & v1_in, VMat & v2_in, R3 match_1[2], R3 match_2[2])
	{
		//copy both
		VMat s1 = v1_in;
		VMat s2 = v2_in;
		Mat m;
		
		Mat m1 = VMat::new_up_mat((match_2[0]-match_1[0]).unit(), match_1[0]);
		Mat m2 = VMat::new_up_mat((match_2[1]-match_1[1]).unit(), match_1[1]);
		s1.transform_volume_forward(m1);
		s2.transform_volume_forward(m2);
		Mat rv = m1;
		m2 = m2.inv();
		m = ll_volume_gpu::phase_correlate_rst(s1, s2, true);
		//s1.transform_volume_forward(m);
		rv = m * rv;
		rv = m2 * rv;
		//s1.transform_volume_forward(m2);
		//s2.transform_volume(m2);
		return rv;
	}

	R3 phase_correlate(VMat & v1, VMat & v2)
	{
		assert(v1.s == v2.s);
		Point3i rv;
		LLGPU_phase_correlate(v1, v2, rv);
		Point3i p = VMat::filter_pc(rv, v1.s);
		return R3((float)p.x, (float)p.y, (float)p.z);
	}

	void fft_mag(VMat & v1, VMat & mag)
	{
		LLGPU_fft3d_mag_swap_quads(v1, mag);
	}

	void log(VMat & v)
	{
		LLGPU_log(v);
	}

	void log_polar(VMat & s)
	{
		LLGPU_log_polar(s);
	}
}

namespace ll_volume_partial
{
	float luke_rotation_estimation_method(VMat & v1, VMat & v2, Size s)
	{
		VMat m1 = v1.s, m2 = v1.s;
		
		ll_volume_gpu::fft_mag(v1, m1);
		ll_volume_gpu::fft_mag(v2, m2);
		m1.normalize();
		m2.normalize();
		float rv = m1.fast_y_rotation_estimation_1a(m2, s, 2.0f);
		return rv;
	}

	void phase_correlate_rt_lrem(VMat & v1, VMat & v2, float & rotation, R3 & translation)
	{
		rotation = luke_rotation_estimation_method(v1, v2);
		VMat tmp = v1;
		ll_volume_gpu::transform_volume(tmp, 0.0f, rotation, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
		translation = ll_volume_gpu::phase_correlate(tmp, v2);
	}

	void phase_correlate_rt_luke(VMat & v1, VMat & v2, float & rotation, R3 & translation, Size s)
	{
		rotation = luke_rotation_estimation_method(v1, v2);
		VMat tmp = v1;
		ll_volume_gpu::transform_volume(tmp, 0.0f, rotation, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
		translation = tmp.luke_fast_translation_estimation_2a(v2, s);
	}

	Mat phase_correlate_rt_luke(VMat & v1, VMat & v2, Size s)
	{
		float rotation;
		R3 translation;
		rotation = luke_rotation_estimation_method(v1, v2);
		VMat tmp = v1;
		ll_volume_gpu::transform_volume(tmp, 0.0f, rotation, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
		translation = tmp.luke_fast_translation_estimation_2a(v2, s);
		return VMat::transformation_matrix(v1.s, 0.0f, rotation, 0.0f, 1.0f, translation.x, translation.y, translation.z).clone();
	}

	Mat pca_lukes_pc_rt(VMat & v1, VMat & v2, Size s)
	{
		Mat m1 = VMat::pca(v1, v2);
		VMat tmp = v1.clone();
		tmp.transform_volume_forward(m1);
		Mat m2; float rotation; R3 translation;
		ll_volume_partial::phase_correlate_rt_luke(tmp, v2, rotation, translation, s); 
		float half = v1.s / 2.0f;
		m2 = Pixel3DSet::transformation_matrix(0.0f, rotation, 0.0f, 1.0f, translation.x, translation.y, translation.z, R3(half, half, half));
		m2 *= m1;
		return m2.clone();
	}
}




	



