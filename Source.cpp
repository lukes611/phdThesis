#include <string>
#include <iostream>
#include "code/basics/SIObj.h"
#include "code/basics/locv3.h"
#include "code/basics/llCamera.h"
#include "code/phd/experiments.h"
#include "code/basics/VMatF.h"
#include "code/basics/locv_algorithms.h"
#include "code/pc/TheVolumePhaseCorrelator.h"
#include "code/phd/Lpcr.h"
#include "code/pc/LFFTW.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_siobj;
using namespace ll_experiments;

/*
todo in fftw:

LLGPU_phase_correlate
    fft3d
    multiply-spectrums
    ifft3d
    find the peak
phase_correlate_rst


*/

#include <fftw3.h>

void fft3D(VMat & volume, VMat & realOut, VMat & imagOut)
{
    realOut = VMat(volume.s);
    imagOut = VMat(volume.s);

    fftw_complex * in   = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * volume.s3);
    fftw_complex * out  = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * volume.s3);

    for(int i = 0; i < volume.s3; i++)
    {
        in[i][0] = volume.data[i];
        in[i][1] = 0.0f;
    }

    fftw_plan plan = fftw_plan_dft_3d(volume.s, volume.s, volume.s, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    fftw_execute(plan);

    for(int i = 0; i < volume.s3; i++)
    {
        realOut.data[i] = out[i][0];
        imagOut.data[i] = out[i][1];
    }

    fftw_free(in);
    fftw_free(out);
}

void ifft3D(VMat & output, VMat & real, VMat & imag)
{
    output = VMat(real.s);

    fftw_complex * in   = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * real.s3);
    fftw_complex * out  = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * real.s3);

    for(int i = 0; i < output.s3; i++)
    {
        in[i][0] = real.data[i];
        in[i][1] = imag.data[i];
    }

    fftw_plan plan = fftw_plan_dft_3d(real.s, real.s, real.s, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);

    fftw_execute(plan);

    for(int i = 0; i < output.s3; i++)
    {
        output.data[i] = out[i][0];
    }

    fftw_free(in);
    fftw_free(out);
}


int main(){


	ll_pix3d::CapturePixel3DSet reader = ll_experiments::openData("Apartment.Texture.rotate", 3);

	Pixel3DSet p1;
	Pixel3DSet p2;

	{
		Pix3D p;
		reader.read_frame(p, 0);
		p1 = p;
	}
	{
		//Pix3D p;
		//reader.read_frame(p, 10);
		//p2 = p;
	}

    VMat v(128, p1, 0.0f, false);

    VMat re, im;
    fft3D(v, re, im);

    VMat v2;
    ifft3D(v2, re, im);

    v2.save_obj(string(DESKTOP_DIR) + "/c2.obj" , 128, 0.2f);






	return 0;
}

