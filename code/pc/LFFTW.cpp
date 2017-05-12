#include "LFFTW.h"

#define HASFFTW

#ifdef HASFFTW

#include <fftw3.h>
using namespace cv;

namespace llfftw
{

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

void multiplySpectrums(VMat & re1, VMat & im1, VMat & re2, VMat & im2)
{
    for(int i = 0; i < re1.s3; i++)
    {
        float R1 =  re1.data[i];
        float I1 =  im1.data[i];
        float R2 =  re2.data[i];
        float I2 = -im2.data[i];

        float TR = R1*R2 - I1*I2;
        float TI = I1*R2 + R1*I2;


        float mag = sqrt(TR*TR + TI*TI);

        re1.data[i] = TR / mag;
        im1.data[i] = TI / mag;
    }
}


Point3i phaseCorrelate(VMat & _v1, VMat & _v2)
{
    //fft3d both
    fftw_complex * v1   = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * _v1.s3);
    fftw_complex * v2   = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * _v2.s3);
    fftw_complex * f1  = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * _v1.s3);
    fftw_complex * f2  = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * _v2.s3);

    //copy data in
    for(int i = 0; i < _v1.s3; i++)
    {
        v1[i][0] = _v1.data[i]; v1[i][1] = 0.0f;
        v2[i][0] = _v2.data[i]; v2[i][1] = 0.0f;
    }

    //create plans
    fftw_plan p1 = fftw_plan_dft_3d(_v1.s, _v1.s, _v1.s, v1, f1, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_plan p2 = fftw_plan_dft_3d(_v1.s, _v1.s, _v1.s, v2, f2, FFTW_FORWARD, FFTW_ESTIMATE);

    fftw_execute(p1);
    fftw_execute(p2);

    //multiply spectrums
    for(int i = 0; i < _v1.s3; i++)
    {
        float R1 =  f1[i][0];
        float I1 =  f1[i][1];
        float R2 =  f2[i][0];
        float I2 = -f2[i][1];

        float TR = R1*R2 - I1*I2;
        float TI = I1*R2 + R1*I2;


        float mag = sqrt(TR*TR + TI*TI);

        f1[i][0] = TR / mag;
        f1[i][1] = TI / mag;
    }

    //invert back to spacial domain
    fftw_plan p3 = fftw_plan_dft_3d(_v1.s, _v1.s, _v1.s, f1, v1, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(p3);

    //find real peak
    int peak_location = 0;
    float peak_value = v1[0][0];
    float tmp;
    for(int i = 1; i < _v1.s3; i++)
    {
        tmp = v1[i][0];
        if(tmp > peak_value)
        {
            peak_value = tmp;
            peak_location = i;
        }
    }

    //free data
    fftw_free(v1);
    fftw_free(v2);
    fftw_free(f1);
    fftw_free(f2);
    fftw_destroy_plan(p1);
    fftw_destroy_plan(p2);
    fftw_destroy_plan(p3);

    //return the peak location
    Point3i ret(0,0,0);

    ret.z = peak_location / _v1.s2;
    peak_location %= _v1.s2;
    ret.y = peak_location / _v1.s;
    ret.x = peak_location % _v1.s;

    return VMat::filter_pc(ret, _v1.s);

}

}

#endif
