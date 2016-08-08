/*
	ll_lib = Luke Lincoln's CV Library

	Author: Luke Lincoln

	contents description: 
		Contains the GPU cuda algorithms I wrote

	depends on: null
*/

#include <iostream>
#include <map>
#include <vector>
#include <functional>
#include <string>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <opencv2\core\core.hpp>
#include <stdio.h>

#include <cufft.h>
#include <stdio.h>
#include <assert.h>
#include "code/gpu/ExternGPUPrograms.h"

using namespace std;
using namespace cv;






#define _USE_MATH_DEFINES
#include <math.h>
// Includes CUDA
#include <cuda_runtime.h>

// Utilities and timing functions

class LCuda_Memory_Ptr
{
public:
	const static int NON_HOST = 0;
	const static int HOST = 1;
	LCuda_Memory_Ptr(int type = NON_HOST)
	{
		_type = type;
		_ptr = NULL;
	}
	LCuda_Memory_Ptr(const LCuda_Memory_Ptr & i)
	{
		_type = i._type;
		_ptr = i._ptr;
	}
	LCuda_Memory_Ptr & operator = (const LCuda_Memory_Ptr & i)
	{
		_type = i._type;
		_ptr = i._ptr;
		return *this;
	}
	template <class T>
	T * operator () ()
	{
		return (T*)_ptr;
	}
	template <class T>
	T * pointer()
	{
		return (T*)_ptr;
	}
	template <class T>
	bool new_(int _size)
	{
		cudaError_t cudaStatus;

		if(_type == NON_HOST)
			cudaStatus = cudaMalloc((void**)&_ptr, _size * sizeof(T));
		else
			cudaStatus = cudaMallocHost((void**)&_ptr, _size * sizeof(T));
		if (cudaStatus != cudaSuccess)
		{
			return false;
		}
		return true;
	}
	template <class T>
	bool copy_into(int _size, T * _data)
	{
		cudaError_t cudaStatus = cudaMemcpy(_ptr, _data, _size * sizeof(T), cudaMemcpyHostToDevice);
		if (cudaStatus != cudaSuccess)
		{
			return false;
		}
		return true;
	}
	template <class T>
	void new_(int _size, T * _data, bool & alloc_worked, bool & cpy_worked)
	{
		alloc_worked = false;
		cpy_worked = false;
		if(!new_<T>(_size)) return;
		alloc_worked = true;
		cpy_worked = copy_into<T>(_size, _data);
	}
	bool delete_()
	{
		if(_type == NON_HOST)
			cudaFree(_ptr);
		else
			cudaFreeHost(_ptr);
		return true;
	}
	template <class T>
	bool retrieve(T * _data, int _size)
	{
		cudaError_t cudaStatus = cudaMemcpy(_data, _ptr, _size * sizeof(T), cudaMemcpyDeviceToHost);
		if (cudaStatus != cudaSuccess)
		{
			return false;
		}
		return true;
	}
	void make_type_host()
	{
		_type = HOST;
	}
	void make_type_non_host()
	{
		_type = NON_HOST;
	}
private:
	void * _ptr;
	int _type;
};

class LCuda_Host_Manager
{
public:
	LCuda_Host_Manager()
	{
		_named_ptrs = new map<string,LCuda_Memory_Ptr>;
	}
	~LCuda_Host_Manager()
	{
		delete_all();
		delete _named_ptrs;
	}
	LCuda_Host_Manager(const LCuda_Host_Manager & i)
	{
		shallow_copy(i);
	}
	LCuda_Host_Manager & operator = (const LCuda_Host_Manager & i)
	{
		if(this == &i) return *this;
		shallow_copy(i);
		return *this;
	}
	bool delete_(string name)
	{
		if(is_in(name))
		{
			bool rv = _named_ptrs->operator[](name).delete_();
			_named_ptrs->erase(name);
			return rv;
		}
		_latest_error = "could not free " + name;
		return false;
	}
	void * operator [] (string name)
	{
		if(is_in(name))
			return (_named_ptrs->operator[](name)).pointer<void>();
		_latest_error = "could not find: " + name + " when accessing.";
		return NULL;
	}
	template <class T>
	T * at(string name)
	{
		if(is_in(name))
			return (_named_ptrs->operator[](name)).pointer<T>();
		_latest_error = "could not find: " + name + " when accessing.";
		return NULL;
	}
	bool set_default_device()
	{
		cudaError_t cudaStatus = cudaSetDevice(0);
		if(cudaStatus != cudaSuccess)
		{
			_latest_error = "cudaSetDevice failed!  Do you have a CUDA-capable GPU installed?";
			return false;
		}
		return true;
	}
	bool is_in(string name)
	{
		return _named_ptrs->count(name) > 0;
	}
	vector<string> keys()
	{
		vector<string> rv;
		for(auto i = _named_ptrs->begin(); i != _named_ptrs->end(); i++)
			rv.push_back(i->first);
		return rv;
	}
	void print_all_keys()
	{
		cout << "Available keys:" << endl;
		vector<string> k = keys();
		for(string s : k) cout << "\t" << s << endl;
		cout << "end." << endl;
	}
	void delete_all()
	{
		vector<string> _keys = keys();
		for(auto i = _keys.begin(); i != _keys.end(); i++)
			delete_(*i);
	}
	template <class T>
	bool new_(string name, int _size)
	{
		if(is_in(name))
		{
			_latest_error = "tried to alloc to " + name + " but " + name + " is already in use";
			return false;
		}
		LCuda_Memory_Ptr mem(LCuda_Memory_Ptr::NON_HOST);
		bool rv = mem.new_<T>(_size);
		if(!rv)
		{
			_latest_error = "cuda alloc failed for " + name;
			return false;
		}
		_named_ptrs->operator[](name) = mem;
		return true;
	}
	template <class T>
	bool hnew_(string name, int _size)
	{
		if(is_in(name))
		{
			_latest_error = "tried to alloc to " + name + " but " + name + " is already in use";
			return false;
		}
		LCuda_Memory_Ptr mem(LCuda_Memory_Ptr::HOST);
		bool rv = mem.new_<T>(_size);
		if(!rv)
		{
			_latest_error = "cuda alloc failed for " + name;
			return false;
		}
		_named_ptrs->operator[](name) = mem;
		return true;
	}
	template <class T>
	bool new_(string name, int _size, T * data)
	{
		if(is_in(name))
		{
			_latest_error = "tried to alloc to " + name + " but " + name + " is already in use";
			return false;
		}
		LCuda_Memory_Ptr mem(LCuda_Memory_Ptr::NON_HOST);
		bool rv = mem.new_<T>(_size);
		if(!rv)
		{
			_latest_error = "cuda alloc failed for " + name;
			return false;
		}
		_named_ptrs->operator[](name) = mem;

		rv = mem.copy_into<T>(_size, data);
		if(!rv)
		{
			_latest_error = "cuda memcpy failed for " + name;
			return false;
		}
		return true;
	}
	template <class T>
	bool hnew_(string name, int _size, T * data)
	{
		if(is_in(name))
		{
			_latest_error = "tried to alloc to " + name + " but " + name + " is already in use";
			return false;
		}
		LCuda_Memory_Ptr mem(LCuda_Memory_Ptr::HOST);
		bool rv = mem.new_<T>(_size);
		if(!rv)
		{
			_latest_error = "cuda alloc failed for " + name;
			return false;
		}
		_named_ptrs->operator[](name) = mem;

		rv = mem.copy_into<T>(_size, data);
		if(!rv)
		{
			_latest_error = "cuda memcpy failed for " + name;
			return false;
		}
		return true;
	}
bool sync()
	{
		cudaError_t cudaStatus = cudaDeviceSynchronize();
		if(cudaStatus != cudaSuccess)
		{
			_latest_error = string("cudaDeviceSynchronize returned error code ") + std::to_string(cudaStatus) +  "after launching addKernel!\n";
			return false;
		}
		return true;
	}
	string error()
	{
		return _latest_error;
	}
	template <class T>
	bool collect(string name, int _size, T * data)
	{
		if(!is_in(name))
		{
			_latest_error = "could not find " + name + " to collect."; 
			return false;
		}
		bool rv = _named_ptrs->operator[](name).retrieve<T>(data, _size);
		if(!rv) _latest_error = "could not memcpy from " + name + " to your pointer in collect().";
		return rv;
	}
	template <class T>
	bool upload(string name, int _size, T * data)
	{
		if(!is_in(name))
		{
			_latest_error = "tried to upload to " + name + " but " + name + " is not available";
			return false;
		}
		if(!_named_ptrs->operator[](name).copy_into<T>(_size, data))
		{
			_latest_error = "cuda memcpy failed in upload for " + name;
			return false;
		}
		return true;
	}
private:
	void shallow_copy(const LCuda_Host_Manager & i)
	{
		_named_ptrs = i._named_ptrs;
	}
	map<string, LCuda_Memory_Ptr> * _named_ptrs;
	string _latest_error;
};

class VMatCufftComplex
{
public:
	cufftComplex * d;
	int s, s2, s3;
	VMatCufftComplex()
	{
		s = s2 = s3 = 1;
		d = new cufftComplex[s3];
	}
	VMatCufftComplex(int s)
	{
		this->s = s;
		s2 = s*s;
		s3 = s2*s;
		d = new cufftComplex[s3];
	}
	VMatCufftComplex(const VMatCufftComplex & v)
	{
		this->s = v.s;
		s2 = s*s;
		s3 = s2*s;
		d = new cufftComplex[s3];
		for(int i = 0; i < s3; i++) d[i] = v.d[i];
	}
	VMatCufftComplex(const VMat & v)
	{
		this->s = v.s;
		s2 = s*s;
		s3 = s2*s;
		d = new cufftComplex[s3];
		for(int i = 0; i < s3; i++)
		{
			d[i].x = v.data[i];
			d[i].y = 0.0f;
		}
	}
	VMatCufftComplex(const VMat & re, const VMat & im)
	{
		this->s = re.s;
		s2 = s*s;
		s3 = s2*s;
		d = new cufftComplex[s3];
		for(int i = 0; i < s3; i++)
		{
			d[i].x = re.data[i];
			d[i].y = im.data[i];
		}
	}
	VMatCufftComplex & operator = (const VMatCufftComplex & v)
	{
		if(this == &v) return *this;
		delete [] d;
		this->s = v.s;
		s2 = s*s;
		s3 = s2*s;
		d = new cufftComplex[s3];
		for(int i = 0; i < s3; i++) d[i] = v.d[i];
		return *this;
	}
	~VMatCufftComplex()
	{
		delete [] d;
	}
	cufftComplex & operator [] (int index)
	{
		return d[index];
	}
	cufftComplex & operator () (int x, int y, int z)
	{
		return d[z*s2 + y*s + x];
	}
	VMat real()
	{
		VMat rv = s;
		for(int i = 0; i < rv.s3; i++) rv.data[i] = this->d[i].x;
		return rv;
	}
	VMat imag()
	{
		VMat rv = s;
		for(int i = 0; i < rv.s3; i++) rv.data[i] = this->d[i].y;
		return rv;
	}
	Point3i peak_real()
	{
		float peak = (*this)(0,0,0).x;
		Point3i rv(0,0,0);
		for(int z = 0; z < s; z++)
		{
			for(int y = 0; y < s; y++)
			{
				for(int x = 0; x < s; x++)
				{
					float v = (*this)(x,y,z).x;
					if(v > peak)
					{
						peak = v;
						rv.x = x; rv.y = y; rv.z = z;
					}
				}
			}
		}
		return rv;
	}
	static Point3i filter_phase_peak(Point3i a, int s)
	{
		function<int(int,int)> f = [](int a, int s) -> int { return (a > s/2) ? s-a: -a; };
		return Point3i(f(a.x,s), f(a.y,s), f(a.z,s));
	}
	static void phase_correlate_rst_adjust_rs(Point3i pc, float & rotation, float & scale, int s)
	{
		R3 q((float)pc.x, (float)pc.y, (float)pc.z);
		q.x *= (-360.0f / (float)s);
		q.y *= (180.0f / (float)s);
		q.z /= (((float) s) / log(((float) s) / 2.56f));
		q.z = exp(q.z);
		rotation = q.x;
		scale = 1.0f /  q.z;
	}
};


struct R3_
{
	float x, y ,z;
	__host__ __device__ void newR3_(float a, float b, float c)
	{
		x = a;
		y = b;
		z = c;
	}
	__host__ __device__ static void GetUnitPointFromAngle(float angle, float & x, float & y)
	{
		angle /= 57.2957795f;
		x = cos(angle);
		y = sin(angle);
	}
	__host__ __device__ void set_from_dual_angles(float a1, float a2)
	{
		if(a2 == 0.0f)
		{
			x = 0.0f;
			y = 1.0f;
			z = 0.0f;
			return;
		}else if(a2 == 180.0f)
		{
			x = 0.0f;
			y = -1.0f;
			z = 0.0f;
			return;
		}
		float x1, y1, x2, y2;
		float zdirx=0.0f, zdiry=0.0f, zdirz=1.0f;
		float ydirx=0.0f, ydiry=1.0f, ydirz=0.0f;
		float xdirx=1.0f, xdiry=0.0f, xdirz=0.0f;
		GetUnitPointFromAngle(a1, x1, y1);
		GetUnitPointFromAngle(a2, x2, y2);
		zdirx *= y1; zdiry *= y1; zdirz *= y1;
		xdirx *= x1; xdiry *= x1; xdirz *= x1;
		xdirx += zdirx; xdiry += zdiry; xdirz += zdirz; 

		xdirx *= y2; xdiry *= y2; xdirz *= y2;
		ydirx *= x2; ydiry *= x2; ydirz *= x2;
		xdirx += ydirx; xdiry += ydiry; xdirz += ydirz;
		x = xdirx;
		y = xdiry;
		z = xdirz;
	}
	__host__ __device__ void log_polar_inv(int s)
	{
		float sf = (float) s;
		x /= sf;
		x *= 360.0f;
		y /= sf;
		y *= 180.0f;
		float M = ((float) s) / log(((float) s) / 2.56f);
		z /= M;
		M = exp(z);
		float a1 = x;
		float a2 = y;
		set_from_dual_angles(a1, a2);
		x *= M;
		y *= M;
		z *= M;
		float hw = sf * 0.5f;
		x += hw;
		y += hw;
		z += hw;
	}

	__host__ __device__ void logonly_inv(int s)
	{
		float sf = (float)s;
		float sfh = sf * 0.5f;
		R3_ hw; hw.newR3_(sfh, sfh, sfh);
		float M = sf / log(sf / 2.56f);

		x -= hw.x;
		y -= hw.y;
		z -= hw.z;

		float mag = sqrt(x*x+y*y+z*z);
		
		x/=mag;
		y/=mag;
		z/=mag;

		mag /= M;
		mag = exp(mag);
		

		x*=mag;
		y*=mag;
		z*=mag;

		x+=hw.x;
		y+=hw.y;
		z+=hw.z;

	}

};

struct LMat_
{
	float * d;
	int r, c;
	__host__ __device__ void new_LMat_(int r, int c, float * d)
	{
		this->r = r;
		this->c = c;
		this->d = d;
	}
	__host__ __device__ float & at(int r, int c)
	{
		return this->d[r*this->c + c];
	}
	__host__ __device__ struct R3_ multiply(struct R3_ v)
	{
		struct R3_ rv;
		rv.newR3_(
			v.x*d[0] + v.y*d[1] + v.z*d[2] + d[3],
			v.x*d[4] + v.y*d[5] + v.z*d[6] + d[7],
			v.x*d[8] + v.y*d[9] + v.z*d[10] + d[11]
		);
		return rv;
	}
};

struct VMat_
{
	int s;
	float * d;
	__host__ __device__ void new_VMat_(int s, float * d)
	{
		this->s = s;
		this->d = d;
	}
	__host__ __device__ float & at(int x, int y, int z)
	{
		return d[z*s*s + y*s + x];
	}
	__host__ __device__ bool inbounds(int x, int y, int z)
	{
		return x>=0 && y>=0 && z>=0 && x<s && y<s && z<s;
	}
	__host__ __device__ float at(struct R3_ r)
	{
		float pix1, pix2, tmp, tmp2;
		int p1x = (int)r.x;
		int p1y = (int)r.y;
		int p1z = (int)r.z;
		
		float a = (inbounds(p1x,p1y, p1z))? (float) at(p1x, p1y, p1z): 0.0f;
		float b = (inbounds(p1x+1,p1y, p1z))? (float) at(p1x+1, p1y, p1z): 0.0f;
		float c = (inbounds(p1x, p1y+1, p1z))? (float) at(p1x, p1y+1, p1z): 0.0f;
		float d = (inbounds(p1x+1,p1y+1, p1z))? (float) at(p1x+1, p1y+1, p1z): 0.0f;
		p1z++;
		float e = (inbounds(p1x,p1y, p1z))? (float) at(p1x, p1y, p1z): 0.0f;
		float f = (inbounds(p1x+1,p1y, p1z))? (float) at(p1x+1, p1y, p1z): 0.0f;
		float g = (inbounds(p1x,p1y+1, p1z))? (float) at(p1x, p1y+1, p1z): 0.0f;
		float h = (inbounds(p1x+1,p1y+1, p1z))? (float) at(p1x+1, p1y+1, p1z): 0.0f;
		p1z--;
		float dx = r.x - (float) p1x;
		float dy = r.y - (float) p1y;
		float dz = r.z - (float) p1z;

		tmp = (1.0f-dx)*a + dx*b;
		tmp2 = (1.0f-dx)*c + dx*d;
		pix1 = (1.0f-dy)*tmp + dy*tmp2;

		tmp = (1.0f-dx)*e + dx*f;
		tmp2 = (1.0f-dx)*g + dx*h;
		pix2 = (1.0f-dy)*tmp + dy*tmp2;

		return (1.0f-dz)*pix1 + dz*pix2;
	}
};

struct R3_ newR3_(float a, float b, float c)
{
	struct R3_ rv;
	rv.x = a;
	rv.y = b;
	rv.z = c;
	return rv;
}

__global__ void multiply_spectrums(cufftComplex * signal1, cufftComplex * signal2, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;

	int bid = z*N*N + y*N + x;

	signal2[bid].y = -signal2[bid].y;

	cufftComplex tmp;
	tmp.x = signal1[bid].x*signal2[bid].x - signal1[bid].y*signal2[bid].y;
	tmp.y = signal1[bid].y*signal2[bid].x + signal1[bid].x*signal2[bid].y;


	float mag = sqrt(tmp.x*tmp.x + tmp.y*tmp.y);
	
	signal1[bid].x = tmp.x / mag;
	signal1[bid].y = tmp.y / mag;

}

__global__ void gpu_get_magnitude(cufftComplex * signal1, float * output, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;

	int bid = z*N*N + y*N + x;
	cufftComplex tmp = signal1[bid];
	float mag = sqrt(tmp.x*tmp.x + tmp.y*tmp.y);
	output[bid] = mag;
}

__global__ void gpu_get_magnitude_swapQuads(cufftComplex * signal1, float * output, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	int hw = N/2;
	if(x > N || y > N || z >= hw) return;

	int ox=x, oy=y, oz=z+hw;

	
	if(x<hw && y<hw)
	{
		ox += hw;
		oy += hw;
	}else if(x>=hw && y<hw)
	{
		ox -= hw;
		oy += hw;
	}else if(x<hw && y>=hw)
	{
		ox += hw;
		oy -= hw;
	}else if(x>=hw && y>=hw)
	{
		ox -= hw;
		oy -= hw;
	}

	int bid = z*N*N + y*N + x;
	int bid2 = oz*N*N + oy*N + ox;
	cufftComplex tmp = signal1[bid];
	cufftComplex tmp2 = signal1[bid2];
	float mag = sqrt(tmp.x*tmp.x + tmp.y*tmp.y);
	float mag2 = sqrt(tmp2.x*tmp2.x + tmp2.y*tmp2.y);

	output[bid] = mag2;
	output[bid2] = mag;
}

__global__ void copytoCufftComplex(cufftComplex * signal1, float * input, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;

	int bid = z*N*N + y*N + x;
	signal1[bid].x = input[bid];
	signal1[bid].y = 0.0f;
}

__global__ void laplacian_gpu(cufftComplex * input, float * output, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;

	int bid = z*N*N + y*N + x;

	if(x < 1 || y < 1 || z < 1 || x >= (N-1) || y >= (N-1) || z >= (N-1))
	{
		output[bid] = 0.0f;
		return;
	}

	//indexes 0 to 25
	float val = input[(z)*N*N + (y)*N + (x)].x * 26.0f;
	val += input[(z)*N*N + (y)*N + (x+1)].x * -1.0f;
	val += input[(z)*N*N + (y-1)*N + (x+1)].x * -1.0f;
	val += input[(z)*N*N + (y-1)*N + (x)].x * -1.0f;
	val += input[(z)*N*N + (y-1)*N + (x-1)].x * -1.0f;
	val += input[(z)*N*N + (y)*N + (x-1)].x * -1.0f;
	val += input[(z)*N*N + (y+1)*N + (x-1)].x * -1.0f;
	val += input[(z)*N*N + (y+1)*N + (x)].x * -1.0f;
	val += input[(z)*N*N + (y+1)*N + (x+1)].x * -1.0f;

	val += input[(z-1)*N*N + (y)*N + (x+1)].x * -1.0f;
	val += input[(z-1)*N*N + (y-1)*N + (x+1)].x * -1.0f;
	val += input[(z-1)*N*N + (y-1)*N + (x)].x * -1.0f;
	val += input[(z-1)*N*N + (y-1)*N + (x-1)].x * -1.0f;
	val += input[(z-1)*N*N + (y)*N + (x-1)].x * -1.0f;
	val += input[(z-1)*N*N + (y+1)*N + (x-1)].x * -1.0f;
	val += input[(z-1)*N*N + (y+1)*N + (x)].x * -1.0f;
	val += input[(z-1)*N*N + (y+1)*N + (x+1)].x * -1.0f;
	val += input[(z-1)*N*N + (y)*N + (x)].x * -1.0f;

	val += input[(z+1)*N*N + (y)*N + (x+1)].x * -1.0f;
	val += input[(z+1)*N*N + (y-1)*N + (x+1)].x * -1.0f;
	val += input[(z+1)*N*N + (y-1)*N + (x)].x * -1.0f;
	val += input[(z+1)*N*N + (y-1)*N + (x-1)].x * -1.0f;
	val += input[(z+1)*N*N + (y)*N + (x-1)].x * -1.0f;
	val += input[(z+1)*N*N + (y+1)*N + (x-1)].x * -1.0f;
	val += input[(z+1)*N*N + (y+1)*N + (x)].x * -1.0f;
	val += input[(z+1)*N*N + (y+1)*N + (x+1)].x * -1.0f;
	val += input[(z+1)*N*N + (y)*N + (x)].x * -1.0f;

	
	output[bid] = val;
}

//gpu transform volume input by matrix m, put result into volume output
__global__ void volume_transform(float * input, float * output, float * m, int N)
{
	
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;

	//create p
	struct R3_ p;
	p.newR3_( (float) x, (float) y, (float) z);

	//create matrix
	struct LMat_ mat;
	mat.new_LMat_(4, 4, m);

	//transform p
	p = mat.multiply(p);

	//creat vmats:
	VMat_ out; out.new_VMat_(N, output);
	VMat_ inp; inp.new_VMat_(N, input);

	//linearly interpolate from input to output according to p
	out.at(x,y,z) = inp.at(p); 
}

//re-written
__global__ void logpolar3d_gpu(float * input, float * output, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;
	struct R3_ p; p.newR3_((float)x, (float)y, (float)z);
	p.log_polar_inv(N);

	struct VMat_ in; in.new_VMat_(N, input);
	struct VMat_ out; out.new_VMat_(N, output);

	
	out.at(x,y,z) = in.at(p);
}

__global__ void logonly3d_gpu(float * input, float * output, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;
	struct R3_ p; p.newR3_((float)x, (float)y, (float)z);
	p.logonly_inv(N);

	struct VMat_ in; in.new_VMat_(N, input);
	struct VMat_ out; out.new_VMat_(N, output);

	
	out.at(x,y,z) = in.at(p);
}

//re-written
__global__ void logpolar3d_gpu_complex_out(float * input, cufftComplex * output, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;
	struct R3_ p; p.newR3_((float)x, (float)y, (float)z);
	p.log_polar_inv(N);

	VMat_ in; in.new_VMat_(N, input);

	output[z*N*N + y*N + x].x = in.at(p);
	output[z*N*N + y*N + x].y = 0.0f;
}

__global__ void hanning_gpu(cufftComplex * input, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;
	
	int hw = N / 2;
	float hw_dist = sqrt((float)(hw * hw));
	float dist = sqrt((float)((x-hw)*(x-hw) + (y-hw)*(y-hw) + (z-hw)*(z-hw)));
	dist = hw_dist - dist;
	hw_dist *= 2.0f;

	

	input[z*N*N + y*N + x].x *= (0.5f * (1.0f - cos((2.0f * M_PI * dist) / (hw_dist - 1.0f))));
	
}

__global__ void log_on_gpu(float * input, int N)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
    int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	if(x >= N || y >= N || z >= N) return;
	input[z*N*N + y*N + x] = log(input[z*N*N + y*N + x]);
}



//vol1,vol2 were the volumes, N can be v1.s, returns true/false, and float rotation, float scale, and R3 translation
//xp,yp,zp were trans params
bool LLGPU_phase_correlate_rst(VMat vol1, VMat vol2, float & rotation, float & scale, Point3i & translation, bool hanning_window_on)
{
	LCuda_Host_Manager m;
	
	try
	{
		int S = vol1.s3;
		VMatCufftComplex vol1_ = vol1;
		VMatCufftComplex vol2_ = vol2;
		
		if(!m.new_<cufftComplex>("vol1_", S, vol1_.d)) throw m.error();
		if(!m.new_<cufftComplex>("vol2_", S, vol2_.d)) throw m.error();
		if(!m.new_<float>("tmp1", S)) throw m.error();
		if(!m.new_<float>("tmp2", S)) throw m.error();
		if(!m.new_<float>("matrix", 16)) throw m.error();


		//setup different types of GPU runs
		dim3 threadsPerBlock(8, 8, 8);
		dim3 numBlocks(vol1.s / threadsPerBlock.x, vol1.s / threadsPerBlock.y, vol1.s / threadsPerBlock.z);
		int hw = vol1.s/2;
		dim3 numBlocks4M(vol1.s / threadsPerBlock.x, vol1.s / threadsPerBlock.y, hw / threadsPerBlock.z);
		
		//hanning window
		if(hanning_window_on)
		{
			//hanning window
			hanning_gpu<<<numBlocks, threadsPerBlock>>>(m.at<cufftComplex>("vol1_"), vol1.s);
			hanning_gpu<<<numBlocks, threadsPerBlock>>>(m.at<cufftComplex>("vol2_"), vol1.s);
			if(!m.sync()) throw m.error();
		}

		//setup fft plan
		cufftHandle plan;
		cufftPlan3d(&plan, vol1.s, vol1.s, vol1.s, CUFFT_C2C);

		//do fft on vol1_ and vol2_
		cufftExecC2C(plan, m.at<cufftComplex>("vol1_"), m.at<cufftComplex>("vol1_"), CUFFT_FORWARD);
		cufftExecC2C(plan, m.at<cufftComplex>("vol2_"), m.at<cufftComplex>("vol2_"), CUFFT_FORWARD);
		if(!m.sync()) throw m.error();
		
		//get the magnitude of both
		gpu_get_magnitude_swapQuads<<<numBlocks4M, threadsPerBlock>>>(m.at<cufftComplex>("vol1_"), m.at<float>("tmp1"), vol1.s);
		gpu_get_magnitude_swapQuads<<<numBlocks4M, threadsPerBlock>>>(m.at<cufftComplex>("vol2_"), m.at<float>("tmp2"), vol1.s);
		if(!m.sync()) throw m.error();

		//get the log of both
		log_on_gpu<<<numBlocks, threadsPerBlock>>>(m.at<float>("tmp1"), vol1.s);
		log_on_gpu<<<numBlocks, threadsPerBlock>>>(m.at<float>("tmp2"), vol1.s);
		if(!m.sync()) throw m.error();
	
		//get the log polar of tmp1 and tmp2 as tmp3 and tmp4 respectfully
		logpolar3d_gpu_complex_out<<<numBlocks, threadsPerBlock>>>(m.at<float>("tmp1"), m.at<cufftComplex>("vol1_"), vol1.s);
		logpolar3d_gpu_complex_out<<<numBlocks, threadsPerBlock>>>(m.at<float>("tmp2"), m.at<cufftComplex>("vol2_"), vol1.s);
		if(!m.sync()) throw m.error();
	

		//gpu_only_pc
		function<Point3i(LCuda_Host_Manager*,cufftHandle*,string,string,VMatCufftComplex*)> f = 
		[](LCuda_Host_Manager * m, cufftHandle * plan, string data1, string data2, VMatCufftComplex * cpu_a) -> Point3i
		{
			int S = cpu_a->s3;
			cufftExecC2C(*plan, m->at<cufftComplex>(data1), m->at<cufftComplex>(data1), CUFFT_FORWARD);
			cufftExecC2C(*plan, m->at<cufftComplex>(data2), m->at<cufftComplex>(data2), CUFFT_FORWARD);
			if(!m->sync()) throw m->error();

			dim3 threadsPerBlock(8, 8, 8);
			dim3 numBlocks(cpu_a->s / threadsPerBlock.x, cpu_a->s / threadsPerBlock.y, cpu_a->s / threadsPerBlock.z);
			multiply_spectrums<<<numBlocks, threadsPerBlock>>>(m->at<cufftComplex>(data1), m->at<cufftComplex>(data2), cpu_a->s);
			if(!m->sync()) throw m->error();

			cufftExecC2C(*plan, m->at<cufftComplex>(data1), m->at<cufftComplex>(data1), CUFFT_INVERSE);

			if(!m->collect<cufftComplex>(data1, S, cpu_a->d)) throw m->error();
			return cpu_a->filter_phase_peak(cpu_a->peak_real(), cpu_a->s);
		};

		//end here

		//Phase Correlate
		translation = f(&m, &plan, "vol1_", "vol2_", &vol1_);
		if(!m.sync()) throw m.error();
		VMatCufftComplex::phase_correlate_rst_adjust_rs(translation, rotation, scale, vol1.s);
		
		//copy from vol1 to tmp1, and from vol2_ to "vol2_"
		if(!m.upload<float>("tmp1", S, vol1.data)) throw m.error();
		if(!m.upload<cufftComplex>("vol2_", S, vol2_.d)) throw m.error();
		
		//transform tmp1 by R/S and set into tmp2
		Mat transformation_matrix = VMat::transformation_matrix(vol1.s, 0.0f, rotation, 0.0f, scale, 0.0f, 0.0f, 0.0f);
		transformation_matrix = transformation_matrix.inv();
		if(!m.upload<float>("matrix", 16, (float*)transformation_matrix.data)) throw m.error();
		volume_transform<<<numBlocks, threadsPerBlock>>>(m.at<float>("tmp1"), m.at<float>("tmp2"), m.at<float>("matrix"), vol1.s);
		if(!m.sync()) throw m.error();

		//copy from tmp2 to vol1_
		//dim3 threadsPerBlock11(9, 9, 9);
		copytoCufftComplex<<<numBlocks, threadsPerBlock>>>(m.at<cufftComplex>("vol1_"), m.at<float>("tmp2"), vol1.s);//wastmp2
		if(!m.sync()) throw m.error();
	
		if(!m.upload<cufftComplex>("vol2_", S, vol2_.d)) throw m.error();
	
		//final PC
		translation = f(&m, &plan, "vol1_", "vol2_", &vol1_);
		if(!m.sync()) throw m.error();
	
		//destroy plan
		cufftDestroy(plan);
	}catch(string e)
	{
		cout << "error on LLGPU_phase_correlate_rst() -> " << e << endl;
		m.print_all_keys();
		return false;
	}
	return true;
}

bool LLGPU_phase_correlate(VMat & v1, VMat & v2, Point3i & rv)
{
	LCuda_Host_Manager m;
	try
	{
		int S = v1.s3;
		VMatCufftComplex v1_ = v1;
		VMatCufftComplex v2_ = v2;
		
		//setup the fft plan
		cufftHandle plan;
		cufftPlan3d(&plan, v1.s, v1.s, v1.s, CUFFT_C2C);

		//setup gpu data
		if(!m.new_<cufftComplex>("v1_", S, v1_.d)) throw m.error();
		if(!m.new_<cufftComplex>("v2_", S, v2_.d)) throw m.error();
		
		//do PC
		cufftExecC2C(plan, m.at<cufftComplex>("v1_"), m.at<cufftComplex>("v1_"), CUFFT_FORWARD);
		cufftExecC2C(plan, m.at<cufftComplex>("v2_"), m.at<cufftComplex>("v2_"), CUFFT_FORWARD);
		if(!m.sync()) throw m.error();
		dim3 threadsPerBlock(8, 8, 8);
		dim3 numBlocks(v1.s / threadsPerBlock.x, v1.s / threadsPerBlock.y, v1.s / threadsPerBlock.z);
		multiply_spectrums<<<numBlocks, threadsPerBlock>>>(m.at<cufftComplex>("v1_"), m.at<cufftComplex>("v2_"), v1.s);
		if(!m.sync()) throw m.error();

		cufftExecC2C(plan, m.at<cufftComplex>("v1_"), m.at<cufftComplex>("v1_"), CUFFT_INVERSE);
		if(!m.sync()) throw m.error();
		if(!m.collect<cufftComplex>("v1_", S, v1_.d)) throw m.error();
		rv = v1_.peak_real();
		
		//destroy the plan
		cufftDestroy(plan);
	}catch(string e)
	{
		cout << "error in gpu_phase_correlate() -> " << e << endl;
		return false;
	}
	return true;
}

bool LLGPU_transform(VMat & v, R3 rotation, float scale, R3 translation)
{
	int S = v.s3;
	Mat transformation_matrix = VMat::transformation_matrix(v.s, rotation.x,rotation.y,rotation.z,scale,translation.x,translation.y,translation.z);
	transformation_matrix = transformation_matrix.inv();
	LCuda_Host_Manager m;
	try
	{
		if(!m.set_default_device()) throw m.error();
		if(!m.new_<float>("input", S, v.data)) throw m.error();
		if(!m.new_<float>("output", S)) throw m.error();
		if(!m.new_<float>("matrix", 16, (float*)transformation_matrix.data)) throw m.error();
		dim3 threadsPerBlock(8, 8, 8);
		dim3 numBlocks(v.s / threadsPerBlock.x, v.s / threadsPerBlock.y, v.s / threadsPerBlock.z);
		volume_transform<<<numBlocks, threadsPerBlock>>>(m.at<float>("input"), m.at<float>("output"), m.at<float>("matrix"), v.s);
		auto cudaStatus = cudaGetLastError();
		if(cudaStatus != cudaSuccess)
		{
			throw string(cudaGetErrorString(cudaStatus)) + "\n";
		}
		if(!m.sync()) throw m.error();
		if(!m.collect<float>("output", S, v.data)) throw m.error();
	}
	catch(string e)
	{
		cout << "error in gpu_transform: " << e << endl;
		return false;
	}

	return true;
}


bool LLGPU_transform(VMat & v, Mat & transformation_matrix_input)
{
	int S = v.s3;
	Mat transformation_matrix = transformation_matrix_input.clone();
	transformation_matrix = transformation_matrix.inv();
	LCuda_Host_Manager m;
	try
	{
		if(!m.set_default_device()) throw m.error();
		if(!m.new_<float>("input", S, v.data)) throw m.error();
		if(!m.new_<float>("output", S)) throw m.error();
		if(!m.new_<float>("matrix", 16, (float*)transformation_matrix.data)) throw m.error();
		dim3 threadsPerBlock(8, 8, 8);
		dim3 numBlocks(v.s / threadsPerBlock.x, v.s / threadsPerBlock.y, v.s / threadsPerBlock.z);
		volume_transform<<<numBlocks, threadsPerBlock>>>(m.at<float>("input"), m.at<float>("output"), m.at<float>("matrix"), v.s);
		auto cudaStatus = cudaGetLastError();
		if(cudaStatus != cudaSuccess)
		{
			throw string(cudaGetErrorString(cudaStatus)) + "\n";
		}
		if(!m.sync()) throw m.error();
		if(!m.collect<float>("output", S, v.data)) throw m.error();
	}
	catch(string e)
	{
		cout << "error in gpu_transform: " << e << endl;
		return false;
	}

	return true;
}


bool LLGPU_fft3d(VMat & v, VMat & re, VMat & im)
{
	LCuda_Host_Manager m;
	try
	{
		int S = v.s3;
		if(!m.set_default_device()) throw m.error();
		VMatCufftComplex v_ = v;
		//setup the plan
		cufftHandle plan;
		cufftPlan3d(&plan, v.s, v.s, v.s, CUFFT_C2C);
		//allocate gpu data
		if(!m.new_<cufftComplex>("v_", S, v_.d)) throw m.error();
		//do fft
		cufftExecC2C(plan, m.at<cufftComplex>("v_"), m.at<cufftComplex>("v_"), CUFFT_FORWARD);
		auto cudaStatus = cudaGetLastError();
		if(cudaStatus != cudaSuccess)
			throw string("addKernel launch failed: ") + string(cudaGetErrorString(cudaStatus)) + "\n";
		if(!m.sync()) throw m.error();
		//copy out data
		if(!m.collect<cufftComplex>("v_", S, v_.d)) throw m.error();
		re = v_.real();
		im = v_.imag();
		cufftDestroy(plan);
	}catch(string s)
	{
		cout << "error in fft3d " << s << endl;
		return false;
	}
	return true;
}

bool LLGPU_ifft3d(VMat & re, VMat & im, VMat & out)
{
	LCuda_Host_Manager m;
	try
	{
		int S = re.s3;
		if(!m.set_default_device()) throw m.error();
		VMatCufftComplex re_im(re,im);
		//setup the plan
		cufftHandle plan;
		cufftPlan3d(&plan, re.s, re.s, re.s, CUFFT_C2C);
		//allocate gpu data
		if(!m.new_<cufftComplex>("re_im", S, re_im.d)) throw m.error();
		//do fft
		cufftExecC2C(plan, m.at<cufftComplex>("re_im"), m.at<cufftComplex>("re_im"), CUFFT_INVERSE);
		auto cudaStatus = cudaGetLastError();
		if(cudaStatus != cudaSuccess)
			throw string("addKernel launch failed: ") + string(cudaGetErrorString(cudaStatus)) + "\n";
		if(!m.sync()) throw m.error();
		//copy out data
		if(!m.collect<cufftComplex>("re_im", S, re_im.d)) throw m.error();
		out = re_im.real();
		cufftDestroy(plan);
	}catch(string s)
	{
		cout << "error in ifft3d " << s << endl;
		return false;
	}
	return true;
}

bool LLGPU_fft3d_mag_swap_quads(VMat & v, VMat & output)
{
	LCuda_Host_Manager m;
	try
	{
		int S = v.s3;
		output = VMat(v.s);
		//copy over to cuda data structure v_
		VMatCufftComplex v_ = v;
		//make fft plan
		cufftHandle plan;
		cufftPlan3d(&plan, v.s, v.s, v.s, CUFFT_C2C);
		//allocate data on gpu
		if(!m.new_<cufftComplex>("v_", S, v_.d)) throw m.error();
		if(!m.new_<float>("output", S, output.data)) throw m.error();
		//fft
		cufftExecC2C(plan, m.at<cufftComplex>("v_"), m.at<cufftComplex>("v_"), CUFFT_FORWARD);
		if(!m.sync()) throw m.error();
		dim3 threadsPerBlock(8, 8, 8);
		int hw = v.s/2;
		dim3 numBlocks(v.s / threadsPerBlock.x, v.s / threadsPerBlock.y, hw / threadsPerBlock.z);
		//swap quadrants and compute magnitude
		gpu_get_magnitude_swapQuads<<<numBlocks, threadsPerBlock>>>(m.at<cufftComplex>("v_"),
			m.at<float>("output"), v.s);
		auto cudaStatus = cudaGetLastError();
		if(cudaStatus != cudaSuccess)
		{
			throw string(cudaGetErrorString(cudaStatus)) + "\n";
		}
		if(!m.sync()) throw m.error();
		if(!m.collect<float>("output", S, output.data)) throw m.error();
		cufftDestroy(plan);
	}catch(string s)
	{
		cout << "error occurred in fft3d_mag_swap_quads() -> " << s << endl;
		return false;
	}
	return true;
}


bool LLGPU_log(VMat & v)
{
	LCuda_Host_Manager m;
	try
	{
		int S = v.s3;
		if(!m.set_default_device()) throw m.error();

		if(!m.new_<float>("v", S, v.data)) throw m.error();
		
		//perform element-wise log
		dim3 threadsPerBlock(8, 8, 8);
		dim3 numBlocks(v.s / threadsPerBlock.x, v.s / threadsPerBlock.y, v.s / threadsPerBlock.z);
		log_on_gpu<<<numBlocks, threadsPerBlock>>>(m.at<float>("v"), v.s);
		auto cudaStatus = cudaGetLastError();
		if(cudaStatus != cudaSuccess)
			throw string(cudaGetErrorString(cudaStatus)) + "\n";
		if(!m.sync()) throw m.error();
		if(!m.collect("v", S, v.data)) throw m.error();
	}catch(string e)
	{
		cout << "error in LLGPU_log() -> " << e << endl;
		return false;
	}
	return true;
}

bool LLGPU_log_polar(VMat & v)
{
	LCuda_Host_Manager m;
	try
	{
		int S = v.s3;
		if(!m.set_default_device()) throw m.error();
		if(!m.new_<float>("input", S, v.data)) throw m.error();
		if(!m.new_<float>("output", S)) throw m.error();
		//perform transform
		dim3 threadsPerBlock(8, 8, 8);
		dim3 numBlocks(v.s / threadsPerBlock.x, v.s / threadsPerBlock.y, v.s / threadsPerBlock.z);
		logpolar3d_gpu<<<numBlocks, threadsPerBlock>>>(m.at<float>("input"), m.at<float>("output"), v.s);
		auto cudaStatus = cudaGetLastError();
		if(cudaStatus != cudaSuccess)
			throw string(cudaGetErrorString(cudaStatus)) + "\n";
		if(!m.sync()) throw m.error();
		if(!m.collect("output", S, v.data)) throw m.error();
	}catch(string e)
	{
		cout << "error in LLGPU_log_polar() -> " << e << endl;
		return false;
	}
	return true;
}

bool LLGPU_log_only(VMat & v)
{
	LCuda_Host_Manager m;
	try
	{
		int S = v.s3;
		if(!m.set_default_device()) throw m.error();
		if(!m.new_<float>("input", S, v.data)) throw m.error();
		if(!m.new_<float>("output", S)) throw m.error();
		//perform transform
		dim3 threadsPerBlock(8, 8, 8);
		dim3 numBlocks(v.s / threadsPerBlock.x, v.s / threadsPerBlock.y, v.s / threadsPerBlock.z);
		logonly3d_gpu<<<numBlocks, threadsPerBlock>>>(m.at<float>("input"), m.at<float>("output"), v.s);
		auto cudaStatus = cudaGetLastError();
		if(cudaStatus != cudaSuccess)
			throw string(cudaGetErrorString(cudaStatus)) + "\n";
		if(!m.sync()) throw m.error();
		if(!m.collect("output", S, v.data)) throw m.error();
	}catch(string e)
	{
		cout << "error in logonly3d_gpu() -> " << e << endl;
		return false;
	}
	return true;
}