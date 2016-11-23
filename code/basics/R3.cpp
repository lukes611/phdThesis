#include "R3.h"
#include <functional>
#include <cstdio>
#include <cstdlib>


namespace ll_R3
{

	void R3::error(std::string st)
	{
		std::cout << st << std::endl;
		system("pause");
		exit(-1);
	}

	//construction:
	R3::R3() { x = y = z = 0.0f; }
	R3::R3(float scalar) { x = y = z = scalar; }
	R3::R3(float xin, float yin) { x = xin; y = yin; z = 0.0f; }
	R3::R3(float xin, float yin, float zin) { x = xin; y = yin; z = zin; }
	R3::R3(const R3 & r2) { x = r2.x; y = r2.y; z = r2.z; }
	R3 & R3::operator = (const R3 & r2) {x = r2.x; y = r2.y; z = r2.z; return *this; }
	R3::R3(const R3 & center, float angle1, float angle2, float radius)
	{
		R3 v1, up(0.0f, 1.0f, 0.0f);
		float x2 = 0.0f, y2 = 0.0f;
		GetUnitPointFromAngle(angle1, v1.x, v1.z);
		GetUnitPointFromAngle(angle2, x2, y2);
		v1 *= x2;
		up *= y2;
		*this = (((v1 + up) * radius) + center);
	}
	void R3::setAs(const R3 & r2) { x = r2.x; y = r2.y; z = r2.z; }

	//I.O
	void R3::print() { printf("[%.2f, %.2f %.2f]\n", x, y, z); }
	std::ostream& operator<<(std::ostream & stream, const R3 & input)
	{
		stream << "[" << input.x << ", " << input.y << ", " << input.z << "] ";
		return stream;
	}
	
	//algebra
	inline R3 R3::operator+(const R3 & r2) const { return R3(x+r2.x, y+r2.y, z+r2.z); }
	inline R3 & R3::operator+=(const R3 & r2) { x+=r2.x; y+=r2.y; z+=r2.z; return *this; }
	
	inline R3 R3::operator-(const R3 & r2) const { return R3(x-r2.x, y-r2.y, z-r2.z); }
	inline R3 & R3::operator-=(const R3 & r2) { x-=r2.x; y-=r2.y; z-=r2.z; return *this; }

	inline float R3::operator*(const R3 & r2) const { return x*r2.x + y*r2.y + z*r2.z; }

	inline R3 R3::operator^(const R3 & r2) const { return R3(y*r2.z - z*r2.y, z*r2.x - x*r2.z, x*r2.y - y*r2.x); }
	inline R3 & R3::operator^=(const R3 & r2) { *this = R3(y*r2.z - z*r2.y, z*r2.x - x*r2.z, x*r2.y - y*r2.x); return *this; }

	inline R3 R3::operator*(float scalar) const { return R3(x*scalar, y*scalar, z*scalar); }
	inline R3 & R3::operator*=(float scalar) { x*=scalar; y*=scalar; z*=scalar; return *this; }

	R3 R3::operator/(float scalar) const { return R3(x/scalar, y/scalar, z/scalar); }
	R3 & R3::operator/=(float scalar) { x/=scalar; y/=scalar; z/=scalar; return *this; }

	R3 R3::operator-() const { return R3(-x,-y,-z); }

	//equality testing
	inline bool R3::operator==(const R3 & r2) const { return x==r2.x && y==r2.y && z==r2.z; }
	bool R3::operator!=(const R3 & r2) const { return x!=r2.x || y!=r2.y || z!=r2.z; }
	inline bool R3::operator<(const R3 & r2) const
	{
		if(x < r2.x) return true;
		if(x > r2.x) return false;
		if(y < r2.y) return true;
		if(y > r2.y) return false;
		if(z < r2.z) return true;
		return false;
	}
	inline bool R3::operator>(const R3 & r2) const
	{
		if(x > r2.x) return true;
		if(x < r2.x) return false;
		if(y > r2.y) return true;
		if(y < r2.y) return false;
		if(z > r2.z) return true;
		return false;
	}
	inline bool R3::operator<=(const R3 & r2) const { return *this<r2 || *this==r2;  }
	inline bool R3::operator>=(const R3 & r2) const { return *this>r2 || *this==r2;  }

	inline float& R3::operator[](int index)
	{
		switch (index)
		{
			case 1: return y;
			case 2: return z;
			default: return x;
		}
	}

	R3 & R3::inv() { x = -x; y = -y; z = -z; return *this; }
	R3 & R3::normalize()
	{
		float m = mag();
		//if(m == 0.0f) error("division by zero in R3::normalize");
		x /= m;
		y /= m;
		z /= m;
		return *this;
	}
	inline float R3::mag() const { return sqrt(x*x + y*y + z*z); }
	R3 R3::unit() const { float m = mag(); return R3(x/m, y/m, z/m); }
	R3 R3::cp(const R3 & r2) const { return unit() ^ r2.unit(); }

	R3 R3::interpolate_to(const R3 & target, float time) const
	{
		R3 to = target;
		to -= *this;
		return *this + to * time;
	}

	void R3::GetUnitPointFromAngle(float angle, float& x, float& y)
	{
		angle /= ll_R3_C::ll_R3_rad2deg;
		x = cos(angle);
		y = sin(angle);
	}

	float R3::precompute_d(const R3 & point_on_plane, const R3 & plane_normal)
	{
		return -(plane_normal * point_on_plane);
	}

	bool R3::ray_plane_intersection(const R3 & point_from, const R3 & ray_vector, const R3 & point_on_plane, const R3 & plane_normal, float & t, R3 * point_of_intersection)
	{
		float denominator = plane_normal * ray_vector;
		if(denominator == 0.0f) return false; 
		float d = precompute_d(point_on_plane, plane_normal);
		t = -(((plane_normal * point_from) + d) / denominator);
		if(point_of_intersection) *point_of_intersection = point_from + ray_vector * t;
		return true;
	}

	bool R3::ray_plane_intersection(const R3 & point_from, const R3 & ray_vector, const R3 & point_on_plane, const R3 & plane_normal, float d, float & t, R3 * point_of_intersection)
	{
		float denominator = plane_normal * ray_vector;
		if(denominator == 0.0f) return false; 
		t = -(((plane_normal * point_from) + d) / denominator);
		if(point_of_intersection) *point_of_intersection = point_from + ray_vector * t;
		return true;
	}

	bool R3::point_within_triangle(const R3 & point, const R3 & triangle_normal, const R3 & a, const R3 & b, const R3 & c)
	{
		R3 Ns[3];
		float chq[3];

		R3 normal = triangle_normal;
		
		normal = normal.unit();
		Ns[0] = ((b - a) ^ normal);
		chq[0] = (point * Ns[0]) - (Ns[0] * a);

		Ns[1] = ((c - b) ^ normal);
		chq[1] = (point * Ns[1]) - (Ns[1] * b);


		Ns[2] = ((a - c) ^ normal);
		chq[2] = (point * Ns[2]) - (Ns[2] * c);

		return	(	
					(chq[0] >= 0.0f && chq[1] >= 0.0f && chq[2] >= 0.0f) ||
					(chq[0] <= 0.0f && chq[1] <= 0.0f && chq[2] <= 0.0f)
				);
	}



	float R3::getAngle(float x, float y){
		float angle = ll_R3_C::ll_R3_rad2deg * atan(y / x);
		if (x < 0.0f){
			angle += 180.0f;
		}
		else if (y < 0.0f){
			angle += 360.0f;
		}
		return angle;
	}

	void R3::get_dual_angles(float & a1, float & a2)
	{
		R3 c2 = unit();
		R3 ydir(0.0f, 1.0f, 0.0f);
		if(c2 == ydir)
		{
			a1 = 0.0f;
			a2 = 0.0f;
			return;
		}else if(c2 == R3(0.0f, -1.0f, 0.0f))
		{
			a1 = 0.0f;
			a2 = 180.0f;
			return;
		}
		a1 = getAngle(c2.x, c2.z);
	
		a2 = acos(ydir * c2);
		a2 *= ll_R3_C::ll_R3_rad2deg;
	}
	void R3::set_from_dual_angles(float a1, float a2)
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
		R3 zdir(0.0f, 0.0f, 1.0f);
		R3 ydir(0.0f, 1.0f, 0.0f);
		R3 xdir(1.0f, 0.0f, 0.0f);
		GetUnitPointFromAngle(a1, x1, y1);
		GetUnitPointFromAngle(a2, x2, y2);
		zdir *= y1;
		xdir *= x1;
		xdir += zdir;

		xdir *= y2;
		ydir *= x2;
		xdir += ydir;
		x = xdir.x;
		y = xdir.y;
		z = xdir.z;
	}

	void R3::min(const R3 & r2)
	{
		x = (x<r2.x)? x : r2.x;
		y = (y<r2.y)? y : r2.y;
		z = (z<r2.z)? z : r2.z;
	}

	void R3::max(const R3 & r2)
	{
		x = (x>r2.x)? x : r2.x;
		y = (y>r2.y)? y : r2.y;
		z = (z>r2.z)? z : r2.z;
	}

	float R3::min()
	{
		float a = (x < y)? x : y;
		return (a < z)? a : z;
	}

	float R3::max()
	{
		float a = (x > y)? x : y;
		return (a > z)? a : z;
	}

	R3 & R3::floor()
	{
		x = (float)((int)x);
		y = (float)((int)y);
		z = (float)((int)z);
		return *this;
	}
	R3 & R3::round()
	{
		x = (float)((int)(x+0.5f));
		y = (float)((int)(y+0.5f));
		z = (float)((int)(z+0.5f));
		return *this;
	}

	float R3::dist(const R3 & r2) const
	{
		return ((*this)-r2).mag();
	}

	float R3::projScalar(const R3 & b) const
	{
		return *this * b.unit();
	}
	R3 R3::project(const R3 & b) const
	{
		R3 bu = b.unit();
		float scalar = (*this) * b.unit();
		return bu * scalar;
	}
}

//Quarternion Class:
namespace ll_Quarternion
{
	Quarternion::Quarternion()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
		w = 0.0f;
	}
	Quarternion::Quarternion(float xin, float yin, float zin, float win)
	{
		x = xin;
		y = yin;
		z = zin;
		w = win;
	}
	Quarternion::Quarternion(float angle1, float angle2)
	{
		ll_R3::R3 z_axis(0.0f, 0.0f, 1.0f);
		Quarternion q1 = Quarternion(ll_R3::R3(0.0f, 1.0f, 0.0f), angle1);
		ll_R3::R3 rotation_number_2_direction = q1.rotate(z_axis);
		Quarternion q2 = Quarternion(rotation_number_2_direction, angle2);
		this->setAs(q2);
	}
	Quarternion Quarternion::clone() const
	{
		return Quarternion(x,y,z,w);
	}
	Quarternion::Quarternion(ll_R3::R3 pole, float angle)
	{
		angle /= (2.0f* ll_R3::ll_R3_C::ll_R3_rad2deg );
		w = cos(angle);
		x = sin(angle);
		pole = pole * x;
		x = pole.x;
		y = pole.y;
		z = pole.z;
	}
	Quarternion::Quarternion(float x, float y, float z)
	{
		Quarternion qx(ll_R3::R3(1.0f, 0.0f, 0.0f), x);
		Quarternion qy(ll_R3::R3(0.0f, 1.0f, 0.0f), y);
		Quarternion qz(ll_R3::R3(0.0f, 0.0f, 1.0f), z);
		*this = qz.getMultiplicationBy(qy).getMultiplicationBy(qx);
	}
	Quarternion::Quarternion(const Quarternion& inp)
	{
		x = inp.x;
		y = inp.y;
		z = inp.z;
		w = inp.w;
	}
	
	Quarternion & Quarternion::operator=(const Quarternion & inp)
	{
		x = inp.x;
		y = inp.y;
		z = inp.z;
		w = inp.w;
		return *this;
	}
	void Quarternion::setAs(const Quarternion& inp)
	{
		x = inp.x;
		y = inp.y;
		z = inp.z;
		w = inp.w;
	}
	float Quarternion::DotProduct(Quarternion inp) const
	{
		return x*inp.x + y*inp.y + z*inp.z + w*inp.w;
	}
	Quarternion Quarternion::getConjugate()
	{
		Quarternion qt = *this;
		qt.x = -qt.x;
		qt.y = -qt.y;
		qt.z = -qt.z;
		return qt;
	}
	Quarternion Quarternion::getInverse()
	{
		Quarternion q1 = *this;
		float dp = q1.DotProduct(q1);
		if (dp == 0.0f){
			return Quarternion(0.0f, 0.0f, 0.0f, 0.0f);
		}
		float mag = 1.0f / dp;
		//float mag = 1.0f / qt_dot(q1,q1);
		q1 = q1.getConjugate();
		q1.x *= mag;
		q1.y *= mag;
		q1.z *= mag;
		q1.w *= mag;
		return q1;
	}
	void Quarternion::MultiplyBy(Quarternion inp)
	{
		this->setAs(this->getMultiplicationBy(inp));
	}
	Quarternion Quarternion::getMultiplicationBy(Quarternion q2)
	{
		Quarternion rv, q1 = *this;
		rv.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
		rv.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
		rv.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
		rv.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
		return rv;
	}
	float Quarternion::getMagnitude() const
	{
		return sqrt(pow(x, 2.0f) + pow(y, 2.0f) + pow(z, 2.0f) + pow(w, 2.0f));
	}
	Quarternion Quarternion::interpolateTo(Quarternion target, float time) const
	{
		Quarternion from(x,y,z,w);
		float t2 = 1.0f - time;
		from.scale(t2);
		target.scale(time);
		Quarternion out = from;
		out.add(target);
		float re_scale = out.getMagnitude();
		out.scale(1.0f / re_scale);
		return out;
	}
	ll_R3::R3 Quarternion::rotate(ll_R3::R3 v) const
	{
		Quarternion q1 = *this;
		Quarternion q1_inv = q1.getInverse();
		Quarternion v_q(v.x, v.y, v.z, 0.0f);
		Quarternion m1 = q1.getMultiplicationBy(v_q);
		v_q = m1.getMultiplicationBy(q1_inv);
		v.x = v_q.x;
		v.y = v_q.y;
		v.z = v_q.z;
		return v;
	}
	void Quarternion::print()
	{
		printf("%.2f %.2f %.2f %.2f\n", x,y,z,w);
	}
	void Quarternion::operator*=(Quarternion inp)
	{
		this->MultiplyBy(inp);
	}
	void Quarternion::add(Quarternion inp)
	{
		x += inp.x;
		y += inp.y;
		z += inp.z;
		w += inp.w;
	}
	void Quarternion::scale(float inp)
	{
		x *= inp;
		y *= inp;
		z *= inp;
		w *= inp;
	}
	

	std::ostream& operator<<(std::ostream& stream, const Quarternion& input)
	{
		stream << input.x << " " << input.y << " " << input.z << " " << input.w;
		return stream;
	}
	Quarternion operator*(const Quarternion& a, const Quarternion& b)
	{
		Quarternion cpa = a.clone();
		return cpa.getMultiplicationBy(b.clone());
	}
	


}