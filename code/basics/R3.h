#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>

//forward declaration
namespace ll_R3 { class R3; }
namespace ll_Quarternion { class Quarternion; }

namespace ll_R3
{

	namespace ll_R3_C
	{
		const float ll_R3_rad2deg = (float)57.295779513082320876798154814105;
		const float ll_R3_deg2rad = (float)0.017453292519943295;
	}
	
	
	class R3
	{
	public:
		float x, y, z;
		
		//error
		void error(std::string st);

		//constructions:
		R3(); //x=y=z=0
		R3(float scalar); //x=y=z=scalar
		R3(float xin, float yin, float zin); //x=xin,y=yin,z=zin
		R3(float xin, float yin); //x=xin, y=yin, z=0
		R3(const R3 & r2); //copy constructor
		R3 & operator = (const R3 & r2);//= op
		R3(const R3 & center, float angle1, float angle2, float radius); // sets r3 to be the 3d point about center, given by angle1 and 
		//angle2 with a radius of radius
		void setAs(const R3 & inp);
		
		//i.o
		void print();
		friend std::ostream& operator<<(std::ostream& stream, const R3& input);
		operator std::string() const
		{
			std::stringstream stream;
			stream << "[" << x << ", " << y << ", " << z << "] ";
			return stream.str();
		}

		//functions
		R3 operator+(const R3 & r2) const; //element wise add : returns [this.x+r2.x,this.y+r2.y,this.z+r2.z]
		R3 & operator+=(const R3 & r2); //this += r2
		
		R3 operator-(const R3 & r2) const; //element wise subtract : returns [this.x-r2.x,this.y-r2.y,this.z-r2.z]
		R3 & operator-=(const R3 & r2); //this -= r2
		
		float operator*(const R3 & r2) const; // dot product : returns this.x*r2.x + this.y*r2.y + this.z*r2.z
		
		R3 operator^(const R3 & r2) const; // cross product : returns a vector orthogonal to *this and r2
		R3 & operator^=(const R3 & r2); //this = cross(this,r2)
		
		
		R3 operator*(float scalar) const; // scale
		R3 & operator*=(float scalar); //this *= scalar

		R3 operator/(float scalar) const; // attenuate
		R3 & operator/=(float scalar); //this /= scalar
		
		
		R3 operator-() const; //return the inverse
		
		//equality testing
		bool operator==(const R3 & r2) const ; //tests for equality
		bool operator!=(const R3 & r2) const ; //tests for inequality
		bool operator<(const R3 & r2) const ; //tests for scanline lt
		bool operator>(const R3 & r2) const ; //tests for scanline gt
		bool operator<=(const R3 & r2) const ; //tests for scanline le
		bool operator>=(const R3 & r2) const ; //tests for scanline ge
		
		float & operator[](int index); //return &Array(x,y,z)[index]
		
		//other functions
		R3 & inv(); //inverse this
		R3 & normalize(); //normalizes this

		//more functions
		float mag() const; //computes the magnitude of this
		R3 unit() const; //grabs the unit version of this
		R3 cp(const R3 & r2) const; //a safe version of the cross product
		
		R3 interpolate_to(const R3 & target, float time) const;

		void get_dual_angles(float & a1, float & a2);
		void set_from_dual_angles(float a1, float a2);

		static void GetUnitPointFromAngle(float angle, float& x, float& y);
		static float getAngle(float x, float y);

		static float precompute_d(const R3 & point_on_plane, const R3 & plane_normal);

		static bool ray_plane_intersection(const R3 & point_from, const R3 & ray_vector, const R3 & point_on_plane, const R3 & plane_normal, float & t, R3 * point_of_intersection = NULL);
		/*
			returns information about whether, a point from *point_from in the direction *ray_vector intersects a plane which
			containes the point *point_on_plane and the normal plane_normal
			returns false if plane cannot be intersected
			t is returned as the the scalar for *ray_vector as to when it intersects the plane
		*/
		static bool ray_plane_intersection(const R3 & point_from, const R3 & ray_vector, const R3 & point_on_plane, const R3 & plane_normal, float d, float & t, R3 * point_of_intersection = NULL);
		/*
			returns information about whether, a point from *point_from in the direction *ray_vector intersects a plane which
			containes the point *point_on_plane and the normal plane_normal, and a D value of *d
			returns false if plane cannot be intersected
			t is returned as the the scalar for *ray_vector as to when it intersects the plane
		*/

		static bool point_within_triangle(const R3 & point, const R3 & triangle_normal, const R3 & triangle_point1, const R3 & triangle_point2, const R3 & triangle_point3);
		/*
			finds whether a point *point, lies within the triangle represented by normal *triangle_normal and points, (*triangle_point1,*triangle_point2,*triangle_point3)
			this function is typically used with another to discover whether a ray intersects a triangle
		*/

		void min(const R3 & r2); //sets to the minimum element wise
		void max(const R3 & r2); //sets to the maximum element wise
		float min(); //gets the smallest value out of x,y & z
		float max(); //gets the largest value out of x,y and z

		R3 & floor(); //floors the values
		R3 & round(); //rounds the values

		float dist(const R3 & r2) const; //computes the euclidean distance between two points
		
		float projScalar(const R3 & b) const; //returns the scalar of the projection of this onto b
		R3 project(const R3 & b) const; //returns the projection of this onto b

	};

	
	
}

namespace ll_Quarternion
{

	class Quarternion
	{
	public:
		float x, y, z, w;
		Quarternion();
		Quarternion(float xin, float yin, float zin, float win);
		Quarternion(float angle1, float angle2);
		Quarternion(float x, float y, float z);
		Quarternion(ll_R3::R3 pole, float angle);
		Quarternion(const Quarternion & inp);
		Quarternion clone() const;
		void setAs(const Quarternion& inp);
		Quarternion & operator=(const Quarternion & inp);
		float DotProduct(Quarternion inp) const;
		Quarternion getConjugate();
		Quarternion getInverse();
		void MultiplyBy(Quarternion inp);
		Quarternion getMultiplicationBy(Quarternion inp);
		float getMagnitude() const;
		Quarternion interpolateTo(Quarternion target, float time) const;
		ll_R3::R3 rotate(ll_R3::R3 inp) const;
		void print();
		friend std::ostream& operator<<(std::ostream& stream, const Quarternion& input);
		friend Quarternion operator*(const Quarternion& a, const Quarternion& b);
		void operator*=(Quarternion inp);
		void add(Quarternion inp);
		void scale(float inp);
	};
	

}


