#pragma once

//depends on ll_R3 library

#include "R3.h"
#include <math.h>
#include <vector>
#include <functional>

//forward declaration
namespace ll_siobj { class SI_Triangle; class SI_FullTriangle; class SI_Quad; class SI_Cube; class SIObj; }

namespace ll_siobj
{


	class SI_Triangle
	{
	public:
		int a, b, c;
		SI_Triangle();
		operator std::string() const { return std::string("(") + std::to_string(a) + ", " + std::to_string(b) + ", " + std::to_string(c) + ")";	}
		friend std::ostream & operator << (std::ostream & _cout, const SI_Triangle & t);
		SI_Triangle(int ina, int inb, int inc);
		SI_Triangle(const SI_Triangle & inp);
		SI_Triangle& operator=(const SI_Triangle & inp);
		void setAs(int ina, int inb, int inc);
		bool operator == (SI_Triangle inp) const;
		bool operator > (SI_Triangle inp) const;
		bool operator < (SI_Triangle inp) const;
		void order(); //orders a,b & c in accending order
	};

	

	class SI_FullTriangle
	{
	public:
		ll_R3::R3 a, b, c;
		SI_FullTriangle(); //a=b=c=R3(0,0,0)
		SI_FullTriangle(const ll_R3::R3 & ina, const ll_R3::R3 & inb, const ll_R3::R3 & inc);
		SI_FullTriangle(const SI_FullTriangle & t2);
		SI_FullTriangle& operator=(const SI_FullTriangle& t2);
		void setAs(ll_R3::R3 ina, ll_R3::R3 inb, ll_R3::R3 inc);
		float RayIntersectsTriangle(const ll_R3::R3 & S,const ll_R3::R3 & V, const ll_R3::R3 & N, ll_R3::R3 & intersectionPoint, bool & hit);
		ll_R3::R3 normal() const;
		/*
		checks whether a ray, from S, to S + V, intersects the triangle
		returns the scalar t for which S + V*t lies within the triangle
		intersectionPoint is the point of intersection
		hit is false if there is no intersection at all
		*/
	};

	class SI_Quad
	{
	public:
		ll_R3::R3 normal, a, b, c, d;
		float distance;
		SI_Quad();
		SI_Quad(const SI_Quad & inp);
		SI_Quad(const ll_R3::R3 & nin, const ll_R3::R3 & a, const ll_R3::R3 & b, const ll_R3::R3 & c, const ll_R3::R3 & d, const float distIn);
		SI_Quad& operator=(const SI_Quad & inp);
		void setAs(const ll_R3::R3 & nin, const ll_R3::R3 & a, const ll_R3::R3 & b, const ll_R3::R3 & c, const ll_R3::R3 & d, const float distance);

		//formerly RayHitAQuad, then RayIntersection
		bool RayIntersection(const ll_R3::R3 & from_point, const ll_R3::R3 & direction, ll_R3::R3 & position_hit, float & time_hit);
		/*
			a simple wrapper for R3::ray_plane_intersection
		*/
	};

	class SI_Cube
	{
	public:
		ll_R3::R3 pos;
		float size;
		SI_Cube();
		SI_Cube(const ll_R3::R3 & ina, float sizein);
		SI_Cube(const SI_Cube & inp);
		SI_Cube& operator=(const SI_Cube& inp);
		void setAs(const SI_Cube & inp);
		void setAs(ll_R3::R3 ina, float sizein);
		float RayIntersection(ll_R3::R3 from_point, ll_R3::R3 direction, bool & wasHit); //formerly rayhitcube, returns time value where hit
		ll_R3::R3 RayIntersection2(ll_R3::R3 from_point, ll_R3::R3 direction, bool & wasHit); //formerly rayhitcube, returns time value where hit
		bool containsPoint(const ll_R3::R3 & point); //formerly pointInCube
		bool face_contains_point(int index, const ll_R3::R3 & P);
		SI_Quad getQuad(int index);
	};

	

	class SIObj
	{
	public:
		std::vector<ll_R3::R3> _points;
		std::vector<SI_Triangle> _triangles;
		std::vector<ll_R3::R3> _normals;
		std::vector<SI_Triangle> _normal_ind;
		std::vector<ll_R3::R3> _uvs;
		std::vector<SI_Triangle> _uv_ind;

		SIObj();
		SIObj(int npoints, int ntriangles);
		SIObj(std::vector<ll_R3::R3> pointList);
		SIObj(const SIObj & obj_in);
		SIObj & operator = (const SIObj & obj_in);
		SIObj(std::string fname);

		~SIObj();

		void open_obj(std::string fname);

		SI_FullTriangle getTriangle(int index) const;
		SI_FullTriangle getNormal(int index) const;
		SI_FullTriangle operator[](int index);
		ll_R3::R3 getCenterOFTriangle(int index) const;
		
		//part of old code for bio-tree:
		float getAvgNormal(int index, SI_Cube cu, int& numHits);
		ll_R3::R3 getAvgNormalR3(int index, SI_Cube cu, int& numHits);
		float getCornerValue(int index, SI_Cube cu, int direction, int corner, bool & wasHit);
		float calculate_MSE_from_SOTPlane_to_Mesh(int index, SI_Cube cu, int direction, float da, float db, float dc, float dd, int& numHits);
		ll_R3::R3 getCornerPoint(SI_Cube cu, int direction, int corner);
		int getClosestMatchingAxis(ll_R3::R3 v);

		void savePLY(std::string fname);
		void saveOBJ(std::string fname);
		void save(std::string fname);
		

		void addTriangles(const std::vector<SI_Triangle> & triList);
		void addTriangles(const std::vector<SI_FullTriangle> & triList);

		
		void mergeCloseVertices(float distance_threshold);


		ll_R3::R3 minimal_vertex_dimensions();
		float minDist();
		ll_R3::R3 maximal_vertex_dimensions();
		float maxDist();

		void mutate_add(ll_R3::R3 p);
		void mutate_subtract(ll_R3::R3 p);
		void mutate_scale(ll_R3::R3 p);
		void mutate_divide(ll_R3::R3 p);
		void mutate_scale(float s);
		void mutate_divide(float s);


		float normalize(float largestDist);
		void unnormalize(float undo);

		ll_R3::R3 toOrigin();
		ll_R3::R3 ew_normalize(float largestDist);
		void ew_unnormalize(ll_R3::R3 undo);
		void compute_normals();
		std::vector<SI_FullTriangle> getTriangles();
		void add(SIObj & inp);
		void add_points(std::vector<ll_R3::R3> & pts);
		void mutate_points(std::function<ll_R3::R3(ll_R3::R3&)> f);
		void mutate_normals(std::function<ll_R3::R3(ll_R3::R3&)> f);
		
	};


}

