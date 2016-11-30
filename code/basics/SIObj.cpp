#include "SIObj.h"
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <cstdio>
#include <cstdlib>

using namespace std;

namespace ll_siobj
{
	using namespace ll_R3;

	//basic triangle indexing object
	SI_Triangle::SI_Triangle()
	{
		a = 0;
		b = 0;
		c = 0;
	}
	SI_Triangle::SI_Triangle(int ina, int inb, int inc)
	{
		setAs(ina, inb, inc);
	}
	SI_Triangle::SI_Triangle(const SI_Triangle& inp)
	{
		a = inp.a;
		b = inp.b;
		c = inp.c;
	}
	SI_Triangle& SI_Triangle::operator = (const SI_Triangle& inp)
	{
		a = inp.a;
		b = inp.b;
		c = inp.c;
		return (*this);
	}
	void SI_Triangle::setAs(int ina, int inb, int inc)
	{
		a = ina;
		b = inb;
		c = inc;
	}
	bool SI_Triangle::operator == (SI_Triangle inp) const
	{
		SI_Triangle tc = *this;
		tc.order();
		inp.order();
		return (tc.a == inp.a && tc.b == inp.b && tc.c == inp.c);
	}
	bool SI_Triangle::operator > (SI_Triangle inp) const
	{
		if((*this) == inp || (*this) < inp) return false;
		return true;
	}
	bool SI_Triangle::operator < (SI_Triangle inp) const
	{
		if(*this == inp)
		{
			return false;
		}else
		{
			SI_Triangle cp = *this;
			cp.order();
			inp.order();
			if(cp.a < inp.a)
			{
				return true;
			}else if(cp.a == inp.a)
			{
				if(cp.b < inp.b)
				{
					return true;
				}else if(cp.b == inp.b)
				{
					if(cp.c < inp.c)
					{
						return true;
					}
				}
			}
			return false;
		}
	}
	void SI_Triangle::order()
	{
		if(c < b) swap(c,b);
		if(b < a) swap(a,b);
		if(c < b) swap(c,b);
	}
	ostream & operator << (ostream & _cout, const SI_Triangle & t)
	{
		_cout << string("(") + to_string(t.a) + ", " + to_string(t.b) + ", " + to_string(t.c) + ")";;
		return _cout;
	}

	//si_fulltriangle
	SI_FullTriangle::SI_FullTriangle()
	{
		a.x = 0.0f;
		a.y = 0.0f;
		a.z = 0.0f;

		b.x = 0.0f;
		b.y = 0.0f;
		b.z = 0.0f;

		c.x = 0.0f;
		c.y = 0.0f;
		c.z = 0.0f;
	}
	SI_FullTriangle::SI_FullTriangle(const ll_R3::R3 & ina, const ll_R3::R3 & inb, const ll_R3::R3 & inc)
	{
		a = ina;
		b = inb;
		c = inc;
	}
	SI_FullTriangle::SI_FullTriangle(const SI_FullTriangle & t2)
	{
		a = t2.a; b = t2.b; c = t2.c;
	}
	SI_FullTriangle& SI_FullTriangle::operator=(const SI_FullTriangle & t2)
	{
		if(this == &t2) return *this;
		a = t2.a; b = t2.b; c = t2.c;
		return *this;
	}

	float SI_FullTriangle::RayIntersectsTriangle(const R3 & S, const R3 & V, const R3 & N, R3 & intersectionPoint, bool & hit){

		float t = 0.0f;
		hit = false;
		intersectionPoint = R3();
		hit = R3::ray_plane_intersection(S, V, a, N, t, &intersectionPoint);
		if(!hit) return t;

		hit = R3::point_within_triangle(intersectionPoint, N, a, b, c);
		return t;
	}

	ll_R3::R3 SI_FullTriangle::normal() const
	{
		return (((b - a).unit()) ^ ((c - a).unit())).unit();
	}

	void SI_FullTriangle::rasterize(vector<R3> & ret, float skip)
	{
        //
        R3 a2c = c-a;
        R3 b2c = c-b;

        float a2cm = a2c.mag();
        float b2cm = b2c.mag();

        int D = (int)(max(a2cm, b2cm) / skip + 0.5f);

        for(int i = 0; i < D; i++)
        {
            float t1 = i / (float) D;
            R3 _a = a.interpolate_to(c, t1);
            R3 _b = b.interpolate_to(c, t1);

            float a2bm = (_b - _a).mag();
            int D = (int)(a2bm + 0.5f);
            for(int j = 0; j < D; j++)
            {
                float t2 = j / (float) D;
                R3 _ = _a.interpolate_to(_b, t2);
                ret.push_back(_);
            }
        }


        //return ret;
	}

	// basic quad object
	SI_Quad::SI_Quad()
	{
		normal.x = 0.0f, normal.y = 0.0f, normal.z = 0.0f;
		a.x = 0.0f, a.y = 0.0f, a.z = 0.0f;
		b.x = 0.0f, b.y = 0.0f, b.z = 0.0f;
		c.x = 0.0f, c.y = 0.0f, c.z = 0.0f;
		d.x = 0.0f, d.y = 0.0f, d.z = 0.0f;
		distance = 0.0f;
	}
	SI_Quad::SI_Quad(const SI_Quad & inp)
	{
		this->setAs(inp.normal, inp.a, inp.b, inp.c, inp.d, inp.distance);
	}
	SI_Quad::SI_Quad(const R3 & nin, const R3 & ain, const R3 & bin, const R3 & cin, const R3 & di, const float distIn)
	{
		setAs(nin, ain, bin, cin, di, distIn);
	}
	SI_Quad& SI_Quad::operator=(const SI_Quad& inp)
	{
		this->setAs(inp.normal, inp.a, inp.b, inp.c, inp.d, inp.distance);
		return (*this);
	}
	void SI_Quad::setAs(const R3 & nin, const R3 & a, const R3 & b, const R3 & c, const R3 & d, const float din)
	{
		normal = nin;
		this->a = a;
		this->b = b;
		this->c = c;
		this->d = d;
		distance = din;
	}
	bool SI_Quad::RayIntersection(const R3 & from_point, const R3 & direction, R3 & position_hit, float & time_hit)
	{
		return R3::ray_plane_intersection(from_point, direction, a, normal, distance, time_hit, &position_hit);
	}


	//a cube object
	SI_Cube::SI_Cube()
	{
		pos = R3();
		size = 0.0f;
	}
	SI_Cube::SI_Cube(const R3 & ina, float sizein)
	{
		this->pos = ina;
		this->size = sizein;
	}
	SI_Cube::SI_Cube(const SI_Cube & inp)
	{
		setAs(inp.pos,inp.size);
	}
	SI_Cube& SI_Cube::operator=(const SI_Cube& inp)
	{
		setAs(inp.pos, inp.size);
		return (*this);
	}
	void SI_Cube::setAs(const SI_Cube & inp)
	{
		setAs(inp.pos, inp.size);
	}
	void SI_Cube::setAs(R3 ina, float sizein)
	{
		pos = ina;
		size = sizein;
	}
	bool SI_Cube::face_contains_point(int index, const R3 & P)
	{
		if (index == 0 || index == 2){
			if (P.x >= pos.x && P.x <= pos.x + size && P.y >= pos.y && P.y <= pos.y + size){
				return true;
			}
			else{
				return false;
			}
		}

		if (index == 1 || index == 3){
			if (P.z >= pos.z && P.z <= pos.z + size && P.y >= pos.y && P.y <= pos.y + size){
				return true;
			}
			else{
				return false;
			}
		}

		if (index == 4 || index == 5){
			if (P.x >= pos.x && P.x <= pos.x + size && P.z >= pos.z && P.z <= pos.z + size){
				return true;
			}
			else{
				return false;
			}
		}
		return false;
	}
	SI_Quad SI_Cube::getQuad(int index)
	{
		R3 N = R3(0.0f, 0.0f, 0.0f);
		R3 a = pos;
		R3 b,c,d;
		float D = 0.0f;
		if (index == 0)
		{
			b = R3(a.x + size, a.y, a.z);
			c = R3(a.x + size, a.y + size, a.z);
			d = R3(a.x, a.y + size, a.z);
			N = R3(0.0f, 0.0f, -1.0f);
		}
		else if (index == 1)
		{
			a.x += size;
			b = R3(a.x, a.y, a.z + size);
			c = R3(a.x, a.y + size, a.z + size);
			d = R3(a.x, a.y + size, a.z);
			N = R3(1.0f, 0.0f, 0.0f);
		}
		else if (index == 2)
		{
			a.z += size;
			b = R3(a.x + size, a.y, a.z);
			c = R3(a.x + size, a.y + size, a.z);
			d = R3(a.x, a.y + size, a.z);
			N = R3(0.0f, 0.0f, 1.0f);
		}
		else if (index == 3)
		{
			d = R3(a.x, a.y, a.z + size);
			c = R3(a.x, a.y + size, a.z + size);
			b = R3(a.x, a.y + size, a.z);
			N = R3(-1.0f, 0.0f, 0.0f);
		}
		else if (index == 4)
		{
			a.y += size;
			b = R3(a.x + size, a.y, a.z);
			c = R3(a.x + size, a.y, a.z + size);
			d = R3(a.x, a.y, a.z + size);
			N = R3(0.0f, 1.0f, 0.0f);
		}
		else
		{
			d = R3(a.x + size, a.y, a.z);
			c = R3(a.x + size, a.y, a.z + size);
			b = R3(a.x, a.y, a.z + size);
			N = R3(0.0f, -1.0f, 0.0f);
		}

		D = -(N * a);
		SI_Quad rq = SI_Quad(N, a, b, c, d, D);
		return rq;
	}
	float SI_Cube::RayIntersection(R3 from_point, R3 direction, bool & wasHit)
	{
		float hitTs[2];

		int hitCount = 0;

		if (containsPoint(from_point) == 0)
		{
			wasHit = false;
			return 0.0f;
		}

		for (int i = 0; i < 6; i++)
		{
			SI_Quad aqu = this->getQuad(i);
			R3 outPos = R3(-1000.0f, 0.0f, 0.0f);
			float time_hit;
			int wh = aqu.RayIntersection(from_point, direction, outPos, time_hit);

			if (!wh)
			{
				continue;
			}

			int hit = face_contains_point(i, outPos);
			if (hit == 1){
				hitTs[hitCount] = time_hit;
				hitCount++;
			}
			if (hitCount == 2){
				break;
			}

		}



		if (hitCount == 2){


			//printf("Correct Face TypeB\n");
			if (hitTs[1] > 0.0f){
				wasHit = 1;
				return hitTs[1];
			}
			else if (hitTs[0] > 0.0f){
				wasHit = 1;
				return hitTs[0];
			}



		}

		wasHit = 0;
		return 0.0f;
	}
	R3 SI_Cube::RayIntersection2(R3 from_point, R3 direction, bool & wasHit)
	{
		float hitTs[2];
		R3 hitVs[2];

		int hitCount = 0;

		if (containsPoint(from_point) == 0){
			wasHit = 0;
			return R3(0.0f, 0.0f, 0.0f);
		}

		for (int i = 0; i < 6; i++){
			SI_Quad aqu = this->getQuad(i);
			R3 outPos = R3(-1000.0f, 0.0f, 0.0f);
			float t = 0.0f;
			int wh = aqu.RayIntersection(from_point, direction, outPos, t);

			if (!wh){
				continue;
			}

			int hit = face_contains_point(i, outPos);
			if (hit == 1){
				hitTs[hitCount] = t;
				hitVs[hitCount] = outPos;
				hitCount++;
			}
			if (hitCount == 2){
				break;
			}

		}



		if (hitCount == 2){


			//printf("Correct Face TypeB\n");
			if (hitTs[1] > 0.0f){
				wasHit = true;
				return hitVs[1];
			}
			else if (hitTs[0] > 0.0f){
				wasHit = true;
				return hitVs[0];
			}



		}

		wasHit = false;
		return R3(0.0f, 0.0f, 0.0f);

	}
	bool SI_Cube::containsPoint(const R3 & point)
	{
		if (point.x >= pos.x && point.x <= pos.x + size &&
			point.y >= pos.y && point.y <= pos.y + size &&
			point.z >= pos.z && point.z <= pos.z + size){
			return true;
		}

		return false;
	}

	SIObj::SIObj()
	{
		clear();
	}

	SIObj::SIObj(int npoints, int ntriangles)
	{
		clear();
		_points = vector<R3>(npoints);
		_triangles = vector<SI_Triangle>(ntriangles);
		//compute_normals();
		_normals = vector<R3>(_triangles.size());
		_normal_ind = vector<SI_Triangle>(_triangles.size());
	}

	SIObj::SIObj(std::vector<R3> pointList)
	{
		_points = pointList;
	}

	SIObj::SIObj(const SIObj & obj_in)
	{
		_points = obj_in._points;
		_triangles = obj_in._triangles;
		_normals = obj_in._normals;
		_normal_ind = obj_in._normal_ind;
		_uvs = obj_in._uvs;
		_uv_ind = obj_in._uv_ind;
	}

	SIObj & SIObj::operator = (const SIObj & obj_in)
	{
		if(this == &obj_in) return *this;
		_points = obj_in._points;
		_triangles = obj_in._triangles;
		_normals = obj_in._normals;
		_normal_ind = obj_in._normal_ind;
		_uvs = obj_in._uvs;
		_uv_ind = obj_in._uv_ind;
		return *this;
	}


	SIObj::SIObj(string fname)
	{
		FILE * fi;

		fi = fopen(fname.c_str(), "r"); //removed windows specific fopen_s
		//fopen_s(&fi, fname.c_str(), "r");

		int numVerts = 0, numTriangles = 0; //setup initial numbers as 0

		if (fi == NULL) //if could not open file, let the programmer know!
		{
			printf("BAD FILE NAME\n");
			system("PAUSE");
			return;
		}

		//read in the number of vertices and triangles
		fscanf(fi, "%i\n", &numVerts);
		fscanf(fi, "%i\n", &numTriangles);



		_points = vector<R3>(numVerts);
		_triangles = vector<SI_Triangle>(numTriangles);
		_normals = vector<R3>(numTriangles);

		int tmpa = 0, tmpb = 0, tmpc = 0;
		float tmpafloat = 0.0f, tmpbfloat = 0.0f, tmpcfloat = 0.0f;
		//read the vertices
		for (int i = 0; i < numVerts; i++)
		{

			fscanf(fi, "%f %f %f\n", &tmpafloat, &tmpbfloat, &tmpcfloat);
			_points[i] = R3(tmpafloat, tmpbfloat, tmpcfloat);

		}

		//read the triangles and calculate their normals
		for (int i = 0; i < numTriangles; i++)
		{
			fscanf(fi, "%i %i %i\n", &tmpa, &tmpb, &tmpc);
			_triangles[i] = SI_Triangle(tmpa, tmpb, tmpc);
		}

		compute_normals();



		fclose(fi);
	}


	SIObj::~SIObj()
	{
	}

	void SIObj::clear()
	{
		_points.clear();
		_triangles.clear();
		_normals.clear();
		_normal_ind.clear();
		_uvs.clear();
		_uv_ind.clear();
	}


	string SIObj::stats()
	{
        stringstream ss;
        ss << "v/t: " << _points.size() << "/" << _triangles.size() << endl;
        return ss.str();
	}

	void SIObj::open_obj(string fname)
	{
		clear();
		ifstream fi(fname.c_str(), ios::in);
		string line = "";

		function<vector<string>(string, char)> split = [](string s, char d)->vector<string>
		{
			string tmp = "";
			vector<string> rv;
			bool last_added = false;
			for(unsigned int i = 0; i < s.size(); i++)
			{
				if(s[i] == d)
				{
					rv.push_back(tmp);
					tmp = "";
					last_added = true;
				}else
				{
					tmp += s[i];
					last_added = false;
				}
			}
			if(!last_added) rv.push_back(tmp);
			return rv;
		};

		function<int(string, char)> contains = [](string s, char c) -> int
		{
			int count = 0;
			for(unsigned int i = 0; i < s.size(); i++) if(s[i] == c) count++;
			return count;
		};


		int count = 0;

		while(getline(fi, line))
		{
			vector<string> sp = split(line, ' ');
			if(sp[0] == "v" && sp.size() >= 4)
			{
				R3 v(stof(sp[1]), stof(sp[2]), stof(sp[3]));
				_points.push_back(v);
			}else if(sp[0] == "vn" && sp.size() >= 4)
			{
				R3 v(stof(sp[1]), stof(sp[2]), stof(sp[3]));
				_normals.push_back(v);
			}else if(sp[0] == "vt" && sp.size() >= 3)
			{
				R3 v(stof(sp[1]), stof(sp[2]), 0.0f);
				_uvs.push_back(v);
			}else if(sp[0] == "f" && sp.size() >= 4)
			{
				if(contains(line, '/') == (sp.size()-1)*2)
				{
					vector<string> pi, pi1, p1 = split(sp[1], '/');
					for(unsigned int i = 2; i < sp.size()-1; i++)
					{
						pi = split(sp[i],'/');
						pi1 = split(sp[i+1],'/');
						_triangles.push_back(SI_Triangle(stoi(p1[0])-1, stoi(pi[0])-1, stoi(pi1[0])-1));
						if(p1[1].size() > 0 && pi[1].size() > 0 && pi1[1].size() > 0)
							_uv_ind.push_back(SI_Triangle(stoi(p1[1])-1, stoi(pi[1])-1, stoi(pi1[1])-1));
						_normal_ind.push_back(SI_Triangle(stoi(p1[2])-1, stoi(pi[2])-1, stoi(pi1[2])-1));
					}

				}else if(contains(line, '/') >= 3)
				{
					vector<string> pi, pi1, p1 = split(sp[1], '/');
					for(unsigned int i = 2; i < sp.size()-1; i++)
					{
						pi = split(sp[i],'/');
						pi1 = split(sp[i+1],'/');
						_triangles.push_back(SI_Triangle(stoi(p1[0])-1, stoi(pi[0])-1, stoi(pi1[0])-1));
						_uv_ind.push_back(SI_Triangle(stoi(p1[1])-1, stoi(pi[1])-1, stoi(pi1[1])-1));
					}
				}else
				{
					for(unsigned int i = 2; i < sp.size()-1; i++)
						_triangles.push_back(SI_Triangle(stoi(sp[1])-1, stoi(sp[i])-1, stoi(sp[i+1])-1));
				}
			}
		}
	}

	void SIObj::save(string fname)
	{
		FILE * fi;

		fi = fopen(fname.c_str(), "w");
		//fopen_s(&fi, fname.c_str(), "w"); //removed windows specific fopen_s

		int num_points = _points.size();
		int num_triangles = _triangles.size();
		fprintf(fi, "%i\n%i\n", num_points, num_triangles);

		for(int i = 0; i < num_points; i++)
		{
			fprintf(fi, "%f %f %f\n", _points[i].x, _points[i].y, _points[i].z);
		}
		for(int i = 0; i < num_triangles; i++)
		{
			fprintf(fi, "%i %i %i\n", _triangles[i].a, _triangles[i].b, _triangles[i].c);
		}
		fclose(fi);
	}

	void SIObj::mergeCloseVertices(float distance_threshold)
	{
		int num_points = _points.size();
		vector<bool> done = vector<bool>(num_points, false);
		float recent = 0.0f;
		for(int j = 0; j < num_points; j++)
		{
			float nr = j/(float)num_points;
			if(nr - recent > 0.01f)
			{
				std::cout << (nr) << std::endl;
				recent = nr;
			}
			if(done[j])
			{
				continue;
			}
			for(int i = j+1; i < num_points; i++)
			{
				//if(j == i)
				//{
					//continue;
				//}
				if(done[i])
				{
					continue;
				}
				R3 a2b = _points[i] - _points[j];
				float dist = a2b.mag();
				if(dist < distance_threshold)
				{
					//a2b *= 0.5f;
					//vertList[j] = vertList[i];
					_points[i] = _points[j];
					done[i] = true;
					done[j] = true;
				}
			}
		}
	}


	SI_FullTriangle SIObj::getTriangle(int index) const
	{
		SI_Triangle t = _triangles[index];
		return SI_FullTriangle(_points[t.a], _points[t.b], _points[t.c]);
	}

	SI_FullTriangle SIObj::getNormal(int index) const
	{
		SI_Triangle t = _normal_ind[index];
		return SI_FullTriangle(_normals[t.a], _normals[t.b], _normals[t.c]);
	}

	R3 SIObj::getCenterOFTriangle(int index) const
	{
		SI_FullTriangle ft = this->getTriangle(index);
		R3 tmpA = ft.a;
		ft.b = (ft.b - ft.a);
		ft.c = (ft.c - ft.a);
		ft.a = R3(0.0f, 0.0f, 0.0f);
		float mag = (ft.b.mag()) / 2.0f;
		ft.b = ((ft.b.unit()) * mag);
		R3 vec2 = (ft.c - ft.b);
		float mag2 = (vec2.mag()) / 2.0f;
		vec2 = ((vec2.unit()) * mag2);
		return ((vec2 + ft.b) + tmpA);

	}

	float SIObj::getAvgNormal(int index, SI_Cube cu, int& numHits){
		SI_FullTriangle ft = this->getTriangle(index);
		float q1, q2;
		R3 incV = ft.b - ft.a, incU;
		q1 = incV.mag() / 2.0f;
		int toq1 = (int)(q1 + 0.5f), toq2;
		incV *= (1.0f / q1);
		R3 iter1 = ft.a, iter2;
		numHits = 0;
		float rval = 0.0f;
		float actualAngle = (float)getClosestMatchingAxis(_normals[index]);
		for(int j = 0; j < toq1; j++)
		{
			iter2 = iter1;
			incU = ft.c - iter2;
			q2 = incU.mag() / 2.0f;
			toq2 = (int)(q2 + 0.5f);
			incU *= (1.0f / q2);
			for(int i = 0; i < q2; i++)
			{
				if(cu.containsPoint(iter2))
				{
					numHits += 1;


				}

				iter2 += incU;
			}


			iter1 += incV;
		}


		return actualAngle * (float)numHits;

	}


	R3 SIObj::getAvgNormalR3(int index, SI_Cube cu, int& numHits){
		SI_FullTriangle ft = this->getTriangle(index);
		float q1, q2;
		R3 incV = ft.b - ft.a, incU;
		q1 = incV.mag() / 2.0f;
		int toq1 = (int)(q1 + 0.5f), toq2;
		incV *= (1.0f / q1);
		R3 iter1 = ft.a, iter2;
		numHits = 0;
		float rval = 0.0f;
		R3 actualAngle = _normals[index];
		for(int j = 0; j < toq1; j++)
		{
			iter2 = iter1;
			incU = ft.c - iter2;
			q2 = incU.mag() / 2.0f;
			toq2 = (int)(q2 + 0.5f);
			incU *= (1.0f / q2);
			for(int i = 0; i < q2; i++)
			{
				if(cu.containsPoint(iter2))
				{
					numHits += 1;


				}

				iter2 += incU;
			}


			iter1 += incV;
		}


		return actualAngle * (float)numHits;

	}

	R3 SIObj::getCornerPoint(SI_Cube cu, int direction, int corner){
		R3 posFrom = R3(0.0f, 0.0f, 0.0f);
		if (direction == 0){
			posFrom = cu.pos;
			if (corner == 1){
				posFrom = (posFrom + R3(0.0f, 0.0f, cu.size));
			}
			else if (corner == 2){
				posFrom = (posFrom + R3(0.0f, cu.size, cu.size));
			}
			else if (corner == 3){
				posFrom = (posFrom + R3(0.0f, cu.size, 0.0f));
			}
		}
		else if (direction == 1){
			posFrom = cu.pos;
			if (corner == 1){
				posFrom = (posFrom + R3(cu.size, 0.0f, 0.0f));
			}
			else if (corner == 2){
				posFrom = (posFrom + R3(cu.size, 0.0f, cu.size));
			}
			else if (corner == 3){
				posFrom = (posFrom + R3(0.0f, 0.0f, cu.size));
			}
		}
		else{
			posFrom = cu.pos;
			if (corner == 1){
				posFrom = (posFrom + R3(cu.size, 0.0f, 0.0f));
			}
			else if (corner == 2){
				posFrom = (posFrom + R3(cu.size, cu.size, 0.0f));
			}
			else if (corner == 3){
				posFrom = (posFrom + R3(0.0f, cu.size, 0.0f));
			}
		}
		return posFrom;
	}

	float SIObj::getCornerValue(int index, SI_Cube cu, int direction, int corner, bool & wasHit){

		SI_FullTriangle ft = this->getTriangle(index);
		bool hit = false;
		float t = 0.0f;
		wasHit = false;
		R3 interP = R3(0.0f, 0.0f, 0.0f);
		R3 dirVec = R3(0.0f, 0.0f, 0.0f);
		R3 posFrom = R3(0.0f, 0.0f, 0.0f);
		if (direction == 0){
			dirVec = R3(1.0f, 0.0f, 0.0f);
			posFrom = cu.pos;
			if (corner == 1){
				posFrom = (posFrom + R3(0.0f, 0.0f, cu.size));
			}
			else if (corner == 2){
				posFrom = (posFrom + R3(0.0f, cu.size, cu.size));
			}
			else if (corner == 3){
				posFrom = (posFrom + R3(0.0f, cu.size, 0.0f));
			}
		}
		else if (direction == 1){
			dirVec = R3(0.0f, 1.0f, 0.0f);
			posFrom = cu.pos;
			if (corner == 1){
				posFrom = (posFrom + R3(cu.size, 0.0f, 0.0f));
			}
			else if (corner == 2){
				posFrom = (posFrom + R3(cu.size, 0.0f, cu.size));
			}
			else if (corner == 3){
				posFrom = (posFrom + R3(0.0f, 0.0f, cu.size));
			}
		}
		else{
			dirVec = R3(0.0f, 0.0f, 1.0f);
			posFrom = cu.pos;
			if (corner == 1){
				posFrom = (posFrom + R3(cu.size, 0.0f, 0.0f));
			}
			else if (corner == 2){
				posFrom = (posFrom + R3(cu.size, cu.size, 0.0f));
			}
			else if (corner == 3){
				posFrom = (posFrom + R3(0.0f, cu.size, 0.0f));
			}
		}

		t = ft.RayIntersectsTriangle(posFrom, dirVec, _normals[index], interP, hit);
		if (hit){
			wasHit = true;
			return t;
		}
		return 0.0f;
	}



	float SIObj::calculate_MSE_from_SOTPlane_to_Mesh(int index, SI_Cube cu, int direction, float da, float db, float dc, float dd, int& numHits)
	{
		//calculate the MSE between an SOT plane and the mesh
		SI_FullTriangle SOT_plane_TriA, SOT_plane_TriB;
		R3 Na = R3();
		R3 Nb = R3();
		R3 hp = R3();
		R3 N = _normals[index];
		if (direction == 0)
		{
			R3 ta = R3(cu.pos.x + da, cu.pos.y, cu.pos.z);
			R3 tb = R3(cu.pos.x + db, cu.pos.y, cu.pos.z + cu.size);
			R3 tc = R3(cu.pos.x + dc, cu.pos.y + cu.size, cu.pos.z + cu.size);
			R3 td = R3(cu.pos.x + dd, cu.pos.y + cu.size, cu.pos.z);
			SOT_plane_TriA = SI_FullTriangle(ta, tc, td);
			SOT_plane_TriB = SI_FullTriangle(ta, tb, tc);
			N = R3(1.0f, 0.0f, 0.0f);

		}
		else if (direction == 1)
		{
			R3 ta = R3(cu.pos.x, cu.pos.y + da, cu.pos.z);
			R3 tb = R3(cu.pos.x + cu.size, cu.pos.y + db, cu.pos.z);
			R3 tc = R3(cu.pos.x + cu.size, cu.pos.y + dc, cu.pos.z + cu.size);
			R3 td = R3(cu.pos.x, cu.pos.y + dd, cu.pos.z + cu.size);
			SOT_plane_TriA = SI_FullTriangle(ta, tc, td);
			SOT_plane_TriB = SI_FullTriangle(ta, tb, tc);
			N = R3(0.0f, 1.0f, 0.0f);
		}
		else
		{
			R3 ta = R3(cu.pos.x, cu.pos.y, cu.pos.z + da);
			R3 tb = R3(cu.pos.x + cu.size, cu.pos.y, cu.pos.z + db);
			R3 tc = R3(cu.pos.x + cu.size, cu.pos.y + cu.size, cu.pos.z + dc);
			R3 td = R3(cu.pos.x, cu.pos.y + cu.size, cu.pos.z + dd);
			SOT_plane_TriA = SI_FullTriangle(ta, tc, td);
			SOT_plane_TriB = SI_FullTriangle(ta, tb, tc);
			N = R3(0.0f, 0.0f, 1.0f);

		}
		Na = ( ( (SOT_plane_TriA.b -  SOT_plane_TriA.a ).unit() ) ^ ( ( SOT_plane_TriA.c - SOT_plane_TriA.a ).unit() ) );
		Nb = Na;
		SI_FullTriangle test_triangle = this->getTriangle(index);


		float iterf1, iterf2;
		R3 incrementor_1 = test_triangle.b - test_triangle.c, incrementor_2;
		iterf1 = incrementor_1.mag() / 2.0f;
		int num_iters1 = (int)(iterf1+0.5f), num_iters2;
		R3 iterator_1 = test_triangle.a;
		incrementor_1.normalize();
		incrementor_1 *= (1.0f /  iterf1);

		double count = 0.0;
		double mean = 0.0;
		//q, a to b
		for (int q = 0; q < num_iters1; q++)
		{
			//a to c
			R3 iterator_2 = iterator_1;
			incrementor_2 = (test_triangle.c - iterator_2);
			iterf2 = incrementor_2.mag() / 2.0f;
			num_iters2 = (int) (iterf2+0.5f);
			incrementor_2.normalize();
			incrementor_2 *= (1.0f / iterf2);
			for (int p = 0; p < num_iters2; p++)
			{

				bool tmpWH1 = false, tmpWH2 = false;
				float Tv1 = SOT_plane_TriA.RayIntersectsTriangle(iterator_2, N, Na, hp, tmpWH1);
				if (tmpWH1)
				{
					mean += pow((double)(Tv1), 2.0);
					count += 1.0;
				}
				else
				{
					float Tv2 = SOT_plane_TriB.RayIntersectsTriangle(iterator_2, N, Nb, hp, tmpWH2);
					if (tmpWH2)
					{
						mean += pow((double)(Tv2), 2.0);
						count += 1.0;
					}
				}


				iterator_2 += incrementor_2;
			}
			iterator_1 += incrementor_1;
		}

		if (count == 0.0)
		{
			numHits = 0;
			return 0.0f;
		}
		else
		{
			numHits = (int)count;
			return (float)(mean);
		}

	}

	int SIObj::getClosestMatchingAxis(R3 v)
	{
		int tlo[6] =
		{
			0,
			0,
			1,
			1,
			2,
			2
		};

		R3 tl[6] =
		{
			R3(1.0f, 0.0f, 0.0f),
			R3(-1.0f, 0.0f, 0.0f),
			R3(0.0f, 1.0f, 0.0f),
			R3(0.0f, -1.0f, 0.0f),
			R3(0.0f, 0.0f, 1.0f),
			R3(0.0f, 0.0f, -1.0f)
		}
		;

		v.normalize();
		int min_index = 0;
		float min = (float)std::abs((double)acos(v * tl[0]));
		for(int i = 1; i < 6; i++)
		{
			float tmp = (float)std::abs((double)acos(v * tl[i]));
			if(tmp < min)
			{
				min = tmp;
				min_index = i;
			}
		}
		return tlo[min_index];
	}

	void SIObj::addTriangles(const vector<SI_Triangle> & triList)
	{
		for(unsigned int i = 0; i < triList.size(); i++)
		{
			_triangles.push_back(triList[i]);
		}
	}

	void SIObj::addTriangles(const vector<SI_FullTriangle> & triList)
	{
		for(unsigned int i = 0; i < triList.size(); i++)
		{
			_points.push_back(triList[i].a);
			int ind_1 = ((int)_points.size())-1;
			_points.push_back(triList[i].b);
			int ind_2 = ((int)_points.size())-1;
			_points.push_back(triList[i].c);
			int ind_3 = ((int)_points.size())-1;
			_triangles.push_back(SI_Triangle(ind_1, ind_2, ind_3));
		}
	}

	vector<SI_FullTriangle> SIObj::getTriangles()
	{
		vector<SI_FullTriangle> rv;
		for(unsigned int i = 0; i < _triangles.size(); i++)
			rv.push_back(getTriangle(i));
		return rv;
	}

	void SIObj::add(SIObj & inp)
	{
		vector<SI_FullTriangle> ts = inp.getTriangles();
		addTriangles(ts);
	}

	void SIObj::savePLY(std::string fname)
	{
		FILE* fi;
		fi = fopen(fname.c_str(), "wb");
		//fopen_s(&fi, fname.c_str(), "wb"); //removing windows specific fopen_s

		int triFiller = 0;
		float vertFiller = 0.0f;

		char headBuffer[1000];
		sprintf(headBuffer, "ply\nformat binary_little_endian 1.0\ncomment Luke Generaged\nelement vertex %i\nproperty float x\nproperty float y\nproperty float z\nproperty int flags\nelement face %i\nproperty list uchar int vertex_indices\nproperty int flags\nend_header\n", _points.size(), _triangles.size());

		fwrite(headBuffer, 1, strlen(headBuffer), fi);

		for(unsigned int i = 0; i < _points.size(); i++)
		{
			fwrite((void*)&_points[i].x, sizeof(float), 1, fi);
			fwrite((void*)&_points[i].y, sizeof(float), 1, fi);
			fwrite((void*)&_points[i].z, sizeof(float), 1, fi);
			fwrite((void*)&vertFiller, sizeof(float), 1, fi);
		}
		unsigned char numPointsInTri = 0x03;
		for(unsigned int i = 0; i < _triangles.size(); i++)
		{
			fwrite((void*)&numPointsInTri, 1, 1, fi);
			fwrite((void*)&_triangles[i].a, sizeof(int), 1, fi);
			fwrite((void*)&_triangles[i].b, sizeof(int), 1, fi);
			fwrite((void*)&_triangles[i].c, sizeof(int), 1, fi);
			fwrite((void*)&triFiller, sizeof(int), 1, fi);
		}

		fclose(fi);
	}

	void SIObj::saveOBJ(std::string fname)
	{
		int numVerts = _points.size();
		int numTriangles = _triangles.size();
		FILE * fi;
		fi = fopen(fname.c_str(), "w");
		//fopen_s(&fi, fname.c_str(), "w"); //removing windows specific fopen_s

		fprintf(fi, "# OBJ FILE : GENERATED BY A PROGRAM WRITTEN BY LUKE LINCOLN\n");
		for(int i = 0; i < numVerts; i++)
		{
			R3 v = _points[i];
			fprintf(fi, "v %f %f %f\n", v.x, v.y, v.z);
		}

		for(int i = 0; i < numTriangles; i++)
		{
			SI_Triangle t = _triangles[i];
			fprintf(fi, "f %i %i %i\n", t.a+1, t.b+1, t.c+1);
		}


		fclose(fi);
	}

	void SIObj::compute_normals()
	{
		if(_normals.size() != _triangles.size() || _normal_ind.size() != _triangles.size())
		{
			_normals = vector<R3>(_triangles.size());
			_normal_ind = vector<SI_Triangle>(_triangles.size());
		}
		for(unsigned int i = 0; i < _triangles.size(); i++)
		{
			SI_FullTriangle t = getTriangle(i);
			_normals[i] = t.normal();
			_normal_ind[i] = SI_Triangle(i,i,i);
		}
	}

	R3 SIObj::minimal_vertex_dimensions()
	{
		R3 rv = _points[0];
		int numVerts = _points.size();
		for(int i = 1; i < numVerts; i++)
		{
			rv.x = (rv.x < _points[i].x) ? rv.x : _points[i].x;
			rv.y = (rv.y < _points[i].y) ? rv.y : _points[i].y;
			rv.z = (rv.z < _points[i].z) ? rv.z : _points[i].z;
		}
		return rv;
	}
	float SIObj::minDist()
	{
		R3 rve = this->minimal_vertex_dimensions();
		float rv = (rve.x < rve.y) ? rve.x : rve.y;
		rv = (rv < rve.z) ? rv : rve.z;
		return rv;
	}
	R3 SIObj::maximal_vertex_dimensions()
	{
		R3 rv = _points[0];
		int numVerts = _points.size();
		for(int i = 1; i < numVerts; i++)
		{
			rv.x = (rv.x > _points[i].x) ? rv.x : _points[i].x;
			rv.y = (rv.y > _points[i].y) ? rv.y : _points[i].y;
			rv.z = (rv.z > _points[i].z) ? rv.z : _points[i].z;
		}
		return rv;
	}
	float SIObj::maxDist()
	{
		R3 rve = this->maximal_vertex_dimensions();
		float rv = (rve.x > rve.y) ? rve.x : rve.y;
		rv = (rv > rve.z) ? rv : rve.z;
		return rv;
	}


	void SIObj::mutate_add(R3 tmp)
	{
		mutate_points([tmp](R3& p)->R3
		{
			return p + tmp;
		});
	}

	void SIObj::mutate_subtract(R3 tmp)
	{
		mutate_points([tmp](R3& p)->R3
		{
			return p - tmp;
		});
	}
	void SIObj::mutate_scale(R3 tmp)
	{
		mutate_points([tmp](R3& p)->R3
		{
			p.x *= tmp.x;
			p.y *= tmp.y;
			p.z *= tmp.z;
			return p;
		});
	}

	void SIObj::mutate_divide(R3 tmp)
	{
		mutate_points([tmp](R3& p)->R3
		{
			p.x /= tmp.x;
			p.y /= tmp.y;
			p.z /= tmp.z;
			return p;
		});
	}

	void SIObj::mutate_scale(float a)
	{
		R3 tmp(a,a,a);
		mutate_points([tmp](R3& p)->R3
		{
			p.x *= tmp.x;
			p.y *= tmp.y;
			p.z *= tmp.z;
			return p;
		});
	}

	void SIObj::mutate_divide(float a)
	{
		R3 tmp(a,a,a);
		mutate_points([tmp](R3& p)->R3
		{
			p.x /= tmp.x;
			p.y /= tmp.y;
			p.z /= tmp.z;
			return p;
		});
	}



	float SIObj::normalize(float largestDist)
	{
		R3 mini;
		float maxD;
		mini = minimal_vertex_dimensions();
		mutate_subtract(mini);
		maxD = maxDist();
		mutate_points([maxD, largestDist](R3 & tmp) -> R3
		{
			tmp /= maxD;
			tmp *= largestDist;
			return tmp;
		});
		return maxD;
	}

	void SIObj::unnormalize(float undo)
	{
		normalize(1.0f);
		mutate_scale(undo);
	}

	R3 SIObj::toOrigin()
	{
		R3 mini = minimal_vertex_dimensions();
		mutate_subtract(mini);
		return mini;
	}

	void SIObj::centerNormalize(float volwidth)
	{
		normalize(volwidth);
		mutate_subtract((maximal_vertex_dimensions() - minimal_vertex_dimensions()) * 0.5f);
		mutate_add(R3(0.5f, 0.5f, 0.5f) * volwidth);
	}

	R3 SIObj::ew_normalize(float largestDist)
	{
		toOrigin();
		R3 maxi = maximal_vertex_dimensions();
		mutate_points([maxi, largestDist](R3 & tmp) -> R3
		{
			tmp.x /= maxi.x;
			tmp.y /= maxi.y;
			tmp.z /= maxi.z;
			tmp *= largestDist;
			return tmp;
		});
		return maxi;
	}

	void SIObj::ew_unnormalize(R3 undo)
	{
		float ld = this->maxDist();
		mutate_points([undo, ld](R3 & p) -> R3
		{
			p /= ld;
			p.x *= undo.x;
			p.y *= undo.y;
			p.z *= undo.z;
			return p;
		});
	}

	void SIObj::add_points(std::vector<ll_R3::R3> & pts)
	{
		for(unsigned int i = 0; i < pts.size(); i++)
			_points.push_back(pts[i]);
	}


	void SIObj::mutate_points(function<R3(R3&)> f)
	{
		int s = _points.size();
		for(int i = 0; i < s; i++) _points[i] = f(_points[i]);
	}

	void SIObj::mutate_normals(function<R3(R3&)> f)
	{
		int s = _normals.size();
		for(int i = 0; i < s; i++) _normals[i] = f(_normals[i]);
	}

	bool SIObj::operator == (const SIObj & o2)
	{
		if(_points.size() != o2._points.size()) return false;
		if(_triangles.size() != o2._triangles.size()) return false;
		for(unsigned int i = 0; i < _points.size(); i++)
			if(_points[i] != o2._points[i]) return false;
		for(unsigned int i = 0; i < _triangles.size(); i++)
			if(_triangles[i].a != o2._triangles[i].a || _triangles[i].b != o2._triangles[i].b || _triangles[i].c != o2._triangles[i].c) return false;
		return true;
	}
}
