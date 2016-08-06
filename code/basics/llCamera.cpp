#include "llCamera.h"

using namespace ll_R3;
using namespace std;

namespace ll_cam
{


	//cad camera
	Cad_cam::Cad_cam(float x, float y, float r, float zac)
	{
		angle_x = x;
		angle_y = y;
		rad = r;
		z_loc = zac;
		init();
	}
	Cad_cam::Cad_cam(const Cad_cam & c)
	{
		angle_x = c.angle_x;
		angle_y = c.angle_y;
		rad = c.rad;
		z_loc = c.z_loc;
		init(c);
	}
	Cad_cam & Cad_cam::operator = (const Cad_cam & c)
	{
		angle_x = c.angle_x;
		angle_y = c.angle_y;
		rad = c.rad;
		z_loc = c.z_loc;
		init(c);
		return *this;
	}
	R3 Cad_cam::cop()
	{
		float x, y, x2, y2;
		R3::GetUnitPointFromAngle(angle_x, x, y);
		R3::GetUnitPointFromAngle(angle_y, x2, y2);
		R3 up(0.0f, 1.0f, 0.0f);
		R3 right(x, 0.0f, y);
		right *= x2;
		up *= y2;
		return ( ( right + up ) * rad ) + R3(0.0f, 0.0f, z_loc);
	}
	R3 Cad_cam::point_no_z()
	{
		float x, y, x2, y2;
		R3::GetUnitPointFromAngle(angle_x, x, y);
		R3::GetUnitPointFromAngle(angle_y, x2, y2);
		R3 up(0.0f, 1.0f, 0.0f);
		R3 right(x, 0.0f, y);
		right *= x2;
		up *= y2;
		return ( ( right + up ) * rad );
	}
	R3 Cad_cam::at()
	{
		return cop() + (R3()-point_no_z()).unit();
	}
	R3 Cad_cam::dir()
	{
		return (R3()-point_no_z()).unit();
	}
	R3 Cad_cam::right()
	{
		R3 fakeUp(0.0f, 1.0f, 0.0f);
		return (fakeUp ^ dir()).unit();
	}
	R3 Cad_cam::up()
	{
		return dir() ^ right();
	}
	void Cad_cam::inc_(float & val, float am, float max, float min)
	{
		float nval = val + am;
		if(nval >= min && nval <= max)
		{
			val = nval;
		}
	}
	void Cad_cam::incX(float am)
	{
		angle_x += am;
		angle_x = fmod(angle_x, 360.0f);
	}
	void Cad_cam::decX(float am)
	{
		angle_x -= am;
		angle_x = (angle_x < 0.0f) ? 360.0f + angle_x : angle_x;
	}
	void Cad_cam::incY(float am)
	{
		angle_y = (angle_y+am > 80.0f) ? angle_y : angle_y+am;
	}
	void Cad_cam::decY(float am)
	{
		angle_y = (angle_y-am < -80.0f) ? angle_y : angle_y-am;
	}
	void Cad_cam::incR(float am)
	{
		inc_(rad, am, 80000.0f, 50.0f);
	}
	void Cad_cam::decR(float am)
	{
		inc_(rad, -am, 80000.0f, 50.0f);
	}
	void Cad_cam::incZ(float am)
	{
		inc_(z_loc, am, 80000.0f, -80000.0f);
	}
	void Cad_cam::decZ(float am)
	{
		inc_(z_loc, -am, 80000.0f, -80000.0f);
	}
	void Cad_cam::left_mouse_down(int x, int y)
	{
		left_pressed = true;
		l_past_x = x;
		l_past_y = y;
	}
	void Cad_cam::left_mouse_up(int x, int y)
	{
		left_pressed = false;
	}
	void Cad_cam::right_mouse_down(int x, int y)
	{
		right_pressed = true;
		r_past_x = x;
		r_past_y = y;
	}
	void Cad_cam::right_mouse_up(int x, int y)
	{
		right_pressed = false;
	}
	void Cad_cam::middle_mouse_down(int x, int y)
	{
		middle_pressed = true;
		m_past_x = x;
		m_past_y = y;
	}
	void Cad_cam::middle_mouse_up(int x, int y)
	{
		middle_pressed = false;
	}
	void Cad_cam::mouse(int x, int y)
	{
		int dir_x, dir_y;
		if(left_pressed)
		{
			dir_x = x - l_past_x;
			dir_y = y - l_past_y;
			R3 dir3((float)dir_x, (float)dir_y, 0.0f);
			dir3 *= 0.5f;
			this->incX(dir3.x);
			this->incY(dir3.y);
			l_past_x = x;
			l_past_y = y;
		}

		if(right_pressed)
		{
			dir_x = x - r_past_x;
			dir_y = y - r_past_y;
			R3 dir3((float)dir_x, (float)dir_y, 0.0f);
			this->incR(dir3.y);
			r_past_x = x;
			r_past_y = y;
		}

		if(middle_pressed)
		{
			dir_x = x - m_past_x;
			dir_y = y - m_past_y;
			R3 dir3((float)dir_x, (float)dir_y, 0.0f);
			this->incZ(dir3.y);
			m_past_x = x;
			m_past_y = y;
		}
	}
	void Cad_cam::keyboard(char key, int x, int y){}
	string Cad_cam::to_string()
	{
		string rv = "angle_x: " + std::to_string(angle_x) + ", angle_y: " + std::to_string(angle_y) + 
			"rad: " + std::to_string(rad) + ", z_loc: " + std::to_string(z_loc);
		return rv;
	}
	void Cad_cam::init()
	{
		left_pressed	= false;
		right_pressed	= false;
		middle_pressed	= false;
		l_past_x		= 0;
		l_past_y		= 0;
		r_past_x		= 0;
		r_past_y		= 0;
		m_past_x		= 0;
		m_past_y		= 0;
	}
	void Cad_cam::init(const Cad_cam & c)
	{
		left_pressed	= c.left_pressed;
		right_pressed	= c.right_pressed;
		middle_pressed	= c.middle_pressed;
		l_past_x		= c.l_past_x;
		l_past_y		= c.l_past_y;
		r_past_x		= c.r_past_x;
		r_past_y		= c.r_past_y;
		m_past_x		= c.m_past_x;
		m_past_y		= c.m_past_y;
	}


	//first person camera
	Fps_cam::Fps_cam()
	{
		location = R3(0.0f, 0.0f, 400.0f);
		angle_x = 46;
		angle_y = 180;
		look_speed = 2.0f;
		move_speed = 10.0f;
	}
	Fps_cam::Fps_cam(R3 location, float angle_y, float angle_x)
	{
		this->location = location;
		this->angle_x = angle_x;
		this->angle_y = angle_y;
		look_speed = 2.0f;
		move_speed = 10.0f;
	}
	Fps_cam::Fps_cam(const Fps_cam & c2)
	{
		angle_x = c2.angle_x;
		angle_y = c2.angle_y;
		location = c2.location;
		look_speed = c2.look_speed;
		move_speed = c2.move_speed;
	}
	Fps_cam & Fps_cam::operator = (const Fps_cam & c2)
	{
		angle_x = c2.angle_x;
		angle_y = c2.angle_y;
		location = c2.location;
		look_speed = c2.look_speed;
		move_speed = c2.move_speed;
		return *this;
	}

	R3 Fps_cam::cop()
	{
		return location;
	}
	R3 Fps_cam::at()
	{
		return location + dir();
	}
	R3 Fps_cam::dir()
	{
		R3 rv;
		rv.set_from_dual_angles(angle_y, angle_x);
		return rv.unit();
	}
	R3 Fps_cam::right()
	{
		R3 fakeUp(0.0f, 1.0f, 0.0f);
		return (fakeUp ^ dir()).unit();
	}
	R3 Fps_cam::up()
	{
		return (dir() ^ right()).unit();
	}

	R3 Fps_cam::flat_dir()
	{
		R3 rv;
		rv.set_from_dual_angles(angle_y, 90.0f);
		return rv;
	}
	R3 Fps_cam::flat_right()
	{
		return -(flat_up() ^ flat_dir()).unit();
	}
	R3 Fps_cam::flat_up()
	{
		return R3(0.0f, 1.0f, 0.0f);
	}

	void Fps_cam::move_forwards()
	{
		location += flat_dir() * move_speed;
	}
	void Fps_cam::move_backwards()
	{
		location += flat_dir() * -move_speed;
	}
	void Fps_cam::strafe_left()
	{
		location += flat_right() * -move_speed;
	}
	void Fps_cam::strafe_right()
	{
		location += flat_right() * move_speed;
	}
	void Fps_cam::lift()
	{
		location += flat_up() * move_speed;
	}
	void Fps_cam::drop()
	{
		location += flat_up() * -move_speed;
	}
	void Fps_cam::look_right()
	{
		angle_y += look_speed;
		angle_y = fmod(angle_y, 360.0f);
	}
	void Fps_cam::look_left()
	{
		angle_y -= look_speed;
		angle_y = (angle_y < 0.0f) ? 360.0f + angle_y : angle_y;
	}
	void Fps_cam::look_down()
	{
		if(angle_x+look_speed < 120)
			angle_x += look_speed;
	}
	void Fps_cam::look_up()
	{
		if(angle_x > 60.0f)
			angle_x -= look_speed;
	}

	//input
	void Fps_cam::left_mouse_down(int x, int y){}
	void Fps_cam::left_mouse_up(int x, int y){}
	void Fps_cam::right_mouse_down(int x, int y){}
	void Fps_cam::right_mouse_up(int x, int y){}
	void Fps_cam::middle_mouse_down(int x, int y){}
	void Fps_cam::middle_mouse_up(int x, int y){}

	void Fps_cam::mouse(int x, int y){}

	void Fps_cam::keyboard(char key, int x, int y)
	{
		switch(key)
		{
			case 'w':	move_forwards();		break;
			case 's':	move_backwards();		break;
			case 'a':	strafe_left();			break;
			case 'd':	strafe_right();			break;
			case 'z':	drop();					break;
			case 'x':	lift();					break;
			case 'i':	look_up();				break;
			case 'j':	look_left();			break;
			case 'k':	look_down();			break;
			case 'l':	look_right();			break;
		}
	}

	string Fps_cam::to_string()
	{
		string rv = "";
		rv = "location: " + string(location) + ", angle_y: " + std::to_string(angle_y) + ", angle_x: " + std::to_string(angle_x);  
		return rv;
	}


}