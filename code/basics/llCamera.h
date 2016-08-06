/*
	ll_lib = Luke Lincoln's Camera classes

	Author: Luke Lincoln

	contents description: 
		Contains some objects and algorithms which are wrappers for opencv3

	depends on: R3
*/
#pragma once
#include "R3.h"
#include <string>

//forward declaration
namespace ll_cam { class ll_camera; class Cad_cam; class Fps_cam; }

namespace ll_cam
{

//abstract camera class
class ll_camera
{
public:
	virtual ll_R3::R3 cop() = 0; //gets the center of projection where the camera is located
	virtual ll_R3::R3 dir() = 0; //gets the vector representing where the camera is facing
	virtual ll_R3::R3 at() = 0; //gets a location where the camera is looking towards
	virtual ll_R3::R3 up() = 0; //gets the up vector based on the facing direction
	virtual ll_R3::R3 right() = 0; //gets the right vector based on the facing direction

	//mouse and keyboard input
	virtual void left_mouse_down(int x, int y) = 0;
	virtual void left_mouse_up(int x, int y) = 0;
	virtual void right_mouse_down(int x, int y) = 0;
	virtual void right_mouse_up(int x, int y) = 0;
	virtual void middle_mouse_down(int x, int y) = 0;
	virtual void middle_mouse_up(int x, int y) = 0;

	virtual void mouse(int x, int y) = 0;

	virtual void keyboard(char key, int x = 0, int y = 0) = 0;
	virtual std::string to_string() = 0;

};

//a cad camera for viewing a scene like a cad application does
class Cad_cam : public ll_camera
{
public:
	float	angle_x,	//the angle about the x-axis
			angle_y,	//the angle about the y-axis
			rad,		//the radius from the origin
			z_loc;		//the location along the z-axis
	

	Cad_cam(float x = 0.0f, float y = 0.0f, float r = 500.0f, float zac = 0.0f);
	Cad_cam(const Cad_cam & c);
	Cad_cam & operator = (const Cad_cam & c);

	ll_R3::R3 cop();

	ll_R3::R3 point_no_z(); //gets the center of projection minus the z_loc change
	ll_R3::R3 at();
	ll_R3::R3 dir();
	ll_R3::R3 right();
	ll_R3::R3 up();

	//input
	void left_mouse_down(int x, int y);
	void left_mouse_up(int x, int y);
	void right_mouse_down(int x, int y);
	void right_mouse_up(int x, int y);
	void middle_mouse_down(int x, int y);
	void middle_mouse_up(int x, int y);

	void mouse(int x, int y);

	void keyboard(char key, int x = 0, int y = 0);

	std::string to_string();
private:

	bool left_pressed, right_pressed, middle_pressed;
	int l_past_x, r_past_x, m_past_x;
	int l_past_y, r_past_y, m_past_y;

	void init();
	void init(const Cad_cam & c2);
	void inc_(float & val, float am, float max = 1000000.0f, float min = 1000000.0f);
	void incX(float am=0.1f);
	void decX(float am=0.1f);
	void incY(float am=0.1f);
	void decY(float am=0.1f);
	void incR(float am=0.1f);
	void decR(float am=0.1f);
	void incZ(float am=0.1f);
	void decZ(float am=0.1f);
};


class Fps_cam : public ll_camera
{
public:
	float angle_y;
	float angle_x;
	float look_speed;
	float move_speed;
	ll_R3::R3 location;

	Fps_cam();
	Fps_cam(ll_R3::R3 location, float angle_y, float angle_x);
	Fps_cam(const Fps_cam & cam);
	Fps_cam & operator = (const Fps_cam & cam);

	ll_R3::R3 cop();
	ll_R3::R3 at();
	ll_R3::R3 dir();
	ll_R3::R3 right();
	ll_R3::R3 up();

	ll_R3::R3 flat_dir();
	ll_R3::R3 flat_right();
	ll_R3::R3 flat_up();

	//input
	void left_mouse_down(int x, int y);
	void left_mouse_up(int x, int y);
	void right_mouse_down(int x, int y);
	void right_mouse_up(int x, int y);
	void middle_mouse_down(int x, int y);
	void middle_mouse_up(int x, int y);

	void mouse(int x, int y);

	void keyboard(char key, int x = 0, int y = 0);

	std::string to_string();


	void move_forwards();
	void move_backwards();
	void strafe_left();
	void strafe_right();
	void look_left();
	void look_right();
	void look_up();
	void look_down();
	void lift();
	void drop();
};



}