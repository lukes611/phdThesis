/*
	ll_lib = Luke Lincoln's Camera classes

	Author: Luke Lincoln

	contents description:
		Contains some functionality which works with opengl

	depends on: R3, siobj, llCamera
*/
#pragma once


#include <libs/glut.h>
#include <libs/freeglut.h>

#include "SIObj.h"
#include "llCamera.h"
#include <string>


namespace ll_gl
{

	namespace ll_cube_verts
	{
		const float cube_vertices[8][3] =
		{
			{-0.5,-0.5,0.5},
			{-0.5,0.5,0.5},
			{0.5,0.5,0.5},
			{0.5,-0.5,0.5},
			{-0.5,-0.5,-0.5},
			{-0.5,0.5,-0.5},
			{0.5,0.5,-0.5},
			{0.5,-0.5,-0.5},
		};
	}
	void default_init(bool smooth_shading = true, bool lighting_on = true, ll_R3::R3 background_color = ll_R3::R3());
	void default_lighting();
	void default_lighting(ll_cam::ll_camera & camera);
	void default_viewing(ll_cam::ll_camera & camera);
	void default_glut_main(std::string window_name = "default_window_name", int width = 512, int height = 512, bool smooth_shading = true, bool lighting_on = true, ll_R3::R3 background_color = ll_R3::R3(), int window_x = 100, int window_y = 100);
	void look_at(ll_cam::ll_camera & camera);
	void camera_mouse_click(ll_cam::ll_camera & camera, int button, int state, int x, int y);
	void turn_on_lights();
	void turn_off_lights();
	void glR3(const ll_R3::R3 & inp);
	void glR3_normal(const ll_R3::R3 & inp);
	void draw_line(const ll_R3::R3 & a, const ll_R3::R3 & b);
	void draw_quad(const ll_R3::R3 & a, const ll_R3::R3 & b, const ll_R3::R3 & c, const ll_R3::R3 & d, const ll_R3::R3 & normal);
	void draw_triangle(const ll_R3::R3 & a, const ll_R3::R3 & b, const ll_R3::R3 & c, const ll_R3::R3 & normal);
	void draw_cube(const ll_R3::R3 & center, float edge_length);
	void fill_cube(const ll_R3::R3 & center, float edge_length);
	void set_mat_color(const ll_R3::R3 & color, float shininess);
	void set_color(const ll_R3::R3 & color);
	void draw_object(const ll_siobj::SIObj & ob);
	void draw_object(const ll_siobj::SIObj & ob, const std::vector<ll_R3::R3> & cols);
	void draw_object_edges(const ll_siobj::SIObj & ob);
	void draw_points(const std::vector<ll_R3::R3> & points);
	void draw_points(const std::vector<ll_R3::R3> & points, const std::vector<ll_R3::R3> & colors);
	void draw_object_normals(const ll_siobj::SIObj & ob, float scalar);
	void draw_grid(ll_R3::R3 from, float width, int num_squares, ll_R3::R3 dir1 = ll_R3::R3(1.0f, 0.0f, 0.0f), ll_R3::R3 dir2 = ll_R3::R3(0.0f, 0.0f, 1.0f));
	//void de;/
}



