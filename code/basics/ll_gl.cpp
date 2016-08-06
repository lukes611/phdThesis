#include "ll_gl.h"

using namespace ll_R3;
using namespace std;
using namespace ll_siobj;

namespace ll_gl
{
	//default stuff
	void default_viewing(ll_cam::ll_camera & camera)
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glFrustum(-1, 1, -1, 1, 1.5, 2000000.0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		look_at(camera);
	}
	void default_init(bool smooth_shading, bool lighting_on, R3 background_color)
	{
		glClearColor (background_color.x, background_color.y, background_color.z, 0.0);

		if(smooth_shading)		glShadeModel (GL_SMOOTH);
		else					glShadeModel (GL_FLAT);
		
		glEnable(GL_DEPTH_TEST);
		
		if(lighting_on)
		{
			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);
		}
	}
	void default_lighting()
	{
		//Point Lighting
		//GLfloat light_position[] = {camera.cam.eye.x,camera.cam.eye.y,camera.cam.eye.z, 1.0f};
		//GLfloat light_position[] = {300.0f, 600.0f, 500.0f, 1.0f};
		GLfloat light_position[] = {300.0f, 300.0f, 300.1f, 1.0f};
		//GLfloat light_position[] = {30.6f, 6.0f, -56.2f, 1.0f};
		//The first 3 elements represent the position of the light
		//The fourth parameter must be set to 1, to indicate point lighting

		GLfloat light_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};//the diffuse values (rgba) 
		GLfloat light_ambient[] = {0.5f, 0.5f, 0.5f, 1.0f};//the ambient values, set darker than diffuse
	
		//Now pass parameters to OpenGL, we changing the settings for GL_LIGHT0, but 
		//these functions work with the other lights
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);//pass the position
		glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);//pass the diffuse values
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);//pass the ambient values

	}
	void default_glut_main(string window_name, int width, int height, bool smooth_shading, bool lighting_on, R3 background_color, int window_x, int window_y)
	{
		int argc = 1;
		char * argv[2] = {"ll_gl", NULL};
		glutInit(&argc, argv);
		glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA);
		glutInitWindowSize (width, height); 
		glutInitWindowPosition (window_x, window_y);
		glutCreateWindow (window_name.c_str());
		default_init(smooth_shading, lighting_on, background_color);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable( GL_BLEND );
	}
	void default_lighting(ll_cam::ll_camera & camera)
	{
		R3 eye = camera.cop();
		//Point Lighting
		GLfloat light_position[] = {eye.x, eye.y, eye.z, 1.0f};
		//GLfloat light_position[] = {300.0f, 600.0f, 500.0f, 1.0f};
		//GLfloat light_position[] = {300.0f, 300.0f, 300.1f, 1.0f};
		//GLfloat light_position[] = {30.6f, 6.0f, -56.2f, 1.0f};
		//The first 3 elements represent the position of the light
		//The fourth parameter must be set to 1, to indicate point lighting

		GLfloat light_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};//the diffuse values (rgba) 
		GLfloat light_ambient[] = {0.5f, 0.5f, 0.5f, 1.0f};//the ambient values, set darker than diffuse
	
		//Now pass parameters to OpenGL, we changing the settings for GL_LIGHT0, but 
		//these functions work with the other lights
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);//pass the position
		glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);//pass the diffuse values
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);//pass the ambient values

	}
	void look_at(ll_cam::ll_camera & camera)
	{
		R3 eye = camera.cop();
		R3 at = camera.at();
		R3 up = camera.up();
		gluLookAt(eye.x, eye.y, eye.z, at.x, at.y, at.z, up.x, up.y, up.z);
	}
	void camera_mouse_click(ll_cam::ll_camera & camera, int button, int state, int x, int y)
	{
		if(button == GLUT_LEFT_BUTTON)
		{
			if(state == GLUT_DOWN )
			{
				camera.left_mouse_down(x,y);
			}else if(state == GLUT_UP)
			{
				camera.left_mouse_up(x,y);
			}
		}

		if(button == GLUT_RIGHT_BUTTON)
		{
			if(state == GLUT_DOWN )
			{
				camera.right_mouse_down(x,y);
			}else if(state == GLUT_UP)
			{
				camera.right_mouse_up(x,y);
			}
		}

		if(button == GLUT_MIDDLE_BUTTON)
		{
			if(state == GLUT_DOWN )
			{
				camera.middle_mouse_down(x,y);
			}else if(state == GLUT_UP)
			{
				camera.middle_mouse_up(x,y);
			}
		}
	}
	void turn_on_lights()
	{
		glEnable(GL_LIGHTING);
	}
	void turn_off_lights()
	{
		glDisable(GL_LIGHTING);
	}
	void glR3(const R3 & inp)
	{
		glVertex3f(inp.x, inp.y, inp.z);
	}
	void glR3_normal(const R3 & inp)
	{
		glNormal3f(inp.x, inp.y, inp.z);
	}
	void draw_line(const R3 & a, const R3 & b)
	{
		glBegin(GL_LINES);
			glR3(a);
			glR3(b);
		glEnd();
	}
	void draw_quad(const R3 & a, const R3 & b, const R3 & c, const R3 & d, const R3 & normal)
	{
		glBegin(GL_QUADS);
			glNormal3f(normal.x, normal.y, normal.z);
			glR3(a);
			glR3(b);
			glR3(c);
			glR3(d);
		glEnd();
	}
	void draw_triangle(const R3 & a, const R3 & b, const R3 & c, const R3 & normal)
	{
		glBegin(GL_TRIANGLES);
			glNormal3f(normal.x, normal.y, normal.z);
			glR3(a);
			glR3(b);
			glR3(c);
		glEnd();
	}
	void draw_cube(const R3 & center, float size)
	{
		glPushMatrix();
		glTranslatef(center.x, center.y, center.z);
		glScalef(size, size, size);
		glBegin(GL_LINE_LOOP);
			glVertex3fv(ll_cube_verts::cube_vertices[0]);
			glVertex3fv(ll_cube_verts::cube_vertices[3]);
			glVertex3fv(ll_cube_verts::cube_vertices[2]);
			glVertex3fv(ll_cube_verts::cube_vertices[1]);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex3fv(ll_cube_verts::cube_vertices[2]);
			glVertex3fv(ll_cube_verts::cube_vertices[3]);
			glVertex3fv(ll_cube_verts::cube_vertices[7]);
			glVertex3fv(ll_cube_verts::cube_vertices[6]);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex3fv(ll_cube_verts::cube_vertices[0]);
			glVertex3fv(ll_cube_verts::cube_vertices[4]);
			glVertex3fv(ll_cube_verts::cube_vertices[7]);
			glVertex3fv(ll_cube_verts::cube_vertices[3]);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex3fv(ll_cube_verts::cube_vertices[1]);
			glVertex3fv(ll_cube_verts::cube_vertices[2]);
			glVertex3fv(ll_cube_verts::cube_vertices[6]);
			glVertex3fv(ll_cube_verts::cube_vertices[5]);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex3fv(ll_cube_verts::cube_vertices[4]);
			glVertex3fv(ll_cube_verts::cube_vertices[5]);
			glVertex3fv(ll_cube_verts::cube_vertices[6]);
			glVertex3fv(ll_cube_verts::cube_vertices[7]);
		glEnd();
		glBegin(GL_LINE_LOOP);
			glVertex3fv(ll_cube_verts::cube_vertices[0]);
			glVertex3fv(ll_cube_verts::cube_vertices[1]);
			glVertex3fv(ll_cube_verts::cube_vertices[5]);
			glVertex3fv(ll_cube_verts::cube_vertices[4]);
		glEnd();
		glPopMatrix();
	}
	void fill_cube(const R3 & center, float size)
	{
		glPushMatrix();
		glTranslatef(center.x, center.y, center.z);
		glScalef(size, size, size);
		glBegin(GL_QUADS);
			glVertex3fv(ll_cube_verts::cube_vertices[0]);
			glVertex3fv(ll_cube_verts::cube_vertices[3]);
			glVertex3fv(ll_cube_verts::cube_vertices[2]);
			glVertex3fv(ll_cube_verts::cube_vertices[1]);

			glVertex3fv(ll_cube_verts::cube_vertices[2]);
			glVertex3fv(ll_cube_verts::cube_vertices[3]);
			glVertex3fv(ll_cube_verts::cube_vertices[7]);
			glVertex3fv(ll_cube_verts::cube_vertices[6]);

			glVertex3fv(ll_cube_verts::cube_vertices[0]);
			glVertex3fv(ll_cube_verts::cube_vertices[4]);
			glVertex3fv(ll_cube_verts::cube_vertices[7]);
			glVertex3fv(ll_cube_verts::cube_vertices[3]);

			glVertex3fv(ll_cube_verts::cube_vertices[1]);
			glVertex3fv(ll_cube_verts::cube_vertices[2]);
			glVertex3fv(ll_cube_verts::cube_vertices[6]);
			glVertex3fv(ll_cube_verts::cube_vertices[5]);

			glVertex3fv(ll_cube_verts::cube_vertices[4]);
			glVertex3fv(ll_cube_verts::cube_vertices[5]);
			glVertex3fv(ll_cube_verts::cube_vertices[6]);
			glVertex3fv(ll_cube_verts::cube_vertices[7]);

			glVertex3fv(ll_cube_verts::cube_vertices[0]);
			glVertex3fv(ll_cube_verts::cube_vertices[1]);
			glVertex3fv(ll_cube_verts::cube_vertices[5]);
			glVertex3fv(ll_cube_verts::cube_vertices[4]);
		glEnd();
		glPopMatrix();
	}
	void set_mat_color(const R3 & color, float shininess)
	{
		GLfloat material_specular[] = { 1.0, 1.0, 1.0, 1.0f };
		GLfloat material_diffuse_and_ambient[] = { color.x, color.y, color.z, 1.0f };
		GLfloat material_shininess[] = { shininess };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material_diffuse_and_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse_and_ambient);
	}
	void set_color(const R3 & color)
	{
		glColor3f(color.x, color.y, color.z);
	}
	void draw_object(const SIObj & ob)
	{
		int nt = ob._triangles.size();
		glBegin(GL_TRIANGLES);
		for(int i = 0; i < nt; i++)
		{
			SI_Triangle t = ob._triangles[i];
			SI_Triangle nt = ob._normal_ind[i];
			glR3_normal(ob._normals[nt.a]);
			glR3(ob._points[t.a]);
			glR3_normal(ob._normals[nt.b]);
			glR3(ob._points[t.b]);
			glR3_normal(ob._normals[nt.c]);
			glR3(ob._points[t.c]);
		
		}
		glEnd();
	}
	void draw_object(const SIObj & ob, const vector<R3> & cols)
	{
		int nt = ob._triangles.size();
		glBegin(GL_TRIANGLES);
		for(int i = 0; i < nt; i++)
		{
			SI_Triangle t = ob._triangles[i];
			SI_Triangle nt = ob._normal_ind[i];
			set_mat_color(cols[i], 1.0f);
			//glR3_normal(ob._normals[nt.a]);
			glR3(ob._points[t.a]);
			//glR3_normal(ob._normals[nt.b]);
			glR3(ob._points[t.b]);
			//glR3_normal(ob._normals[nt.c]);
			glR3(ob._points[t.c]);
		
		}
		glEnd();
	}
	void draw_object_edges(const SIObj & ob)
	{
		int nt = ob._triangles.size();
		for(int i = 0; i < nt; i++)
		{
			SI_Triangle t = ob._triangles[i];
			glBegin(GL_LINE_LOOP);
				glR3(ob._points[t.a]);
				glR3(ob._points[t.b]);
				glR3(ob._points[t.c]);
			glEnd();
		}
	}
	void draw_points(const vector<R3> & points)
	{
		int s = static_cast<int>(points.size());
		glBegin(GL_POINTS);
		for(int i = 0; i < s; i++)
			glR3(points[i]);
		glEnd();
	}
	void draw_points(const std::vector<ll_R3::R3> & points, const std::vector<ll_R3::R3> & colors)
	{
		int s = static_cast<int>(points.size());
		glBegin(GL_POINTS);
		for(int i = 0; i < s; i++)
		{
			set_mat_color(colors[i], 1.0f);
			glR3(points[i]);
		}
		glEnd();
	}
	void draw_object_normals(const ll_siobj::SIObj & ob, float scalar)
	{
		int nt = ob._triangles.size();
		glBegin(GL_LINES);
		for(int i = 0; i < nt; i++)
		{
			R3 N = ob._normals[i];
			SI_FullTriangle t = ob.getTriangle(i);
			R3 c = ob.getCenterOFTriangle(i);
			glColor3f(1.0f, 0.0f, 0.0f); 
			glR3(c);
			glColor3f(1.0f, 1.0f, 1.0f);
			c += ob._normals[i] * scalar;
			glR3(c);
		}
	}
	void draw_grid(ll_R3::R3 from, float width, int num_squares, R3 dir1, R3 dir2)
	{
		dir1 *= width;
		dir2 *= width;
		dir1 /= (float) num_squares;
		dir2 /= (float) num_squares;
		R3 p = from, q;
		for(int y = 0; y < num_squares; y++)
		{
			q = p;
			for(int x = 0; x < num_squares; x++)
			{
				draw_line(q, q+dir1);
				draw_line(q, q+dir2);
				draw_line(q+dir1, q+dir1+dir2);
				draw_line(q+dir1+dir2, q+dir2);

				q += dir2;
			}
			p += dir1;
		}
	}
}