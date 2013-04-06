/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <fiducials/gl/Drawing.h>

using namespace std;
using namespace Eigen;

namespace fiducials {

void glVertex( const Eigen::Vector3d& p )
{
    glVertex3dv(p.data());
}

void DrawRectangle( const IRectangle& r )
{
  glBegin(GL_LINE_STRIP);
    glVertex2f(r.x1,r.y1);
    glVertex2f(r.x2,r.y1);
    glVertex2f(r.x2,r.y2);
    glVertex2f(r.x1,r.y2);
    glVertex2f(r.x1,r.y1);
  glEnd();
}

void DrawCross( float x, float y, int r )
{
  glBegin(GL_LINES);
    glVertex2f(x,y-r);
    glVertex2f(x,y+r);
    glVertex2f(x-r,y);
    glVertex2f(x+r,y);
  glEnd();
}

void DrawCross( float x, float y, float z, int r )
{
  glBegin(GL_LINES);
    glVertex3f(x,y-r,z);
    glVertex3f(x,y+r,z);
    glVertex3f(x-r,y,z);
    glVertex3f(x+r,y,z);
    glVertex3f(x,y,z-r);
    glVertex3f(x,y,z+r);
  glEnd();
}

void DrawCross( const Vector2d& p, int r )
{
  DrawCross(p[0],p[1],r);
}

void DrawCross( const Vector3d& p, int r )
{
  DrawCross(p[0],p[1],p[2],r);
}

void DrawCircle( const Vector2d& p, double radius )
{
  glBegin(GL_POLYGON);
  for( double a=0; a< 2*M_PI; a += M_PI/50.0 )
  {
    glVertex2d(
      p[0] + radius * cos(a),
      p[1] + radius * sin(a)
    );
  }
  glEnd();
}

void DrawTarget( const TargetRandomDot& t, const Vector2d& offset, double scale, double sat, double val )
{
  const double r = t.Radius() * scale;

  for( unsigned int i=0; i<t.Circles2D().size(); ++i )
  {
    const Vector2d p = t.Circles2D()[i] * scale + offset;
    glBinColor(i,t.Circles2D().size(),sat,val);
    DrawCircle(p,r);
  }
}

void DrawTarget( const vector<int>& map, const TargetRandomDot& target, const Vector2d& offset, double scale, double sat, double val )
{
  const double r = target.Radius() * scale;

  for( unsigned int i=0; i<map.size(); ++i )
  {
    const int t = map[i];
    if( t >= 0 )
    {
      const Vector2d p = target.Circles2D()[t] * scale + offset;
      glBinColor(t,target.Circles2D().size(),sat,val);
      DrawCircle(p,r);
    }
  }
}

void DrawAxis(float s)
{
  glBegin(GL_LINES);
  glColor3f(1,0,0);
  glVertex3f(0,0,0);
  glVertex3f(s,0,0);
  glColor3f(0,1,0);
  glVertex3f(0,0,0);
  glVertex3f(0,s,0);
  glColor3f(0,0,1);
  glVertex3f(0,0,0);
  glVertex3f(0,0,s);
  glEnd();
}

void DrawAxis( const Sophus::SE3d& T_wf, float scale )
{
  glSetFrameOfReferenceF(T_wf);
  DrawAxis(scale);
  glUnsetFrameOfReference();
}

void DrawFrustrum( const Matrix3d& Kinv, int w, int h, float scale )
{
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glBegin(GL_TRIANGLE_FAN);
  glVertex3d(0,0,0);
  glVertex( scale * Kinv * Eigen::Vector3d(0,0,1) );
  glVertex( scale * Kinv * Eigen::Vector3d(w,0,1) );
  glVertex( scale * Kinv * Eigen::Vector3d(w,h,1) );
  glVertex( scale * Kinv * Eigen::Vector3d(0,h,1) );
  glVertex( scale * Kinv * Eigen::Vector3d(0,0,1) );
  glEnd();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void DrawFrustrum( const Matrix3d& Kinv, int w, int h, const Sophus::SE3d& T_wf, float scale )
{
  glSetFrameOfReferenceF(T_wf);
  DrawFrustrum(Kinv,w,h,scale);
  glUnsetFrameOfReference();
}

void DrawGrid(float num_lines, float line_delta)
{
    glBegin(GL_LINES);

    for(int i = -num_lines; i < num_lines; i++){
        glVertex3f( line_delta*num_lines, i*line_delta, 0.0);
        glVertex3f(-line_delta*num_lines, i*line_delta, 0.0);

        glVertex3f(i*line_delta,  line_delta*num_lines, 0.0);
        glVertex3f(i*line_delta, -line_delta*num_lines, 0.0);
    }

//    glColor4ub(255, 0, 0, 128);
//    glVertex3f( line_delta*num_lines , 0.0, 0.0);
//    glVertex3f(-line_delta*num_lines , 0.0, 0.0);

//    glColor4ub(0, 255, 0, 128);
//    glVertex3f( 0.0,  line_delta*num_lines, 0.0);
//    glVertex3f( 0.0, -line_delta*num_lines, 0.0);

    glEnd();
}

}
