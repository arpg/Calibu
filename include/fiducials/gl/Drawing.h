/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (C) 2010  Steven Lovegrove, Richard Newcombe, Hauke Strasdat
 *                     Imperial College London
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

#pragma once

#include <fiducials/config.h>

#ifdef _WIN_
#include <Windows.h>
#endif

#include <GL/glew.h>

#ifdef _OSX_
  #include <OpenGL/gl.h>
#else
  #include <GL/gl.h>
#endif

#include <Eigen/Dense>

#include <fiducials/target/TargetRandomDot.h>
#include <fiducials/utils/Rectangle.h>
#include <fiducials/utils/Utils.h>

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

void DrawCross( const Eigen::Vector2d& p, int r )
{
  DrawCross(p[0],p[1],r);
}

void DrawCross( const Eigen::Vector3d& p, int r )
{
  DrawCross(p[0],p[1],p[2],r);
}



// h [0,360)
// s [0,1]
// v [0,1]
inline void glHsvColor( double hue, double s, double v )
{
  const double h = hue / 60.0;
  const int i = floor(h);
  const double f = (i%2 == 0) ? 1-(h-i) : h-i;
  const double m = v * (1-s);
  const double n = v * (1-s*f);
  switch(i)
  {
  case 0: glColor3d(v,n,m); break;
  case 1: glColor3d(n,v,m); break;
  case 2: glColor3d(m,v,n); break;
  case 3: glColor3d(m,n,v); break;
  case 4: glColor3d(n,m,v); break;
  case 5: glColor3d(v,m,n); break;
  default:
    break;
  }
}

inline void glBinColor( int bin, int max_bins, double sat = 1.0, double val = 1.0 )
{
  if( bin >= 0 )
  {
    const double hue = (double)(bin%max_bins) * 360.0 / (double)max_bins;
    glHsvColor(hue,sat,val);
  }else{
    glColor3f(1,1,1);
  }
}

inline void glSetFrameOfReferenceF( const Sophus::SE3d& T_wf )
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixd( T_wf.matrix().data() );
}

inline void glUnsetFrameOfReference()
{
  glPopMatrix();
}

inline void SetPixelTransferScale( float scale )
{
  glPixelTransferf(GL_RED_SCALE, scale);
  glPixelTransferf(GL_GREEN_SCALE, scale);
  glPixelTransferf(GL_BLUE_SCALE, scale);
}

/// Adapted from From TooN so3.h:
/// creates an SO3 as a rotation that takes Vector a into the direction of Vector b
/// with the rotation axis along a ^ b. If |a ^ b| == 0, it creates the identity rotation.
/// An assertion will fail if Vector a and Vector b are in exactly opposite directions.
/// @param a source Vector
/// @param b target Vector
Sophus::SO3d Rotation(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    Eigen::Vector3d n = a.cross(b);

    if(n.squaredNorm() == 0) {
        //check that the vectors are in the same direction if cross product is 0. If not,
        //this means that the rotation is 180 degrees, which leads to an ambiguity in the rotation axis.
//        assert(a.dot(b)>=0);
        return Sophus::SO3d();
    }

    n.normalize();
    Eigen::Matrix3d R1;
    R1.col(0) = a.normalized();
    R1.col(1) = n;
    R1.col(2) = n.cross(R1.col(0));

    Eigen::Matrix3d M;
    M.col(0) = b.normalized();
    M.col(1) = n;
    M.col(2) = n.cross(M.col(0));
    M = M * R1.transpose();

    return Sophus::SO3d(M);
}

inline Sophus::SE3d PlaneBasis_wp(const Eigen::Vector3d& nd_w)
{
    const double d = 1.0 / nd_w.norm();
    const Eigen::Vector3d n = d * nd_w;
    const Sophus::SO3d R_wn = Rotation(Eigen::Vector3d(0,0,-1),n);
    return Sophus::SE3d(R_wn, -d*n);
}

inline void Draw_z0(float scale, int grid)
{
    const float maxord = grid*scale;
    glBegin(GL_LINES);
    for(int i=-grid; i<=grid; ++i )
    {
        glVertex2f(i*scale,-maxord);
        glVertex2f(i*scale,+maxord);
        glVertex2f(-maxord, i*scale);
        glVertex2f(+maxord, i*scale);
    }
    glEnd();
}

inline void DrawPlane(const Eigen::Vector3d& nd_w, float scale, int grid)
{
    const Sophus::SE3d T_wn = PlaneBasis_wp(nd_w);
    glSetFrameOfReferenceF(T_wn);
    Draw_z0(scale,grid);
    glUnsetFrameOfReference();
}

inline void DrawPlane(const Eigen::Vector4d& N_w, float scale, int grid)
{
    // TODO: Verify that this works.
    const Eigen::Vector3d nd_w = N_w.head<3>() / N_w(3);
    DrawPlane(nd_w, scale, grid);
}

void DrawCircle( const Eigen::Vector2d& p, double radius )
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

void DrawTarget( const TargetRandomDot& t, const Eigen::Vector2d& offset, double scale, double sat, double val )
{
  const double r = t.Radius() * scale;

  for( unsigned int i=0; i<t.Circles2D().size(); ++i )
  {
    const Eigen::Vector2d p = t.Circles2D()[i] * scale + offset;
    glBinColor(i,t.Circles2D().size(),sat,val);
    DrawCircle(p,r);
  }
}

void DrawTarget( const std::vector<int>& map, const TargetRandomDot& target, const Eigen::Vector2d& offset, double scale, double sat, double val )
{
  const double r = target.Radius() * scale;

  for( unsigned int i=0; i<map.size(); ++i )
  {
    const int t = map[i];
    if( t >= 0 )
    {
      const Eigen::Vector2d p = target.Circles2D()[t] * scale + offset;
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

void DrawFrustrum( const Eigen::Matrix3d& Kinv, int w, int h, float scale )
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

void DrawFrustrum( const Eigen::Matrix3d& Kinv, int w, int h, const Sophus::SE3d& T_wf, float scale )
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
    glEnd();
}

}
