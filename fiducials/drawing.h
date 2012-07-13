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

#ifndef DRAWING_H
#define DRAWING_H

#ifdef _WIN_
#include <Windows.h>
#endif

#include <GL/glew.h>
#include <GL/gl.h>

#include <Eigen/Dense>

// TODO: Work out why this is required to build in Debug
#undef GL_VERSION_2_0
#undef GL_VERSION_2_1
#undef GL_VERSION_3_0
#undef GL_ARB_gpu_shader_fp64
#include <unsupported/Eigen/OpenGLSupport>

#include "rectangle.h"
#include "target.h"
#include "utils.h"

void DrawRectangle( const IRectangle& r );

void DrawCross( float x, float y, int r = 5 );

void DrawCross( const Eigen::Vector2d& p, int r = 5 );

void DrawCross( const Eigen::Vector3d& p, int r = 5 );

void DrawCircle( const Eigen::Vector2d& p, double radius = 5 );

void DrawTarget( const Target& t, const Eigen::Vector2d& offset, double scale = 1.0, double sat = 1.0, double val = 1.0 );

void DrawTarget( const std::vector<int>& map, const Target& target, const Eigen::Vector2d& offset, double scale = 1.0, double sat = 1.0, double val = 1.0 );

void SetPixelTransferScale( float scale );

void glDrawAxis(float s);

void glDrawFrustrum(
  const Eigen::Matrix3d& Kinv, int w, int h, float scale
);

void glDrawFrustrum(
  const Eigen::Matrix3d& Kinv, int w, int h, const Sophus::SE3& T_wf, float scale
);

void glDrawGrid(float num_lines = 30, float line_delta = 2);

// Inlines

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

inline void glSetFrameOfReferenceF( const Sophus::SE3& T_wf )
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrix( T_wf.matrix() );
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

//inline Sophus::SO3 RotationRHSBasis_wz(const Eigen::Vector3d& z_w, const Eigen::Vector3d& up_w)
//{
//    const Eigen::Vector3d n = z_w;
//    const Eigen::Vector3d r  = n ^ up_w;
//    const Eigen::Vector3d up  = -(r ^ n);

//    Eigen::Matrix3d R_wn;
//    R_wn.slice<0,0,3,1>() = r.as_col();
//    R_wn.slice<0,1,3,1>() = up.as_col();
//    R_wn.slice<0,2,3,1>() = n.as_col();


//    Matrix is correct - converting to SO3 breaks!
//    Sophus::SO3 Eigen_R_wn(R_wn);
//    std::cout << R_wn << std::endl;
//    std::cout << Eigen_R_wn << std::endl;
//    return Eigen_R_wn;
//}

/// Adapted from From TooN so3.h:
/// creates an SO3 as a rotation that takes Vector a into the direction of Vector b
/// with the rotation axis along a ^ b. If |a ^ b| == 0, it creates the identity rotation.
/// An assertion will fail if Vector a and Vector b are in exactly opposite directions.
/// @param a source Vector
/// @param b target Vector
Sophus::SO3 Rotation(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    Eigen::Vector3d n = a.cross(b);

    if(n.squaredNorm() == 0) {
        //check that the vectors are in the same direction if cross product is 0. If not,
        //this means that the rotation is 180 degrees, which leads to an ambiguity in the rotation axis.
//        assert(a.dot(b)>=0);
        return Sophus::SO3();
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

    return Sophus::SO3(M);
}

// TODO: Check this works.
//inline Sophus::SE3 PlaneBasis_wp(const Eigen::Vector4d& N_w)
//{
//    const Eigen::Vector3d n = N_w.head<3>();
//    const Sophus::SO3 R_wn = Rotation(Eigen::Vector3d(0,0,-1),n);
//    return Sophus::SE3(R_wn, -N_w[3]*n);
//}

inline Sophus::SE3 PlaneBasis_wp(const Eigen::Vector3d& nd_w)
{
    const double d = 1.0 / nd_w.norm();
    const Eigen::Vector3d n = d * nd_w;
    const Sophus::SO3 R_wn = Rotation(Eigen::Vector3d(0,0,-1),n);
    return Sophus::SE3(R_wn, -d*n);
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
    const Sophus::SE3 T_wn = PlaneBasis_wp(nd_w);
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

#endif // DRAWING_H
