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

#include <pangolin/pangolin.h>
#include <pangolin/gl.h>

#include <Eigen/Dense>
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

inline Sophus::SE3 PlaneBasis_wp(const Eigen::Vector4d& N_w)
{
    const Eigen::Vector3d n = N_w.head<3>();
    const Sophus::SO3 R_wn = Sophus::SO3(Eigen::Vector3d(0,0,-1),n);
    return Sophus::SE3(R_wn, -N_w[3]*n);
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

inline void DrawPlane(const Eigen::Vector4d& N_w, float scale, int grid)
{
    const Sophus::SE3 T_wn = PlaneBasis_wp(N_w);
    glSetFrameOfReferenceF(T_wn);
    Draw_z0(scale,grid);
    glUnsetFrameOfReference();
}

#endif // DRAWING_H
