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
#include <cvd/gl_helpers.h>
#include <TooN/TooN.h>
#include "rectangle.h"
#include "target.h"
#include "utils.h"

void DrawRectangle( const IRectangle& r );

void DrawCross( float x, float y, int r = 5 );

void DrawCross( const TooN::Vector<2>& p, int r = 5 );

void DrawCross( const TooN::Vector<3>& p, int r = 5 );

void DrawCircle( const TooN::Vector<2>& p, double radius = 5 );

void DrawTarget( const Target& t, const TooN::Vector<2>& offset, double scale = 1.0, double sat = 1.0, double val = 1.0 );

void DrawTarget( const std::vector<int>& map, const Target& target, const TooN::Vector<2>& offset, double scale = 1.0, double sat = 1.0, double val = 1.0 );

void SetPixelTransferScale( float scale );

void glDrawAxis(float s);

void glDrawFrustrum(
  const TooN::Matrix<3,3>& Kinv, int w, int h, float scale
);

void glDrawFrustrum(
  const TooN::Matrix<3,3>& Kinv, int w, int h, const TooN::SE3<>& T_wf, float scale
);

void glDrawGrid(float num_lines = 30, float line_delta = 2);

// Inlines

inline void glSetFrameOfReferenceF( const TooN::SE3<>& T_wf )
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  CVD::glMultMatrix( T_4x4(T_wf) );
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

//inline TooN::SO3<> RotationRHSBasis_wz(const TooN::Vector<3>& z_w, const TooN::Vector<3>& up_w)
//{
//    const TooN::Vector<3> n = z_w;
//    const TooN::Vector<3> r  = n ^ up_w;
//    const TooN::Vector<3> up  = -(r ^ n);

//    TooN::Matrix<3,3> R_wn;
//    R_wn.slice<0,0,3,1>() = r.as_col();
//    R_wn.slice<0,1,3,1>() = up.as_col();
//    R_wn.slice<0,2,3,1>() = n.as_col();


//    Matrix is correct - converting to SO3 breaks!
//    TooN::SO3<> toon_R_wn(R_wn);
//    std::cout << R_wn << std::endl;
//    std::cout << toon_R_wn << std::endl;
//    return toon_R_wn;
//}

inline TooN::SE3<> PlaneBasis_wp(const TooN::Vector<4>& N_w)
{
    const TooN::Vector<3> n = N_w.slice<0,3>();
    const TooN::SO3<> R_wn = TooN::SO3<>(TooN::makeVector(0,0,-1),n);
    return TooN::SE3<>(R_wn, -N_w[3]*n);
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

inline void DrawPlane(const TooN::Vector<4>& N_w, float scale, int grid)
{
    const TooN::SE3<> T_wn = PlaneBasis_wp(N_w);
    glSetFrameOfReferenceF(T_wn);
    Draw_z0(scale,grid);
    glUnsetFrameOfReference();
}

#endif // DRAWING_H
