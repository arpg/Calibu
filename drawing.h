/**
 * @author  Steven Lovegrove
 *
 * Copyright (C) 2010  Steven Lovegrove
 *                     Imperial College London
 *
 * drawing.h is part of RobotVision.
 *
 * RobotVision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * RobotVision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * and the GNU Lesser General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DRAWING_H
#define DRAWING_H

#include <pangolin/pangolin.h>
#include <cvd/gl_helpers.h>
#include <TooN/TooN.h>
#include "rectangle.h"
#include "target.h"

void glColorHSV( double hue, double s, double v );

void glColorBin( int bin, int max_bins, double sat = 1.0, double val = 1.0 );

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

// Inlines

template<typename P>
inline TooN::Matrix<4,4,P> T_4x4(const TooN::SE3<P>& T)
{
  TooN::Matrix<4,4,P> ret = TooN::Identity;
  ret.template slice<0,0,3,3>() = T.get_rotation().get_matrix();
  ret.T()[3].template slice<0,3>() = T.get_translation();
  return ret;
}

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

#endif // DRAWING_H
