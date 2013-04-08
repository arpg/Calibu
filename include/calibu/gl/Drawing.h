/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
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

#include <pangolin/gldraw.h>

#include <Eigen/Dense>

#include <calibu/target/TargetRandomDot.h>
#include <calibu/utils/Rectangle.h>
#include <calibu/utils/Utils.h>
#include <calibu/utils/PlaneBasis.h>

namespace calibu {

using namespace pangolin;

void glDrawRectangle( const IRectangle& r )
{
    glBegin(GL_LINE_STRIP);
    glVertex2f(r.x1,r.y1);
    glVertex2f(r.x2,r.y1);
    glVertex2f(r.x2,r.y2);
    glVertex2f(r.x1,r.y2);
    glVertex2f(r.x1,r.y1);
    glEnd();
}

inline void glDrawPlane(const Eigen::Vector3d& nd_w, float scale, int grid)
{
    const Sophus::SE3d T_wn = PlaneBasis_wp(nd_w);
    glSetFrameOfReference(T_wn.matrix());
    glDraw_z0(scale,grid);
    glUnsetFrameOfReference();
}

inline void glDrawPlane(const Eigen::Vector4d& N_w, float scale, int grid)
{
    // TODO: Verify that this works.
    const Eigen::Vector3d nd_w = N_w.head<3>() / N_w(3);
    glDrawPlane(nd_w, scale, grid);
}

void glDrawTarget( const TargetRandomDot& t, const Eigen::Vector2d& offset, double scale, double sat, double val )
{
    const double r = t.Radius() * scale;
    
    for( unsigned int i=0; i<t.Circles2D().size(); ++i )
    {
        const Eigen::Vector2d p = t.Circles2D()[i] * scale + offset;
        glColorBin(i,t.Circles2D().size(),sat,val);
        glDrawCircle(p,r);
    }
}

void glDrawTarget( const std::vector<int>& map, const TargetRandomDot& target, const Eigen::Vector2d& offset, double scale, double sat, double val )
{
    const double r = target.Radius() * scale;
    
    for( unsigned int i=0; i<map.size(); ++i )
    {
        const int t = map[i];
        if( t >= 0 )
        {
            const Eigen::Vector2d p = target.Circles2D()[t] * scale + offset;
            glColorBin(t,target.Circles2D().size(),sat,val);
            glDrawCircle(p,r);
        }
    }
}

}
