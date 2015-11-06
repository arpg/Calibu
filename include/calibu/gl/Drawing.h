/*
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#pragma once

#include <pangolin/gl/gldraw.h>

#include <Eigen/Dense>

#include <calibu/Platform.h>
#include <calibu/target/Target.h>
#include <calibu/utils/Rectangle.h>
#include <calibu/utils/Utils.h>
#include <calibu/utils/PlaneBasis.h>

namespace calibu {

inline void glDrawRectPerimeter( const IRectangle& r )
{
    pangolin::glDrawRectPerimeter(r.x1,r.y1, r.x2, r.y2);
}

inline void glDrawPlane(const Eigen::Vector3d& nd_w, float scale, int grid)
{
    const Sophus::SE3d T_wn = PlaneBasis_wp(nd_w);
    pangolin::glSetFrameOfReference(T_wn.matrix());
    pangolin::glDraw_z0(scale,grid);
    pangolin::glUnsetFrameOfReference();
}

inline void glDrawPlane(const Eigen::Vector4d& N_w, float scale, int grid)
{
    // TODO: Verify that this works.
    const Eigen::Vector3d nd_w = N_w.head<3>() / N_w(3);
    glDrawPlane(nd_w, scale, grid);
}

inline void glDrawTarget( const TargetInterface& t, const Eigen::Vector2d& offset, double scale, double sat, double val )
{

    for( unsigned int i=0; i<t.Circles2D().size(); ++i )
    {
        const double r = t.CircleRadius(i) * scale;
        const Eigen::Vector2d p = t.Circles2D()[i] * scale + offset;
        pangolin::glColorBin(i,t.Circles2D().size(),sat,val);
        pangolin::glDrawCircle(p,r);
    }
}

inline void glDrawTarget( const std::vector<int>& map, const TargetInterface& target, const Eigen::Vector2d& offset, double scale, double sat, double val )
{
    const double r = target.CircleRadius() * scale;

    for( unsigned int i=0; i<map.size(); ++i )
    {
        const int t = map[i];
        if( t >= 0 )
        {
            const Eigen::Vector2d p = target.Circles2D()[t] * scale + offset;
            pangolin::glColorBin(t,target.Circles2D().size(),sat,val);
            pangolin::glDrawCircle(p,r);
        }
    }
}

}
