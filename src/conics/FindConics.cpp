/* 
   This file is part of the Calibu Project.
   http://robotics.gwu.edu/git/?p=calibu

   Copyright (C) 2013 George Washington University
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

#include <calibu/conics/FindConics.h>

namespace calibu
{

////////////////////////////////////////////////////////////////////////////

template<typename TdI>
Eigen::Matrix3d FindEllipse(
        const int w, const int /*h*/,
        const TdI* dI,
        const IRectangle& r,
        double& /*residual*/
        ) {
    //Precise ellipse estimation without contour point extraction
    //Jean-Nicolas Ouellet, Patrick Hebert
    
    // This normalisation is in the wrong space.
    // We'd be better off normalising image values to [0,1] range.

//    const Eigen::Vector2d c = Eigen::Vector2d(
//                r.x1 + (r.x2-r.x1) / 2.0,
//                r.y1 + (r.y2-r.y1) / 2.0
//                );
    
//    const Eigen::Vector2d f = Eigen::Vector2d(
//                2.0 / r.Width(), 2.0 / r.Height()
//                );
    
//    // Transform to approximately unit circle at origin
//    Eigen::Matrix3d H;
//    H <<
//         f(0), 0, c(0),
//            0, f(1), c(1),
//            0, 0, 1;
    
//    Eigen::Matrix3d Hinv;
//    Hinv <<
//            1/f(0), 0, -c(0)/f(0),
//            0, 1/f(1), -c(1)/f(1),
//            0, 0, 1;

    
    // Form system Ax = b to solve
    Eigen::Matrix<double,5,5> A = Eigen::Matrix<double,5,5>::Zero();
    Eigen::Matrix<double,5,1> b = Eigen::Matrix<double,5,1>::Zero();
    
//    float elementCount = 0;
    for( int v=r.y1; v<=r.y2; ++v )
    {
        const TdI* dIv = dI + v*w;
        for( int u=r.x1; u<=r.x2; ++u )
        {
            // li = (ai,bi,ci)' = (I_ui,I_vi, -dI' x_i)'
            const Eigen::Vector3d d =
                    Eigen::Vector3d(dIv[u][0],dIv[u][1],-(dIv[u][0] * u + dIv[u][1] * v) );
//            const Eigen::Vector3d li = //H.T() * d;
//                    Eigen::Vector3d( d[0]*H(0,0), d[1]*H(1,1), d[0]*H(0,2) + d[1] * H(1,2) + d[2] );
            const Eigen::Vector3d li = d;
            Eigen::Matrix<double,5,1> Ki;
            Ki << li[0]*li[0], li[0]*li[1], li[1]*li[1], li[0]*li[2], li[1]*li[2];
            A += Ki*Ki.transpose();
            b += -Ki*li[2]*li[2];
//            elementCount++;
        }
    }
    
    const Eigen::Matrix<double,5,1> x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    
    //  //compute the risidual on the system to see if the algebraic error is too large.
    //  //note: maybe there is a better error metric.
    //  const Eigen::Vector<5> error =  A*x - b;
    //  residual = error*error;
    //  //residual/=elementCount;
    
    Eigen::Matrix3d C_star_norm;
    C_star_norm << x[0],x[1]/2.0,x[3]/2.0,  x[1]/2.0,x[2],x[4]/2.0,  x[3]/2.0,x[4]/2.0,1.0;
    
//    const Eigen::Matrix3d C = Hinv.transpose() * C_star_norm.inverse() * Hinv;
    const Eigen::Matrix3d C = C_star_norm.inverse();
    //  const Matrix3d C_star = LU<3>(C).get_inverse();
    //  return C_star/C_star[2][2];
    
    return C; //C/C(2,2);
}

////////////////////////////////////////////////////////////////////////////

template<typename TdI>
void FindConics(
        const int w, const int h,
        const std::vector<PixelClass>& candidates,
        const TdI* dI,
        std::vector<Conic>& conics
        ) {
    for( unsigned int i=0; i<candidates.size(); ++i )
    {
        const IRectangle region = candidates[i].bbox;
        
        Conic conic;
        double residual = 0;
        conic.C = FindEllipse(w,h,dI,region, residual);
        
        conic.bbox = region;
        conic.Dual = conic.C.inverse();
        conic.Dual /= conic.Dual(2,2);
        conic.center = Eigen::Vector2d(conic.Dual(0,2),conic.Dual(1,2));
        
        const double max_dist = (region.Width() + region.Height()) / 8.0;
        if( (conic.center - region.Center()).norm() < max_dist)
            conics.push_back( conic );
    }
}

////////////////////////////////////////////////////////////////////////////

void FindCandidateConicsFromLabels(
        unsigned w, unsigned h,
        const std::vector<PixelClass>& labels,
        std::vector<PixelClass>& candidates,
        float min_area,    float max_area,
        float min_density, float min_aspect
        ) {
    const int border = 3;
    
    for( unsigned int i=0; i<labels.size(); ++i )
    {
        if( labels[i].equiv == -1 )
        {
            const IRectangle& r = labels[i].bbox;
            // reject rectangles clipped by camera view
            if( r.x1 >= border && r.y1 >= border && r.x2 < (int)w-border && r.y2 < (int)h-border)
            {
                const float area = r.Area();
                if( min_area <= area && area <= max_area )
                {
                    const float aspect = (float)r.Width() / (float)r.Height();
                    if( min_aspect < aspect && aspect < 1.0 / min_aspect )
                    {
                        const double density = (double)labels[i].size / (double)area;
                        if( min_density <=  density )
                        {
                            PixelClass candidate = labels[i];
                            candidate.bbox = r.Grow(2).Clamp(border,border,w-(1+border),h-(1+border));
                            candidates.push_back(candidate);
                        }
                    }
                    
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////
#include <Eigen/Eigen>

template Eigen::Matrix3d FindEllipse( const int, const int, const Eigen::Vector2f*, const IRectangle&, double& );
template void FindConics( const int, const int, const std::vector<PixelClass>& candidates, const Eigen::Vector2f* dI, std::vector<Conic>& conics );

}
