/* 
   This file is part of the Calibu Project.
https://robotics.gwu.edu/git/calibu

Copyright (C) 2013 George Washington University,
Steven Lovegrove,
Gabe Sibley

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

#include <calibu/cam/Rectify.h>
#include <calibu/utils/Range.h>

namespace calibu
{

    void CreateLookupTable(
            const calibu::CameraModelInterface& cam_from,
            const Eigen::Matrix3d R_onKinv,
            Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& lookup_warp
            )
    {
        for(size_t r = 0; r < cam_from.Height(); ++r) {
            for(size_t c = 0; c < cam_from.Width(); ++c) {
                // Remap
                const Eigen::Vector3d p_o = R_onKinv * Eigen::Vector3d(c,r,1);
                Eigen::Vector2d p_warped = cam_from.Map(calibu::Project(p_o));

                // Clamp to valid image coords
                p_warped[0] = std::min(std::max(0.0, p_warped[0]), cam_from.Width() - 1.0 );
                p_warped[1] = std::min(std::max(0.0, p_warped[1]), cam_from.Height() - 1.0 );
 
//                printf("dst[%d,%d] = in[%f,%f]\n", r, c, p_warped[1], p_warped[0] );
                //            // Set to (0,0)
                //            if(!(0 < p_warped[0] && p_warped[0] < (cam_from.Width()-1)
                //                 && 0 < p_warped[1] && p_warped[1] < (cam_from.Height()-1))) {
                //                p_warped = Eigen::Vector2d::Zero();
                //            }

                lookup_warp(r,c) = p_warped.cast<float>();
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    void CreateLookupTable(
            const calibu::CameraModelInterface& cam_from,
            const Eigen::Matrix3d R_onKinv,
            LookupTable& lut  
            )
    {
        int w = cam_from.Width();
        int h = cam_from.Height();
        for( int r = 0; r < h; ++r) {
            for( int c = 0; c < w; ++c) {
                // Remap
                const Eigen::Vector3d p_o = R_onKinv * Eigen::Vector3d(c,r,1);
                Eigen::Vector2d p_warped = cam_from.Map(calibu::Project(p_o));

                // Clamp to valid image coords
                p_warped[0] = std::min(std::max(0.0, p_warped[0]), cam_from.Width() - 1.0 );
                p_warped[1] = std::min(std::max(0.0, p_warped[1]), cam_from.Height() - 1.0 );

                // Truncates the values for the left image
                int u  = (int) p_warped[0];
                int v  = (int) p_warped[1];
                float su = p_warped[0] - (double)u;
                float sv = p_warped[1] - (double)v;

                // Pre-compute the bilinear interpolation weights
                BilinearLutPoint p;
                if( u<0 || v<0 || u >= w || v >= h ) {
                    // To avoid branching this value is set to 
                    // something 'reasonable', ie the top left pixel
                    p.idx0 = 0;
                    p.idx1 = 0;
                    p.w00  = 1;
                    p.w01  = 0;
                    p.w10  = 0;
                    p.w11  = 0;
                } else {
                    p.idx0 = u + v*w;
                    p.idx1 = u + v*w + w;
                    p.w00  = (1-su)*(1-sv);
                    p.w01  =    su *(1-sv);
                    p.w10  = (1-su)*sv;
                    p.w11  =     su*sv;
                }
                lut.SetPoint( r, c, p );
            }
        }
    }

    void Rectify(
            LookupTable& lut,
            unsigned char* pInputImageData,
            unsigned char* pOutputRectImageData,
            int w,
            int h
            )
    {
        // Make the most of the continuous block of memory!
        BilinearLutPoint* ptr   = &lut.m_vLutPixels[0];

        const int nHeight = lut.Height();
        const int nWidth  = lut.Width();

        // The top left pixel is used as default value in the LUT for out
        // of image access to avoid an if statement
        char cTopLeftPixelVal = pInputImageData[0];
        pInputImageData[0] = 0; // To ensure black will be used when rectifying
        for( int r = 0; r < nHeight; r++ ) {
            for( int c = 0; c < nWidth; c++ ) {
                *pOutputRectImageData++ =
                    (char) ( ptr->w00 * pInputImageData[ ptr->idx0 ] +
                             ptr->w01 * pInputImageData[ ptr->idx0 + 1 ] +
                             ptr->w10 * pInputImageData[ ptr->idx1 ] +
                             ptr->w11 * pInputImageData[ ptr->idx1 + 1 ] );
                ptr++;
            }
        }

        // Set top left back to its initial value
        pInputImageData[0] = cTopLeftPixelVal;
    }

    /*
    float lerp(unsigned char a, unsigned char b, float t)
    {
        return (float)a + t*((float)b-(float)a);
    }           

    void Rectify(
            Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& lookup_warp,
            unsigned char* in,
            unsigned char* out,
            int w, 
            int h
            )
    {
        for(size_t r=0; r < h; ++r) {
            for(size_t c=0; c < w; ++c) {
                const Eigen::Vector2f p = lookup_warp(r,c);

                const float ix = floorf(p[0]);
                const float iy = floorf(p[1]);
                const float fx = p[0] - ix;
                const float fy = p[1] - iy;

                const int pl = (int)ix;
                const int pr = pl + 1;
                const int pt = (int)iy;
                const int pb = pt + 1;

                out[r*w+c] = lerp(
                        lerp( in[pt*w+pl], in[pt*w+pr], fx ),
                        lerp( in[pb*w+pl], in[pb*w+pr], fx ),
                        fy
                        );
            }
        }
    }
    */

} // end namespace

