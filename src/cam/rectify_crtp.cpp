/*
  This file is part of the Calibu Project.
  https://github.com/gwu-robotics/Calibu

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

#include <calibu/cam/rectify_crtp.h>
#include <calibu/utils/Range.h>

namespace calibu
{

  ///////////////////////////////////////////////////////////////////////////////
  void CreateLookupTable(
      const std::shared_ptr<calibu::CameraInterface<double> >& cam_from,
      LookupTable& lut )
  {
    /*
       TODO figure out what K should be for the "new" camera based on
       what part of the original image we want to keep.

    // Work out parameters of "new" linear camera. We want to map range
    // width/height to image via K.
    const calibu::Range range_width
      = calibu::MinMaxRotatedCol( cam_from, Eigen::Matrix3d::Identity() );
    const calibu::Range range_height
      = calibu::MinMaxRotatedRow( cam_from, Eigen::Matrix3d::Identity() );

    printf(" range_width.Size() = %f, range_width.maxr = %f, range_width.minr = %f\n",
        range_width.Size(), range_width.maxr, range_width.minr );
    printf(" range_height.Size() = %f, range_height.maxr = %f, range_height.minr = %f\n",
        range_height.Size(), range_height.maxr, range_height.minr );

    double fu = (cam_from.Width()-1) / range_width.Size();
    double fv = (cam_from.Height()-1) / range_height.Size();
    double u0 = -fu * range_width.minr;
    double v0 = -fv * range_height.minr;
    */

    // This is a hack as there is no guarntee the first 4 params are the K
    // matrix.  Really we should workout what a good linear model would be
    // based on what portion of the original image is on the z=1 plane (e.g.,
    // cameras with FOV > 180 will need to ignore some pixels).
    double fu = cam_from->GetParams()[0];
    double fv = cam_from->GetParams()[1];
    double u0 = cam_from->GetParams()[2];
    double v0 = cam_from->GetParams()[3];

    // linear camera model inv(K) matrix
    Eigen::Matrix3d R_onKinv;
    R_onKinv << 1.0/fu,        0,   -u0 / fu,
                     0,   1.0/fv,   -v0 / fv,
                     0,        0,           1;

    CreateLookupTable( cam_from, R_onKinv, lut );
  }



  ///////////////////////////////////////////////////////////////////////////////
  void CreateLookupTable(
      const std::shared_ptr<calibu::CameraInterface<double>>& cam_from,
      const Eigen::Matrix3d& R_onKinv,
      LookupTable& lut
      )
  {
    const int w = cam_from->Width();
    const int h = cam_from->Height();

    // make sure we have mem in the look up table
    lut.m_vLutPixels.resize( w*h );
    lut.m_nWidth = w;

    for( int r = 0; r < h; ++r) {
      for( int c = 0; c < w; ++c) {
        // Remap
        const Eigen::Vector3d p_o = R_onKinv * Eigen::Vector3d(c,r,1);
        Eigen::Vector2d p_warped = cam_from->Project(p_o);

        // Clamp to valid image coords. This will cause out of image
        // data to be stretched from nearest valid coords with
        // no branching in rectify function.
        p_warped[0] = std::min(std::max(0.0, p_warped[0]), w - 1.0 );
        p_warped[1] = std::min(std::max(0.0, p_warped[1]), h - 1.0 );

        // Truncates the values for the left image
        int u  = (int) p_warped[0];
        int v  = (int) p_warped[1];
        float su = p_warped[0] - (double)u;
        float sv = p_warped[1] - (double)v;

        // Fix pixel access for last row/column to ensure all are in bounds
        if(u == (w-1)) {
          u -= 1;
          su = 1.0;
        }
        if(v == (w-1)) {
          v -= 1;
          sv = 1.0;
        }

        // Pre-compute the bilinear interpolation weights
        BilinearLutPoint p;
        p.idx0 = u + v*w;
        p.idx1 = u + v*w + w;
        p.w00  = (1-su)*(1-sv);
        p.w01  =    su *(1-sv);
        p.w10  = (1-su)*sv;
        p.w11  =     su*sv;

        lut.SetPoint( r, c, p );
      }
    }
  }

  void CreateLookupTable(
      const std::shared_ptr<calibu::CameraInterface<double>>& cam_from,
      const Eigen::Matrix3d& R_onKinv,
      Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& lookup_warp
      )
  {
    for(size_t r = 0; r < cam_from->Height(); ++r) {
      for(size_t c = 0; c < cam_from->Width(); ++c) {
        // Remap
        const Eigen::Vector3d p_o = R_onKinv * Eigen::Vector3d(c,r,1);
        Eigen::Vector2d p_warped = cam_from->Project(p_o);

        // Clamp to valid image coords
        p_warped[0] = std::min(std::max(0.0, p_warped[0]), cam_from->Width() - 1.0 );
        p_warped[1] = std::min(std::max(0.0, p_warped[1]), cam_from->Height() - 1.0 );

        lookup_warp(r,c) = p_warped.cast<float>();
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////
  void Rectify(
      const LookupTable& lut,
      const unsigned char* pInputImageData,
      unsigned char* pOutputRectImageData,
      int w,
      int h
      )
  {
    // Make the most of the continuous block of memory!
    const BilinearLutPoint* ptr   = &lut.m_vLutPixels[0];

    const int nHeight = lut.Height();
    const int nWidth  = lut.Width();

    // Make sure we have been given a correct lookup table.
    assert(w== nWidth && h == nHeight);

    for( int nRow = 0; nRow < nHeight; nRow++ ) {
      for( int nCol = 0; nCol < nWidth; nCol++ ) {
        *pOutputRectImageData++ =
          (char) ( ptr->w00 * pInputImageData[ ptr->idx0 ] +
              ptr->w01 * pInputImageData[ ptr->idx0 + 1 ] +
              ptr->w10 * pInputImageData[ ptr->idx1 ] +
              ptr->w11 * pInputImageData[ ptr->idx1 + 1 ] );
        ptr++;
      }
    }
  }



} // end namespace

