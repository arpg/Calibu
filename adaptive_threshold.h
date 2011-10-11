#ifndef ADAPTIVE_THRESHOLD_H
#define ADAPTIVE_THRESHOLD_H

#include <algorithm>
#include <cvd/image.h>

template<typename TI,typename TintI,typename Tout>
void AdaptiveThreshold( const CVD::BasicImage<TI>& I, const CVD::BasicImage<TintI>& intI, CVD::BasicImage<Tout>& out, float threshold, int s, Tout pass, Tout fail )
{
  // Adaptive Thresholding Using the Integral Image
  // Derek Bradley, Gerhard Roth

  const int w = I.size().x;
  const int h = I.size().y;

  for ( int j=0; j<h; ++j )
  {
    const int y1 = std::max(1,j-s);
    const int y2 = std::min(h-1,j+s);

    for( int i=0; i<w; ++i )
    {
      const int x1 = std::max(1,i-s);
      const int x2 = std::min(w-1,i+s);
      const int count = (x2-x1)*(y2-y1);
      const TintI sum = intI[y2][x2] - intI[y1-1][x2] - intI[y2][x1-1] + intI[y1-1][x1-1];
      out[j][i] = (I[j][i]*count <= sum*threshold) ? pass : fail;
    }
  }
}

#endif // ADAPTIVE_THRESHOLD_H
