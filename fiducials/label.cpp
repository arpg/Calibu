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

#include "label.h"

#include <vector>

using namespace std;
using namespace TooN;
using namespace CVD;

inline short RootLabel(vector<PixelClass>& labels, short label )
{
  if( label >= 0 )
  {
    while( labels[label].equiv >= 0)
      label = labels[label].equiv;
  }
  return label;
}

inline void AssignLabel( const BasicImage<byte>& I, BasicImage<short>& label, vector<PixelClass>& labels, int r, int c )
{
  const bool v = I[r][c] < 128;
  if( v )
  {
    const short lup = r>0? RootLabel(labels,label[r-1][c]) : -1;
    const short lleft = c>0? RootLabel(labels,label[r][c-1]) : -1;

    if( lup >= 0 && lleft >= 0 && lup != lleft )
    {
      // merge
      label[r][c] = lup;
      labels[lup].size += labels[lleft].size + 1;
      labels[lleft].equiv = lup;
      labels[lup].bbox.Insert(labels[lleft].bbox);
    }else if( lup >= 0 )
    {
      // assign to lup
      label[r][c] = lup;
      ++labels[lup].size;
      labels[lup].bbox.Insert(c,r);
    }else if( lleft >= 0 )
    {
      // assign to lleft
      label[r][c] = lleft;
      ++labels[lleft].size;
      labels[lleft].bbox.Insert(c,r);
    }else{
      // new label
      labels.push_back( (PixelClass){-1,IRectangle(c,r,c,r),1 } );
      label[r][c] = labels.size()-1;
    }
  }
}

void Label( const BasicImage<byte>& I, BasicImage<short>& label, vector<PixelClass>& labels )
{
  const int w = I.size().x;
  const int h = I.size().y;

  label.fill(-1);

  for( int d=0; d < max(w,h); ++d )
  {
    if( d<w )
    {
      for( int r=0; r< std::min(d,h); ++r )
        AssignLabel(I,label,labels,r,d);
    }

    if( d<h )
    {
      for( int c=0; c<= std::min(d,w); ++c )
        AssignLabel(I,label,labels,d,c);
    }
  }

}
