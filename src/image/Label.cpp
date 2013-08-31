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

#include <calibu/image/Label.h>

#include <vector>

using namespace std;
using namespace Eigen;

namespace calibu {

inline short RootLabel(vector<PixelClass>& labels, short label )
{
    if( label >= 0 )
    {
        while( labels[label].equiv >= 0)
            label = labels[label].equiv;
    }
    return label;
}

inline void AssignLabel( const int w, const int h, const unsigned char* I, short* label, vector<PixelClass>& labels, int r, int c, unsigned char passval )
{
    if( I[r*w+c] == passval )
    {
        short* labelr = label + r*w;
        short* labelrm1 = labelr - w;
        
        const short lup = r>0? RootLabel(labels,labelrm1[c]) : -1;
        const short lleft = c>0? RootLabel(labels,labelr[c-1]) : -1;
        
        if( lup >= 0 && lleft >= 0 && lup != lleft )
        {
            // merge
            labelr[c] = lup;
            labels[lup].size += labels[lleft].size + 1;
            labels[lleft].equiv = lup;
            labels[lup].bbox.Insert(labels[lleft].bbox);
        }else if( lup >= 0 )
        {
            // assign to lup
            labelr[c] = lup;
            ++labels[lup].size;
            labels[lup].bbox.Insert(c,r);
        }else if( lleft >= 0 )
        {
            // assign to lleft
            labelr[c] = lleft;
            ++labels[lleft].size;
            labels[lleft].bbox.Insert(c,r);
        }else{
            // new label
            labels.push_back( (PixelClass){-1,IRectangle(c,r,c,r),1 } );
            labelr[c] = labels.size()-1;
        }
    }
}

void Label( int w, int h, const unsigned char* I, short* label, vector<PixelClass>& labels, unsigned char passval )
{
    std::fill(label,label+w*h, -1);
    
    for( int d=0; d < max(w,h); ++d )
    {
        if( d<w )
        {
            for( int r=0; r< std::min(d,h); ++r )
                AssignLabel(w,h,I,label,labels,r,d, passval);
        }
        
        if( d<h )
        {
            for( int c=0; c<= std::min(d,w); ++c )
                AssignLabel(w,h,I,label,labels,d,c, passval);
        }
    }
    
}

}
