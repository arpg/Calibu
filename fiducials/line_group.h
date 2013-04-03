/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (C) 2013  Steven Lovegrove
 *                     George Washington University
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

#include <list>

namespace fiducials {

struct Opposite
{
    size_t c;
    size_t o1;
    size_t o2;
};

struct LineGroup
{
    LineGroup(Opposite o)
        : ops{o.o1, o.c, o.o2}
    {
    }
    
    bool Merge(LineGroup& o)
    {
        if(last() == o.second() && pen() == o.first() ) {
            ops.insert(ops.end(), std::next(o.ops.begin(),2), o.ops.end() );
            o.ops.clear();
            return true;
        }else if(o.last() == second() && o.pen() == first() ) {
            ops.insert(ops.begin(), o.ops.begin(), std::prev(o.ops.end(),2) );
            o.ops.clear();            
            return true;
        }else if( last() == o.pen() && pen() == o.last() ) {
            ops.insert(ops.end(), std::next(o.ops.rbegin(),2), o.ops.rend() );
            o.ops.clear();
            return true;
        }else if( first() == o.second() && second() == o.first() ) {
            ops.insert(ops.begin(), o.ops.rbegin(), std::prev(o.ops.rend(),2) );
            o.ops.clear();
            return true;
        }
        return false;
    }
    
    void Reverse()
    {
        std::list<size_t> rev_ops;
        rev_ops.insert(rev_ops.begin(), ops.rbegin(), ops.rend());
        ops = rev_ops;
    }

    size_t first() { return *ops.begin(); }
    size_t last() { return *ops.rbegin(); }
    size_t second() { return *std::next(ops.begin()); }
    size_t pen() { return *std::next(ops.rbegin()); }
    
    std::list<size_t> ops;
    double theta;
    int k;
};

}
