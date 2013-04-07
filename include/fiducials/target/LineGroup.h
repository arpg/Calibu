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

#include <Eigen/Eigen>
#include <vector>
#include <array>
#include <algorithm>
#include <fiducials/conics/Conic.h>

namespace fiducials {

const static int GRID_INVALID = std::numeric_limits<int>::min();

struct Triple;

struct Vertex
{
    inline Vertex(size_t id, const Conic& c)
        : id(id), conic(c), pc(c.center), pg(GRID_INVALID,GRID_INVALID)
    {
    }
    
    inline bool HasGridPosition()
    {
        return pg(0) != GRID_INVALID;
    }
        
    size_t id;
    Conic conic;
    Eigen::Vector2d pc;
    Eigen::Vector2i pg;
    std::vector<Triple> triples;
};

struct Triple
{
    inline Triple(Vertex& o1, Vertex& c, Vertex& o2)
    {
        vs = {{&o1, &c, &o2}};
        
        // enforce canonical order to aid comparisons
        if( vs[2]->id < vs[0]->id ) {
            std::swap( vs[0], vs[2]);
        }
    }
    
    inline Vertex& Center() { return *vs[1]; }
    inline const Vertex& Center() const { return *vs[1]; }
    
    inline Vertex& Neighbour(size_t i) { return i ? *vs[2] : *vs[0]; }    
    inline const Vertex& Neighbour(size_t i) const { return i ? *vs[2] : *vs[0]; }
    
    inline Vertex& Vert(size_t i) { return *vs[i]; }
    inline const Vertex& Vert(size_t i) const { return *vs[i]; }

    inline bool Contains(const Vertex& v) const
    {
        const bool found = std::find(vs.begin(), vs.end(), &v) != vs.end();
        return found;
    }
    
    // Colinear sequence of vertices, v[0], v[1], v[2]. v[1] is center
    std::array<Vertex*,3> vs;
};

bool operator==(const Vertex& lhs, const Vertex& rhs)
{
    return lhs.id == rhs.id;
}

bool operator==(const Triple& lhs, const Triple& rhs)
{
    // This will actually compare pointers, but that's okay.
    return std::equal( lhs.vs.begin(), lhs.vs.end(), rhs.vs.begin());
}

bool AreCollinear(const Triple& t1, const Triple& t2)
{
    return t1.Contains(t2.Center()) && t2.Contains(t1.Center());
}

double Distance(const Vertex& v1, const Vertex& v2)
{
    return (v2.pc - v1.pc).norm();
}








struct Opposite
{
    Opposite() {}
    Opposite(const Triple& t)
    {
        c = t.Center().id;
        o1 = t.Neighbour(0).id;
        o2 = t.Neighbour(1).id;
    }

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
