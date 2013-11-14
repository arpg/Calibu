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

#include <list>

#include <Eigen/Eigen>
#include <vector>
#include <array>
#include <set>
#include <algorithm>
#include <calibu/conics/Conic.h>

namespace calibu {

const static int GRID_INVALID = std::numeric_limits<int>::min();

struct Triple;

struct Vertex
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    inline Vertex(size_t id, const Conic& c)
        : id(id), conic(c), pc(c.center), pg(GRID_INVALID,GRID_INVALID), area(0.0), value(-1)
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
    std::set<Vertex*> neighbours;
    double area;
    int value;
};

struct Triple
{
    inline Triple(Vertex& o1, Vertex& c, Vertex& o2)
    {
        vs = {{&o1, &c, &o2}};
    }

    inline Vertex& Center() { return *vs[1]; }
    inline const Vertex& Center() const { return *vs[1]; }

    inline Vertex& Neighbour(size_t i) { return i ? *vs[2] : *vs[0]; }
    inline const Vertex& Neighbour(size_t i) const { return i ? *vs[2] : *vs[0]; }

    inline Vertex& OtherNeighbour(size_t i) { return i ? *vs[0] : *vs[2]; }
    inline const Vertex& OtherNeighbour(size_t i) const { return i ? *vs[0] : *vs[2]; }

    inline Vertex& Vert(size_t i) { return *vs[i]; }
    inline const Vertex& Vert(size_t i) const { return *vs[i]; }

    inline Eigen::Vector2d Dir() const
    {
        return vs[2]->pc - vs[0]->pc;
    }

    inline bool Contains(const Vertex& v) const
    {
        const bool found = std::find(vs.begin(), vs.end(), &v) != vs.end();
        return found;
    }

    inline bool In(const std::set<Vertex*> bag) const
    {
        return bag.find(vs[0]) != bag.end() && bag.find(vs[2]) != bag.end();
    }

    inline void Reverse()
    {
        std::swap(vs[0], vs[2]);
    }

    // Colinear sequence of vertices, v[0], v[1], v[2]. v[1] is center
    std::array<Vertex*,3> vs;
};

inline bool operator==(const Vertex& lhs, const Vertex& rhs)
{
    return lhs.id == rhs.id;
}

inline bool AreCollinear(const Triple& t1, const Triple& t2)
{
    return t1.Contains(t2.Center()) && t2.Contains(t1.Center());
}

inline double Distance(const Vertex& v1, const Vertex& v2)
{
    return (v2.pc - v1.pc).norm();
}

inline std::ostream& operator<<(std::ostream& os, const Vertex& v)
{
    os << "(" << v.pg.transpose() << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Triple& t)
{
    os << t.Vert(0) << " - " << t.Vert(1) << " - " << t.Vert(2);
    return os;
}

struct LineGroup
{
    LineGroup(const Triple& o)
        : ops{o.vs[0]->id, o.vs[1]->id, o.vs[2]->id}
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
