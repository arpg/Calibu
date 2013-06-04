/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu
   
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

namespace calibu
{

/// Represet min/max range.
struct Range
{
    static Range Open()
    {
        return Range(-std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max());
    }

    static Range Closed()
    {
        return Range(std::numeric_limits<double>::max(),
                     -std::numeric_limits<double>::max());
    }
    
    inline Range()
        : minr(std::numeric_limits<double>::max()),
          maxr(-std::numeric_limits<double>::max())
    {
        // Empty range
    }
    
    inline Range(double minr, double maxr)
        : minr(minr), maxr(maxr)
    {
    }
    
    inline bool Empty() const
    {
        return maxr <= minr;
    }
    
    // Expand range to include v
    inline void Insert(double v)
    {
        minr = std::min(minr,v);
        maxr = std::max(maxr,v);
    }
    
    // Enforce that range is greater than v
    inline void ExcludeLessThan(double v)
    {
        minr = std::max(minr,v);
    }

    // Enforce that range is less than v
    inline void ExcludeGreaterThan(double v)
    {
        maxr = std::min(maxr,v);
    }
    
    inline double Size() const
    {
        return maxr - minr;
    }
    
    double minr;
    double maxr;
};

inline Range Union(const Range& lhs, const Range& rhs)
{
    return Range( std::min(lhs.minr, rhs.minr), std::max(lhs.maxr, rhs.maxr) );
}

inline Range Intersection(const Range& lhs, const Range& rhs)
{
    return Range( std::max(lhs.minr, rhs.minr), std::min(lhs.maxr, rhs.maxr) );
}

inline std::ostream& operator<<(std::ostream& os, const Range& r)
{
    os << "[" << r.minr << ", " << r.maxr << "]";
    return os;
}

}
