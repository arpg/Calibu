/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (C) 2010  Hauke Strasdat, Steven Lovegrove
 *                     Imperial College London
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

#include <Eigen/Dense>

namespace fiducials {

class Rectangle
{
public:
  Rectangle(){}

  Rectangle(const Rectangle & r)
  {
    this->x1 = r.x1;
    this->x2 = r.x2;
    this->y2 = r.y2;
    this->y1 = r.y1;
  }

  Rectangle(double x1, double y1, double x2, double y2)
  {
    this->x1 = x1;
    this->x2 = x2;
    this->y2 = y2;
    this->y1 = y1;
  }

  Rectangle(const Eigen::Vector2d & tl, const Eigen::Vector2d & br)
  {
    this->x1 = tl[0];
    this->x2 = br[0];
    this->y2 = br[1];
    this->y1 = tl[1];
  }

  double x1, y1, x2, y2;

  double Width() const
  {
    return std::max(0.,x2-x1);
  }

  double Height() const
  {
    return std::max(0.,y2-y1);
  }

  bool intersectsWith(const Rectangle & other) const
  {
    if(y2 <= other.y1)	return false;
    if(y1    >= other.y2)	return false;
    if(x2  <= other.x1)	return false;
    if(x1   >= other.x2)	return false;
    return true;
  }

  bool contains(const Rectangle & other) const
  {
    if(  y1  > other.y1)   return false;
    if(  x1 > other.x1)  return false;
    if( x2 < other.x2) return false;
    if(y2 < other.y2) return false;
    return true;
  }

  bool contains(const Eigen::Vector2d & v) const
  {
    if(  y1  > v[1])   return false;
    if(  x1 > v[0])  return false;
    if( x2 < v[0]) return false;
    if(y2 < v[1]) return false;
    return true;
  }
};


class IRectangle{
public:
  IRectangle(){}

  IRectangle(const IRectangle & r)
  {
    this->x1 = r.x1;
    this->x2 = r.x2;
    this->y2 = r.y2;
    this->y1 = r.y1;
  }

  IRectangle(int x1, int y1, int x2, int y2)
  {
    this->x1 = x1;
    this->x2 = x2;
    this->y2 = y2;
    this->y1 = y1;
  }

  int x1, y1, x2, y2;

  int Width() const
  {
    return std::max(0,x2+1-x1);
  }

  int Height() const
  {
    return std::max(0,y2+1-y1);
  }
  
  int Area() const
  {
      return Width()*Height();
  }

  bool IntersectsWith(const IRectangle & other) const
  {
    if(y2 < other.y1)	return false;
    if(y1 > other.y2)	return false;
    if(x2 < other.x1)	return false;
    if(x1 > other.x2)	return false;
    return true;
  }

  bool Contains(const IRectangle & other) const
  {
    if(y1 >= other.y1)   return false;
    if(x1 >= other.x1)  return false;
    if(x2 <= other.x2) return false;
    if(y2 <= other.y2) return false;
    return true;
  }

  void Insert( int x, int y)
  {
    x1 = std::min(x1,x);
    x2 = std::max(x2,x);
    y1 = std::min(y1,y);
    y2 = std::max(y2,y);
  }

  void Insert( const IRectangle& d)
  {
    x1 = std::min(x1,d.x1);
    x2 = std::max(x2,d.x2);
    y1 = std::min(y1,d.y1);
    y2 = std::max(y2,d.y2);
  }

  IRectangle Grow( int r ) const
  {
    IRectangle ret(*this);
    ret.x1 -= r;
    ret.x2 += r;
    ret.y1 -= r;
    ret.y2 += r;
    return ret;
  }

  IRectangle Clamp( int minx, int miny, int maxx, int maxy ) const
  {
    IRectangle ret(*this);
    ret.x1 = std::max(minx,ret.x1);
    ret.y1 = std::max(miny,ret.y1);
    ret.x2 = std::min(maxx,ret.x2);
    ret.y2 = std::min(maxy,ret.y2);
    return ret;
  }

  bool Contains( int x, int y ) const
  {
    return x1 <= x && x <= x2 && y1 <= y && y <= y2;
  }

  bool Contains( const Eigen::Vector2d& p ) const
  {
    return Contains(p[0],p[1]);
  }
  
  Eigen::Vector2d Center() const
  {
      return Eigen::Vector2d((x2+x1)/2.0, (y2+y1)/2.0);
  }

};

}
