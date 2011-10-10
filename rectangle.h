/**
 * @author  Hauke Strasdat, Steven Lovegrove
 *
 * Copyright (C) 2010  Hauke Strasdat, Steven Lovegrove
 *                     Imperial College London
 */


#ifndef RV_RECTANGLE_H
#define RV_RECTANGLE_H

//#include <cvd/image_io.h>
//#include "maths_utils.h"

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

  Rectangle(const TooN::Vector<2> & tl, const TooN::Vector<2> & br)
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

  bool contains(const TooN::Vector<2> & v) const
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

//    IRectangle(const CVD::ImageRef & tl, const CVD::ImageRef & br)
//    {
//      this->x1 = tl.x;
//      this->x2 = br.x;
//      this->y2 = br.y;
//      this->y1 = tl.y;
//    }

  int x1, y1, x2, y2;

  int Width() const
  {
    return std::max(0,x2-x1);
  }

  int Height() const
  {
    return std::max(0,y2-y1);
  }

  bool IntersectsWith(const IRectangle & other) const
  {
    if(y2 <other.y1)	return false;
    if(y1    > other.y2)	return false;
    if(x2  < other.x1)	return false;
    if(x1   > other.x2)	return false;
    return true;
  }

  bool Contains(const IRectangle & other) const
  {
    if(  y1  >= other.y1)   return false;
    if(  x1 >= other.x1)  return false;
    if( x2 <= other.x2) return false;
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
    ::IRectangle ret(*this);
    ret.x1 -= r;
    ret.x2 += r;
    ret.y1 -= r;
    ret.y2 += r;
    return ret;
  }

//    IRectangle Clamp( const CVD::ImageRef& size ) const
//    {
//      RobotVision::IRectangle ret(*this);
//      ret.x1 = std::max(0,ret.x1);
//      ret.y1 = std::max(0,ret.y1);
//      ret.x2 = std::min(size.x-1,ret.x2);
//      ret.y2 = std::min(size.y-1,ret.y2);
//      return ret;
//    }

  bool Contains( int x, int y ) const
  {
    return x1 <= x && x <= x2 && y1 <= y && y <= y2;
  }

  bool Contains( const TooN::Vector<2>& p ) const
  {
    return Contains(p[0],p[1]);
  }

};

#endif // RV_RECTANGLE_H
