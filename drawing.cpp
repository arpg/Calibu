#include "drawing.h"

using namespace std;
using namespace TooN;
using namespace CVD;

// h [0,360)
// s [0,1]
// v [0,1]
void glColorHSV( double hue, double s, double v )
{
  const double h = hue / 60.0;
  const int i = floor(h);
  const double f = (i%2 == 0) ? 1-(h-i) : h-i;
  const double m = v * (1-s);
  const double n = v * (1-s*f);
  switch(i)
  {
  case 0: glColor3d(v,n,m); break;
  case 1: glColor3d(n,v,m); break;
  case 2: glColor3d(m,v,n); break;
  case 3: glColor3d(m,n,v); break;
  case 4: glColor3d(n,m,v); break;
  case 5: glColor3d(v,m,n); break;
  default:
    break;
  }

}

void glColorBin( int bin, int max_bins, double sat, double val )
{
  const double hue = (double)(bin%max_bins) * 360.0 / (double)max_bins;
  glColorHSV(hue,sat,val);
}

void DrawRectangle( const IRectangle& r )
{
  glBegin(GL_LINE_STRIP);
    glVertex2f(r.x1,r.y1);
    glVertex2f(r.x2,r.y1);
    glVertex2f(r.x2,r.y2);
    glVertex2f(r.x1,r.y2);
    glVertex2f(r.x1,r.y1);
  glEnd();
}

void DrawCross( float x, float y, int r )
{
  glBegin(GL_LINES);
    glVertex2f(x,y-r);
    glVertex2f(x,y+r);
    glVertex2f(x-r,y);
    glVertex2f(x+r,y);
  glEnd();
}

void DrawCross( const Vector<2>& p, int r )
{
  DrawCross(p[0],p[1],r);
}

void DrawCircle( const Vector<2>& p, double radius )
{
  glBegin(GL_POLYGON);
  for( double a=0; a< 2*M_PI; a += M_PI/50.0 )
  {
    glVertex2d(
      p[0] + radius * cos(a),
      p[1] + radius * sin(a)
    );
  }
  glEnd();
}

void DrawTarget( const Target& t, const Vector<2>& offset, double scale, double sat, double val )
{
  const double r = t.Radius() * scale;

  for( unsigned int i=0; i<t.circles().size(); ++i )
  {
    const Vector<2> p = t.circles()[i] * scale + offset;
    glColorBin(i,t.circles().size(),sat,val);
    DrawCircle(p,r);
  }
}

void DrawTarget( const vector<int>& map, const Target& target, const Vector<2>& offset, double scale, double sat, double val )
{
  const double r = target.Radius() * scale;

  for( unsigned int i=0; i<map.size(); ++i )
  {
    const int t = map[i];
    if( t >= 0 )
    {
      const Vector<2> p = target.circles()[t] * scale + offset;
      glColorBin(t,target.circles().size(),sat,val);
      DrawCircle(p,r);
    }
  }
}

void glDrawFrustrum( const Matrix<3,3>& Kinv, int w, int h, float scale )
{
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glBegin(GL_TRIANGLE_FAN);
  glVertex3d(0,0,0);
  CVD::glVertex( -scale * Kinv * TooN::makeVector(0,0,1) );
  CVD::glVertex( -scale * Kinv * TooN::makeVector(w,0,1) );
  CVD::glVertex( -scale * Kinv * TooN::makeVector(w,h,1) );
  CVD::glVertex( -scale * Kinv * TooN::makeVector(0,h,1) );
  CVD::glVertex( -scale * Kinv * TooN::makeVector(0,0,1) );
  glEnd();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void glDrawFrustrum( const Matrix<3,3>& Kinv, int w, int h, const TooN::SE3<>& T_wf, float scale )
{
  glSetFrameOfReferenceF(T_wf);
  glDrawFrustrum(Kinv,w,h,scale);
  glUnsetFrameOfReference();
}
