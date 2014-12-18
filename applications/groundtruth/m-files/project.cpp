#include "mex.h"
#include <Eigen/Eigen>

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

Matrix4d Cart2T( Vector6d pz )
{
  Matrix4d T;
  double x, y, z, r, p, q;
  x = pz(0);
  y = pz(1);
  z = pz(2);
  r = pz(3);
  p = pz(4);
  q = pz(5);
  double cq, cp, cr, sq, sp, sr;
  cr = cos(r);
  cp = cos(p);
  cq = cos(q);

  sr = sin(r);
  sp = sin(p);
  sq = sin(q);

  T(0, 0) = cp * cq;
  T(0, 1) = -cr * sq + sr * sp * cq;
  T(0, 2) = sr * sq + cr * sp * cq;

  T(1, 0) = cp * sq;
  T(1, 1) = cr * cq + sr * sp * sq;
  T(1, 2) = -sr * cq + cr * sp * sq;

  T(2, 0) = -sp;
  T(2, 1) = sr * cp;
  T(2, 2) = cr * cp;

  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  T.row(3) = Vector4d(0.0, 0.0, 0.0, 1.0);
  return T;
}

double bilinear( double x, double y, double* Im )
{
  int width = 8;
  int px = floor(x);
  int py = floor(y);
  float dx = x - px;
  float dy = y - py;

  if ((px > 6) || (py > 6)) {
    return Im[px + py*width];
  }
  float a = Im[px + py*width];
  float b = Im[px + 1 + py*width];
  float c = Im[px + (py + 1)*width];
  float d = Im[px + 1 + (py + 1)*width];

  return   a*(1 - dx)*(1 - dy) + b*(dx)*(1 - dy)
         + c*(dy)*(1 - dx) + d*(dx * dy);

}

double color( Vector3d pt,
              Vector3d tl, Vector3d tr,
              Vector3d bl, Vector3d br,
              double* tex )

{
  Vector3d o, t, r, f;
  o = (tl - pt);
  t = (tr - pt);
  r = (br - pt);
  f = (bl - pt);

  o = o / o.norm();
  t = t / t.norm();
  r = r / r.norm();
  f = f / f.norm();
  double theta = 0;

  theta += acos(o.dot(t));
  theta += acos(t.dot(r));
  theta += acos(r.dot(f));
  theta += acos(f.dot(o));

  if (fabs(theta - 2*M_PI) < 0.001) {
    Vector3d dx, dy;
    dx = (tr - tl) / 8;
    dy = (bl - tl) / 8;
    float x, y;
    x = (pt - tl).dot(dx) / (dx.norm() * dx.norm());
    y = (pt - tl).dot(dy) / (dy.norm() * dy.norm());
    if ((x >= 0) && (x < 7) && (y >= 0) && (y < 7))
      return bilinear(x, y, tex);
    else
      return 0;
  }
  else {
    return 0;
  }
}

Vector3d intersect(int px, int py, Matrix3d K, Vector6d pose,
                   Vector3d tl, Vector3d tr,
                   Vector3d bl, Vector3d br )
{
  Matrix3d Ki = K.inverse();
  Vector3d p0 = tl;
  Vector3d l0( pose(0), pose(1), pose(2));

  Vector3d n, dx, dy;
  dx = tr - tl;
  dy = bl - tl;
  n = dy.cross( dx );

  Matrix4d T = Cart2T( pose );

  Vector4d homo_pt;
  Vector3d l(px, py, 1);
  l = Ki * l;
  homo_pt(0) = l(0);
  homo_pt(1) = l(1);
  homo_pt(2) = l(2);
  homo_pt(3) = 1;
  homo_pt = T * homo_pt;
  l(0) = homo_pt(0) / homo_pt(3);
  l(1) = homo_pt(1) / homo_pt(3);
  l(2) = homo_pt(2) / homo_pt(3);

  l = l - l0;

  if (l.dot(n) == 0){
    return Vector3d(0, 0, 0);
  }

  double d = (p0 - l0).dot(n) / (l.dot(n));

  Vector3d toRet = d*l + l0;

  //  mexPrintf("Intersection: <%f, %f, %f>\n", toRet(0), toRet(1), toRet(2));
  return toRet;
}

void project( double* im, int w, int h, Matrix3d K,
              Vector6d pose,
              Vector3d tl,
              Vector3d tr,
              Vector3d bl,
              Vector3d br,
              double* tex )
{
  int i = 0;
  int j = 0;
  for (j = 0; j < h; j++ ){
    for (i = 0; i < w; i++){
      im[j + i*h] = color( intersect(i, j, K, pose, tl, tr, bl, br),
                           tl, tr, bl, br, tex);
    }
  }
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
  //  project( w, h, pose, K, tl, tr, bl, br, tex )

  double* im;
  double* _k;
  double* _tl;
  double* _tr;
  double* _bl;
  double* _br;
  double* _pose;
  double* tex;
  int w, h;

  w = mxGetScalar(prhs[0]);
  h = mxGetScalar(prhs[1]);

  _pose = mxGetPr(prhs[2]);
  Map< Matrix< double, 6, 1 > > pose(_pose);

  _k = mxGetPr(prhs[3]);
  Map< Matrix< double, 3, 3 > > K(_k);

  tex = mxGetPr(prhs[8]);

  _tl = mxGetPr( prhs[6] );
  _tr = mxGetPr( prhs[5] );
  _bl = mxGetPr( prhs[7] );
  _br = mxGetPr( prhs[4] );

  Map< Matrix< double, 3, 1 > > tl(_tl);
  Map< Matrix< double, 3, 1 > > tr(_tr);
  Map< Matrix< double, 3, 1 > > bl(_bl);
  Map< Matrix< double, 3, 1 > > br(_br);

  plhs[0] = mxCreateDoubleMatrix(h, w, mxREAL);
  im = mxGetPr(plhs[0]);

  project(im, w, h, K, pose, tl, tr, bl, br, tex);
  return;
}
