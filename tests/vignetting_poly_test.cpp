#include <gtest/gtest.h>
#include <calibu/pcalib/vignetting_poly.h>

namespace calibu
{
namespace testing
{

TEST(EvenPoly6Vignetting, Constructor)
{
  {
    const int w = 640;
    const int h = 480;
    EvenPoly6Vignetting<double> vignetting(w, h);
    ASSERT_EQ(w, vignetting.Width());
    ASSERT_EQ(h, vignetting.Height());
    ASSERT_EQ("epoly6", vignetting.Type());
    ASSERT_EQ(3, vignetting.NumParams());
    ASSERT_EQ(3, vignetting.GetParams().size());
  }

  {
    const int w = 320;
    const int h = 240;
    EvenPoly6Vignetting<double> vignetting(w, h);
    ASSERT_EQ(w, vignetting.Width());
    ASSERT_EQ(h, vignetting.Height());
    ASSERT_EQ("epoly6", vignetting.Type());
    ASSERT_EQ(3, vignetting.NumParams());
    ASSERT_EQ(3, vignetting.GetParams().size());
  }
}

TEST(EvenPoly6Vignetting, Params)
{
  const int w = 640;
  const int h = 480;
  EvenPoly6Vignetting<double> vignetting(w, h);
  Eigen::VectorXd params = vignetting.GetParams();

  ASSERT_EQ(3, params.size());
  ASSERT_DOUBLE_EQ(0, params[0]);
  ASSERT_DOUBLE_EQ(0, params[1]);
  ASSERT_DOUBLE_EQ(0, params[2]);

  params = Eigen::Vector3d(0.5, -0.1, 2.1);
  vignetting.SetParams(params);
  params = vignetting.GetParams();

  ASSERT_DOUBLE_EQ( 0.5, params[0]);
  ASSERT_DOUBLE_EQ(-0.1, params[1]);
  ASSERT_DOUBLE_EQ( 2.1, params[2]);

  ASSERT_THROW(vignetting.SetParams(Eigen::Vector2d()), Exception);
  ASSERT_THROW(vignetting.SetParams(Eigen::Vector4d()), Exception);
}

TEST(EvenPoly6Vignetting, Attenuation)
{
  const int w = 320;
  const int h = 240;
  EvenPoly6Vignetting<double> vignetting(w, h);
  Eigen::Vector3d params;

  double* c = params.data();
  const Eigen::Vector2d size(w, h);
  const Eigen::Vector2d center = 0.5 * size;
  const double max_radius = center.norm();

  params = Eigen::Vector3d(0.5, -0.1, 2.1);
  vignetting.SetParams(params);

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      const double u = x + 0.5;
      const double v = y + 0.5;
      const Eigen::Vector2d point(u, v);
      const double radius = (point - center).norm();
      const double ratio = radius / max_radius;

      const double x1 = ratio * ratio;
      const double x2 = x1 * x1;
      const double x3 = x2 * x1;
      const double expected = 1 + c[0] * x1 + c[1] * x2 + c[2] * x3;

      ASSERT_DOUBLE_EQ(expected, vignetting(u, v));

      ASSERT_DOUBLE_EQ(expected, EvenPoly6Vignetting<double>::GetAttenuation(
          c, u, v, w, h));
    }
  }
}

TEST(EvenPoly6Vignetting, Reset)
{
  const int w = 320;
  const int h = 240;
  EvenPoly6Vignetting<double> vignetting(w, h);
  Eigen::Vector3d params(0.5, -0.1, 2.1);
  vignetting.SetParams(params);
  vignetting.Reset();

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      ASSERT_DOUBLE_EQ(1, vignetting(x, y));
    }
  }

  EvenPoly6Vignetting<double>::ResetParameters(params.data(), w, h);
  ASSERT_DOUBLE_EQ(0, params[0]);
  ASSERT_DOUBLE_EQ(0, params[1]);
  ASSERT_DOUBLE_EQ(0, params[2]);
}

} // namespace testing

} // namespace calibu