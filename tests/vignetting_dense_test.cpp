#include <gtest/gtest.h>
#include <calibu/pcalib/vignetting_dense.h>

namespace calibu
{
namespace testing
{

TEST(DenseVignetting, Constructor)
{
  {
    const int w = 640;
    const int h = 480;
    DenseVignetting<double> vignetting(w, h);
    ASSERT_EQ(w, vignetting.Width());
    ASSERT_EQ(h, vignetting.Height());
    ASSERT_EQ("dense", vignetting.Type());
    ASSERT_EQ(w * h, vignetting.NumParams());
    ASSERT_EQ(w * h, vignetting.GetParams().size());
  }

  {
    const int w = 320;
    const int h = 240;
    DenseVignetting<double> vignetting(w, h);
    ASSERT_EQ(w, vignetting.Width());
    ASSERT_EQ(h, vignetting.Height());
    ASSERT_EQ("dense", vignetting.Type());
    ASSERT_EQ(w * h, vignetting.NumParams());
    ASSERT_EQ(w * h, vignetting.GetParams().size());
  }
}

TEST(DenseVignetting, Params)
{
  const int w = 320;
  const int h = 240;
  DenseVignetting<double> vignetting(w, h);
  Eigen::VectorXd params = vignetting.GetParams();

  ASSERT_EQ(w * h, params.size());

  for (int i = 0; i < params.size(); ++i)
  {
    ASSERT_DOUBLE_EQ(1, params[i]);
  }

  for (int i = 0; i < params.size(); ++i)
  {
    params[i] = 0.1 * i;
  }

  vignetting.SetParams(params);
  params = vignetting.GetParams();

  for (int i = 0; i < params.size(); ++i)
  {
    ASSERT_DOUBLE_EQ(0.1 * i, params[i]);
  }

  ASSERT_THROW(vignetting.SetParams(Eigen::Vector2d()), Exception);
  ASSERT_THROW(vignetting.SetParams(Eigen::Vector4d()), Exception);
}

TEST(DenseVignetting, Attenuation)
{
  const int w = 320;
  const int h = 240;
  Eigen::VectorXd params(w * h);
  DenseVignetting<double> vignetting(w, h);

  for (int i = 0; i < params.size(); ++i)
  {
    params[i] = 0.1 * i;
  }

  vignetting.SetParams(params);

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      const double u = x + 0.5;
      const double v = y + 0.5;
      const int index = y * w + x;
      const double expected = params[index];

      ASSERT_DOUBLE_EQ(expected, vignetting(u, v));

      ASSERT_DOUBLE_EQ(expected, DenseVignetting<double>::GetAttenuation(
          params.data(), u, v, w, h));
    }
  }
}

TEST(DenseVignetting, Reset)
{
  const int w = 320;
  const int h = 240;
  Eigen::VectorXd params(w * h);
  DenseVignetting<double> vignetting(w, h);

  for (int i = 0; i < params.size(); ++i)
  {
    params[i] = 0.1 * i;
  }

  vignetting.SetParams(params);
  vignetting.Reset();

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      const double u = x + 0.5;
      const double v = y + 0.5;
      ASSERT_DOUBLE_EQ(1, vignetting(u, v));
    }
  }

  DenseVignetting<double>::ResetParameters(params.data(), w, h);

  for (int i = 0; i < params.size(); ++i)
  {
    ASSERT_DOUBLE_EQ(1, params[i]);
  }
}

} // namespace testing

}