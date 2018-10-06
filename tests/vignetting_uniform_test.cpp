#include <gtest/gtest.h>
#include <calibu/pcalib/vignetting_uniform.h>

namespace calibu
{
namespace testing
{

TEST(UniformVignetting, Constructor)
{
  {
    const int w = 640;
    const int h = 480;
    UniformVignetting<double> vignetting(w, h);
    ASSERT_EQ(w, vignetting.Width());
    ASSERT_EQ(h, vignetting.Height());
    ASSERT_EQ("uniform", vignetting.Type());
    ASSERT_EQ(0, vignetting.NumParams());
    ASSERT_EQ(0, vignetting.GetParams().size());
  }

  {
    const int w = 320;
    const int h = 240;
    UniformVignetting<double> vignetting(w, h);
    ASSERT_EQ(w, vignetting.Width());
    ASSERT_EQ(h, vignetting.Height());
    ASSERT_EQ("uniform", vignetting.Type());
    ASSERT_EQ(0, vignetting.NumParams());
    ASSERT_EQ(0, vignetting.GetParams().size());
  }
}

TEST(UniformVignetting, Params)
{
  const int w = 640;
  const int h = 480;
  UniformVignetting<double> vignetting(w, h);
  ASSERT_EQ(0, vignetting.GetParams().size());
  ASSERT_THROW(vignetting.SetParams(Eigen::Vector2d()), Exception);
}

TEST(UniformVignetting, Attenuation)
{
  const int w = 320;
  const int h = 240;
  UniformVignetting<double> vignetting(w, h);
  const double* params = nullptr;

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      ASSERT_DOUBLE_EQ(1, vignetting(x, y));

      ASSERT_DOUBLE_EQ(1, UniformVignetting<double>::GetAttenuation(
          params, x, y, w, h));
    }
  }
}

TEST(UniformVignetting, Reset)
{
  const int w = 320;
  const int h = 240;
  UniformVignetting<double> vignetting(w, h);

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      ASSERT_DOUBLE_EQ(1, vignetting(x, y));
    }
  }
}

} // namespace testing

} // namespace calibu