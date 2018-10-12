#include <gtest/gtest.h>
#include <calibu/pcalib/response_linear.h>

namespace calibu
{
namespace testing
{

TEST(LinearResponse, Constructor)
{
  LinearResponse<double> response;
  ASSERT_EQ(0, response.NumParams());
  ASSERT_EQ("linear", response.Type());
  ASSERT_EQ(0, response.GetRange()[0]);
  ASSERT_EQ(1, response.GetRange()[1]);
  ASSERT_EQ(0, response.GetParams().size());
}

TEST(LinearResponse, Range)
{
  LinearResponse<double> response;
  ASSERT_EQ(0, response.GetRange()[0]);
  ASSERT_EQ(1, response.GetRange()[1]);
  ASSERT_TRUE(response.InRange(0.0));
  ASSERT_TRUE(response.InRange(0.5));
  ASSERT_TRUE(response.InRange(1.0));
  ASSERT_FALSE(response.InRange(-0.01));
  ASSERT_FALSE(response.InRange(+1.01));

  response.SetRange(0, 255);
  ASSERT_EQ(0, response.GetRange()[0]);
  ASSERT_EQ(255, response.GetRange()[1]);
  ASSERT_TRUE(response.InRange(0.0));
  ASSERT_TRUE(response.InRange(127.5));
  ASSERT_TRUE(response.InRange(255.0));
  ASSERT_FALSE(response.InRange(-0.01));
  ASSERT_FALSE(response.InRange(+255.01));

  response.SetRange(Eigen::Vector2d(100, 65535));
  ASSERT_EQ(100, response.GetRange()[0]);
  ASSERT_EQ(65535, response.GetRange()[1]);
  ASSERT_TRUE(response.InRange(100.0));
  ASSERT_TRUE(response.InRange(32717.5));
  ASSERT_TRUE(response.InRange(65535.0));
  ASSERT_FALSE(response.InRange(99.99));
  ASSERT_FALSE(response.InRange(65535.01));
}

TEST(LinearResponse, Params)
{
  LinearResponse<double> response;
  ASSERT_EQ(0, response.GetParams().size());
  ASSERT_THROW(response.SetParams(Eigen::Vector2d()), Exception);
}

TEST(LinearResponse, Response)
{
  LinearResponse<double> response;
  double* params = nullptr;

  response.SetRange(0, 255);

  for (int i = 0; i < 255; ++i)
  {
    const double x = i;
    ASSERT_DOUBLE_EQ(x, response(x));
    ASSERT_DOUBLE_EQ(x, LinearResponse<double>::GetResponse(params, x));
  }

#ifndef NDEBUG
  ASSERT_THROW(response(-1), Exception);
  ASSERT_THROW(response(256), Exception);
#endif

  response.SetRange(0, 1);

  for (int i = 0; i < 100; ++i)
  {
    const double x = i / 99.0;
    ASSERT_DOUBLE_EQ(x, response(x));
    ASSERT_DOUBLE_EQ(x, LinearResponse<double>::GetResponse(params, x));
  }

#ifndef NDEBUG
  ASSERT_THROW(response(-0.1), Exception);
  ASSERT_THROW(response(1.1), Exception);
#endif
}

TEST(LinearResponse, Reset)
{
  LinearResponse<double> response;
  response.SetRange(0, 255);
  response.Reset();

  for (int i = 0; i < 255; ++i)
  {
    ASSERT_DOUBLE_EQ(i, response(i));
  }
}

} // namespace testing

} // namespace calibu