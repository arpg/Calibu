#include <gtest/gtest.h>
#include <calibu/pcalib/response_poly.h>

namespace calibu
{
namespace testing
{

TEST(Poly3Response, Constructor)
{
  Poly3Response<double> response;
  ASSERT_EQ(3, response.NumParams());
  ASSERT_EQ("poly3", response.Type());
  ASSERT_EQ(0, response.GetRange()[0]);
  ASSERT_EQ(1, response.GetRange()[1]);
  ASSERT_EQ(3, response.GetParams().size());
}

TEST(Poly3Response, Range)
{
  Poly3Response<double> response;
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

TEST(Poly3Response, Params)
{
  Poly3Response<double> response;
  Eigen::VectorXd params = response.GetParams();

  ASSERT_EQ(3, params.size());
  ASSERT_DOUBLE_EQ(1, params[0]);
  ASSERT_DOUBLE_EQ(0, params[1]);
  ASSERT_DOUBLE_EQ(0, params[2]);

  params = Eigen::Vector3d(0.5, -0.1, 2.1);
  response.SetParams(params);
  params = response.GetParams();
  ASSERT_DOUBLE_EQ( 0.5, params[0]);
  ASSERT_DOUBLE_EQ(-0.1, params[1]);
  ASSERT_DOUBLE_EQ( 2.1, params[2]);

  ASSERT_THROW(response.SetParams(Eigen::Vector2d()), Exception);
  ASSERT_THROW(response.SetParams(Eigen::Vector4d()), Exception);
}

TEST(Poly3Response, Response)
{
  Poly3Response<double> response;
  Eigen::Vector3d params;
  double* c = params.data();

  params = Eigen::Vector3d(0.5, -0.1, 2.1);
  response.SetParams(params);
  response.SetRange(0, 255);

  for (int i = 0; i < 255; ++i)
  {
    const double x1 = i;
    const double x2 = x1 * x1;
    const double x3 = x2 * x1;
    const double expected = c[0] * x1 + c[1] * x2 + c[2] * x3;
    ASSERT_DOUBLE_EQ(expected, Poly3Response<double>::GetResponse(c, x1));
    ASSERT_DOUBLE_EQ(expected, response(x1));
  }

#ifndef NDEBUG
  ASSERT_THROW(response(-1), Exception);
  ASSERT_THROW(response(256), Exception);
#endif

  params = Eigen::Vector3d(2.3, 0.8, -1.7);
  response.SetParams(params);
  response.SetRange(0, 1);

  for (int i = 0; i < 100; ++i)
  {
    const double x1 = i / 99.0;
    const double x2 = x1 * x1;
    const double x3 = x2 * x1;
    const double expected = c[0] * x1 + c[1] * x2 + c[2] * x3;
    ASSERT_DOUBLE_EQ(expected, Poly3Response<double>::GetResponse(c, x1));
    ASSERT_DOUBLE_EQ(expected, response(x1));
  }

#ifndef NDEBUG
  ASSERT_THROW(response(-0.1), Exception);
  ASSERT_THROW(response(1.1), Exception);
#endif
}

TEST(Poly3Response, Reset)
{
  Poly3Response<double> response;
  Eigen::Vector3d params;

  response.SetRange(0, 255);
  response.SetParams(Eigen::Vector3d(0.5, -0.1, 2.1));
  response.Reset();

  params = response.GetParams();
  ASSERT_DOUBLE_EQ(1, params[0]);
  ASSERT_DOUBLE_EQ(0, params[1]);
  ASSERT_DOUBLE_EQ(0, params[2]);

  for (int i = 0; i < 255; ++i)
  {
    ASSERT_DOUBLE_EQ(i, response(i));
  }

  params = Eigen::Vector3d(0.5, -0.1, 2.1);
  Poly3Response<double>::ResetParameters(params.data());
  ASSERT_DOUBLE_EQ(1, params[0]);
  ASSERT_DOUBLE_EQ(0, params[1]);
  ASSERT_DOUBLE_EQ(0, params[2]);
}

TEST(Poly4Response, Constructor)
{
  Poly4Response<double> response;
  ASSERT_EQ(4, response.NumParams());
  ASSERT_EQ("poly4", response.Type());
  ASSERT_EQ(0, response.GetRange()[0]);
  ASSERT_EQ(1, response.GetRange()[1]);
  ASSERT_EQ(4, response.GetParams().size());
}

TEST(Poly4Response, Range)
{
  Poly4Response<double> response;
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

  response.SetRange(Eigen::Vector2d(100, 65545));
  ASSERT_EQ(100, response.GetRange()[0]);
  ASSERT_EQ(65545, response.GetRange()[1]);
  ASSERT_TRUE(response.InRange(100.0));
  ASSERT_TRUE(response.InRange(42717.5));
  ASSERT_TRUE(response.InRange(65545.0));
  ASSERT_FALSE(response.InRange(99.99));
  ASSERT_FALSE(response.InRange(65545.01));
}

TEST(Poly4Response, Params)
{
  Poly4Response<double> response;
  Eigen::VectorXd params = response.GetParams();

  ASSERT_EQ(4, params.size());
  ASSERT_DOUBLE_EQ(1, params[0]);
  ASSERT_DOUBLE_EQ(0, params[1]);
  ASSERT_DOUBLE_EQ(0, params[2]);
  ASSERT_DOUBLE_EQ(0, params[3]);

  params = Eigen::Vector4d(0.5, -0.1, 2.1, 5.6);
  response.SetParams(params);
  params = response.GetParams();
  ASSERT_DOUBLE_EQ( 0.5, params[0]);
  ASSERT_DOUBLE_EQ(-0.1, params[1]);
  ASSERT_DOUBLE_EQ( 2.1, params[2]);
  ASSERT_DOUBLE_EQ( 5.6, params[3]);

  ASSERT_THROW(response.SetParams(Eigen::Vector2d()), Exception);
  ASSERT_THROW(response.SetParams(Eigen::Vector3d()), Exception);
}

TEST(Poly4Response, Response)
{
  Poly4Response<double> response;
  Eigen::Vector4d params;
  double* c = params.data();

  params = Eigen::Vector4d(0.5, -0.1, 2.1, -0.2);
  response.SetParams(params);
  response.SetRange(0, 255);

  for (int i = 0; i < 255; ++i)
  {
    const double x1 = i;
    const double x2 = x1 * x1;
    const double x3 = x2 * x1;
    const double x4 = x2 * x2;
    const double expected = c[0] * x1 + c[1] * x2 + c[2] * x3 + c[3] * x4;
    ASSERT_DOUBLE_EQ(expected, Poly4Response<double>::GetResponse(c, x1));
    ASSERT_DOUBLE_EQ(expected, response(x1));
  }

#ifndef NDEBUG
  ASSERT_THROW(response(-1), Exception);
  ASSERT_THROW(response(256), Exception);
#endif

  params = Eigen::Vector4d(2.4, 0.8, -1.7, 0.1);
  response.SetParams(params);
  response.SetRange(0, 1);

  for (int i = 0; i < 100; ++i)
  {
    const double x1 = i / 99.0;
    const double x2 = x1 * x1;
    const double x3 = x2 * x1;
    const double x4 = x2 * x2;
    const double expected = c[0] * x1 + c[1] * x2 + c[2] * x3 + c[3] * x4;
    ASSERT_DOUBLE_EQ(expected, Poly4Response<double>::GetResponse(c, x1));
    ASSERT_DOUBLE_EQ(expected, response(x1));
  }

#ifndef NDEBUG
  ASSERT_THROW(response(-0.1), Exception);
  ASSERT_THROW(response(1.1), Exception);
#endif
}

TEST(Poly4Response, Reset)
{
  Poly4Response<double> response;
  Eigen::Vector4d params;

  response.SetRange(0, 255);
  response.SetParams(Eigen::Vector4d(0.5, -0.1, 2.1, -0.2));
  response.Reset();

  params = response.GetParams();
  ASSERT_DOUBLE_EQ(1, params[0]);
  ASSERT_DOUBLE_EQ(0, params[1]);
  ASSERT_DOUBLE_EQ(0, params[2]);
  ASSERT_DOUBLE_EQ(0, params[3]);

  for (int i = 0; i < 255; ++i)
  {
    ASSERT_DOUBLE_EQ(i, response(i));
  }

  params = Eigen::Vector4d(0.5, -0.1, 2.1, -0.2);
  Poly4Response<double>::ResetParameters(params.data());
  ASSERT_DOUBLE_EQ(1, params[0]);
  ASSERT_DOUBLE_EQ(0, params[1]);
  ASSERT_DOUBLE_EQ(0, params[2]);
  ASSERT_DOUBLE_EQ(0, params[3]);
}

} // namespace testing

} // namespace calibu