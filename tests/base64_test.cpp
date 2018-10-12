#include <gtest/gtest.h>
#include <calibu/pcalib/base64.h>

namespace calibu
{
namespace testing
{

TEST(DoubleEncoder, Mapping)
{
  std::vector<double> values;
  values.push_back(+0.0);
  values.push_back(-0.0);
  values.push_back(+1.0);
  values.push_back(-1.0);
  values.push_back(+1.357103);
  values.push_back(-1.357103);
  values.push_back(+8.03117E13);
  values.push_back(-8.03117E13);
  values.push_back(std::numeric_limits<double>::max());
  values.push_back(std::numeric_limits<double>::min());
  values.push_back(+std::numeric_limits<double>::infinity());
  values.push_back(-std::numeric_limits<double>::infinity());
  values.push_back(std::numeric_limits<double>::quiet_NaN());

  for (double expected : values)
  {
    const uint64_t encoding = DoubleEncoder::Encode(expected);
    const double found = DoubleEncoder::Decode(encoding);
    if (std::isnan(expected)) ASSERT_TRUE(std::isnan(found));
    else ASSERT_DOUBLE_EQ(expected, found);
  }
}

TEST(Base64, Encode)
{
  {
    std::vector<uint8_t> data = { 255 };
    const std::string found = Base64::Encode(data);
    const std::string expected = "/w";
    ASSERT_EQ(expected, found);
  }

  {
    std::vector<uint8_t> data = { 255, 126 };
    const std::string found = Base64::Encode(data);
    const std::string expected = "/34";
    ASSERT_EQ(expected, found);
  }

  {
    std::vector<uint8_t> data = { 0, 1, 2 };
    const std::string found = Base64::Encode(data);
    const std::string expected = "AAEC";
    ASSERT_EQ(expected, found);
  }

  {
    std::vector<uint8_t> data = { 0, 1, 2, 3, 4, 5 };
    const std::string found = Base64::Encode(data);
    const std::string expected = "AAECAwQF";
    ASSERT_EQ(expected, found);
  }
}

TEST(Base64, Decode)
{
  {
    const std::string data = "/w";
    std::vector<uint8_t> expected = { 255, 0 };
    const std::vector<uint8_t> found = Base64::Decode(data);
    ASSERT_EQ(expected.size(), found.size());

    for (size_t i = 0; i < expected.size(); ++i)
    {
      ASSERT_EQ(expected[i], found[i]);
    }
  }

  {
    const std::string data = "/34";
    std::vector<uint8_t> expected = { 255, 126, 0 };
    const std::vector<uint8_t> found = Base64::Decode(data);
    ASSERT_EQ(expected.size(), found.size());

    for (size_t i = 0; i < expected.size(); ++i)
    {
      ASSERT_EQ(expected[i], found[i]);
    }
  }

  {
    const std::string data = "AAEC";
    std::vector<uint8_t> expected = { 0, 1, 2 };
    const std::vector<uint8_t> found = Base64::Decode(data);
    ASSERT_EQ(expected.size(), found.size());

    for (size_t i = 0; i < expected.size(); ++i)
    {
      ASSERT_EQ(expected[i], found[i]);
    }
  }

  {
    const std::string data = "AAECAwQF";
    std::vector<uint8_t> expected = { 0, 1, 2, 3, 4, 5 };
    const std::vector<uint8_t> found = Base64::Decode(data);
    ASSERT_EQ(expected.size(), found.size());

    for (size_t i = 0; i < expected.size(); ++i)
    {
      ASSERT_EQ(expected[i], found[i]);
    }
  }
}

TEST(Base64, Mapping)
{
  std::vector<uint8_t> expected(256);
  std::iota(expected.begin(), expected.end(), 0);
  const std::string encoding = Base64::Encode(expected);
  const std::vector<uint8_t> found = Base64::Decode(encoding);

  ASSERT_LE(expected.size(), found.size());

  for (size_t i = 0; i < expected.size(); ++i)
  {
    ASSERT_EQ(expected[i], found[i]);
  }
}

TEST(Base64Encode, Mapping)
{
  Eigen::MatrixXd matrix(640, 480);
  matrix = Eigen::MatrixXd::Random(640, 480);

  Base64Encoder encoder;
  encoder << matrix;

  const std::string encoding = encoder.str();

  Base64Decoder decoder(encoding);

  Eigen::MatrixXd found(640, 480);
  decoder >> found;

  for (int col = 0; col < matrix.cols(); ++col)
  {
    for (int row = 0; row < matrix.rows(); ++row)
    {
      ASSERT_DOUBLE_EQ(matrix(row, col), found(row, col));
    }
  }
}

} // namespace testing

} // namespace calibu