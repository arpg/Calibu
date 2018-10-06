#include <gtest/gtest.h>
#include <calibu/exception.h>

namespace calibu
{
namespace testing
{

TEST(Exception, Constructor)
{
  const int expected_line = __LINE__;
  const std::string expected_file = __FILE__;
  const std::string expected_desc = "test description";

  const std::string expected_what = expected_file + "(" +
      std::to_string(expected_line) + "): " + expected_desc;

  Exception exception(expected_line, expected_file, expected_desc);
  ASSERT_EQ(expected_line, exception.line());
  ASSERT_EQ(expected_file, exception.file());
  ASSERT_EQ(expected_desc, exception.desc());
  ASSERT_EQ(expected_what, exception.what());
}

TEST(Exception, Throw)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_desc = "test description";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    CALIBU_THROW(expected_desc);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_desc;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_desc, exception.desc());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }

  ASSERT_TRUE(thrown);
}

TEST(Exception, AssertDescription)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_desc = "test description";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    CALIBU_ASSERT_DESC(0 > 1, expected_desc);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_desc;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_desc, exception.desc());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }

  ASSERT_TRUE(thrown);
  ASSERT_NO_THROW(CALIBU_ASSERT_DESC(0 < 1, ""));
}

TEST(Exception, Assert)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_desc = "assertion failed: 0 > 1";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    CALIBU_ASSERT(0 > 1);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_desc;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_desc, exception.desc());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }

  ASSERT_TRUE(thrown);
  ASSERT_NO_THROW(CALIBU_ASSERT(0 < 1));
}

TEST(Exception, DebugDescription)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_desc = "test description";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    CALIBU_DEBUG_DESC(0 > 1, expected_desc);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_desc;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_desc, exception.desc());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }


#ifdef NDEBUG

  ASSERT_FALSE(thrown);

  int value = 0;
  CALIBU_DEBUG_DESC(++value, "");
  ASSERT_EQ(0, value);

#else

  ASSERT_TRUE(thrown);

#endif

  ASSERT_NO_THROW(CALIBU_DEBUG_DESC(0 < 1, ""));
}

TEST(Exception, Debug)
{
  int expected_line;
  const std::string expected_file = __FILE__;
  const std::string expected_desc = "assertion failed: 0 > 1";
  bool thrown = false;

  try
  {
    expected_line = __LINE__ + 1;
    CALIBU_DEBUG(0 > 1);
  }
  catch (const Exception& exception)
  {
    const std::string expected_what = expected_file + "(" +
        std::to_string(expected_line) + "): " + expected_desc;

    ASSERT_EQ(expected_line, exception.line());
    ASSERT_EQ(expected_file, exception.file());
    ASSERT_EQ(expected_desc, exception.desc());
    ASSERT_EQ(expected_what, exception.what());
    thrown = true;
  }


#ifdef NDEBUG

  ASSERT_FALSE(thrown);

  int value = 0;
  CALIBU_DEBUG(++value);
  ASSERT_EQ(0, value);

#else

  ASSERT_TRUE(thrown);

#endif

  ASSERT_NO_THROW(CALIBU_DEBUG(0 < 1));
}

} // namespace testing

} // namespace calibu