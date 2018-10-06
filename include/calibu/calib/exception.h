#pragma once

#include <exception>
#include <string>

#define CALIBU_THROW(text) \
  throw ::calibu::Exception(__LINE__, __FILE__, text)

#define CALIBU_ASSERT_MSG(cond, text) \
  if (!(cond)) CALIBU_THROW(text)

#define CALIBU_ASSERT(cond) \
  CALIBU_ASSERT_MSG(cond, "assertion failed: " #cond)

#ifdef NDEBUG
#define CALIBU_DEBUG_MSG(cond, text)
#define CALIBU_DEBUG(cond)
#else
#define CALIBU_DEBUG_MSG CALIBU_ASSERT_MSG
#define CALIBU_DEBUG CALIBU_ASSERT
#endif

namespace calibu
{

class Exception : public std::exception
{
  public:

    Exception(int line, const std::string& file, const std::string& text) :
      line_(line),
      file_(file),
      text_(text)
    {
      Initialize();
    }

    inline int line() const
    {
      return line_;
    }

    inline const std::string& file() const
    {
      return file_;
    }

    inline const std::string& text() const
    {
      return text_;
    }

    inline const char* what() const throw() override
    {
      return what_.c_str();
    }

  private:

    void Initialize()
    {
      what_ = file_ + "(" + std::to_string(line_) + "): " + text_;
    }

  protected:

    int line_;

    std::string file_;

    std::string text_;

    std::string what_;
};

} // namespace calibu