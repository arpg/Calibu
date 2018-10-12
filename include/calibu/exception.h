#pragma once

#include <exception>
#include <string>

#define CALIBU_THROW(desc) \
  throw ::calibu::Exception(__LINE__, __FILE__, desc)

#define CALIBU_ASSERT_DESC(cond, desc) \
  if (!(cond)) CALIBU_THROW(desc)

#define CALIBU_ASSERT(cond) \
  CALIBU_ASSERT_DESC(cond, "assertion failed: " #cond)

#ifdef NDEBUG
#define CALIBU_DEBUG_DESC(cond, desc)
#define CALIBU_DEBUG(cond)
#else
#define CALIBU_DEBUG_DESC CALIBU_ASSERT_DESC
#define CALIBU_DEBUG CALIBU_ASSERT
#endif

namespace calibu
{

/**
 * Respresents a generic exception thrown from the calibu framework. An
 * Exception object can be created directly or via one of the macros
 * defined in calibu/exception.h
 */
class Exception : public std::exception
{
  public:

    /**
     * Creates an exception with the given metadata
     * @param line line the error occurred on
     * @param file file the error occurred in
     * @param desc description of the error
     */
    Exception(int line, const std::string& file, const std::string& desc) :
      line_(line),
      file_(file),
      desc_(desc)
    {
      Initialize();
    }

    /**
     * Returns the error line number
     * @return line the error occurred on
     */
    inline int line() const
    {
      return line_;
    }

    /**
     * Returns the error source file
     * @return file the error occurred in
     */
    inline const std::string& file() const
    {
      return file_;
    }

    /**
     * Returns the error description
     * @return description of the error
     */
    inline const std::string& desc() const
    {
      return desc_;
    }

    /**
     * Returns a formatted string giving the full error message
     * @return full error message
     */
    inline const char* what() const throw() override
    {
      return what_.c_str();
    }

  private:

    /**
     * Constructs the formated string from the given metadata
     */
    void Initialize()
    {
      what_ = file_ + "(" + std::to_string(line_) + "): " + desc_;
    }

  protected:

     /** Line the error occurred on */
    int line_;

     /** File the error occurred in */
    std::string file_;

     /** Description of the error */
    std::string desc_;

     /** Full error message */
    std::string what_;
};

} // namespace calibu