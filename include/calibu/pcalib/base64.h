#pragma once

#include <algorithm>
#include <sstream>
#include <Eigen/Eigen>

#include <iostream>

namespace calibu
{

/**
 * Handles encoding & decoding double-precision floating-point numbers,
 * ensuring values can be written and read in a platform-independent manner
 */
class DoubleEncoder
{
  protected:

    /** IEEE-754 exponent bias */
    static const int32_t exp_bias = 1023;

    /** IEEE-754 max exponent value */
    static const uint64_t max_exp = (uint64_t(1) << 11) - 1;

    /** IEEE-754 max significand value */
    static const uint64_t max_sig = (uint64_t(1) << 52) - 1;

    /** IEEE-754 representation of zero */
    static const uint64_t zero = 0x0000000000000000ul;

    /** IEEE-754 representation of quiet NaN */
    static const uint64_t qnan = 0x7FF8000000000001ul;

    /** IEEE-754 representation of positive infinity */
    static const uint64_t pinf = 0x7FF0000000000000ul;

    /** IEEE-754 representation of negative infinity */
    static const uint64_t ninf = 0xFFF0000000000000ul;

  public:

    /**
     * Encodes the given double with little-endian, IEEE-754 format
     * @param data value to be encoded
     * @return little-endian, IEEE-754 encoding
     */
    static inline uint64_t Encode(double data)
    {
      uint64_t result;
      if (data == 0) result = zero;
      else if (std::isnan(data)) result = qnan;
      else if (data == +std::numeric_limits<double>::infinity()) result = pinf;
      else if (data == -std::numeric_limits<double>::infinity()) result = ninf;
      else result = EncodeNormal(data);
      Format(result);
      return result;
    }

    /**
     * Decodes the given little-endian, IEEE-754 encoding to a double
     * @param little-endian, IEEE-754 value to be decoded
     * @return data decoded float-point value
     */
    static inline double Decode(uint64_t data)
    {
      Format(data);
      double result;
      if (data == zero) result = 0;
      else if (data == qnan) result = std::numeric_limits<double>::quiet_NaN();
      else if (data == pinf) result = +std::numeric_limits<double>::infinity();
      else if (data == ninf) result = -std::numeric_limits<double>::infinity();
      else result = DecodeNormal(data);
      return result;
    }

  protected:

    /**
     * Encodes the normal double with little-endian, IEEE-754 format
     * @param data normal value to be encoded
     * @return little-endian, IEEE-754 encoding
     */
    static inline uint64_t EncodeNormal(double data)
    {
      int32_t exp;
      uint64_t result = 0;
      const double fsig = std::frexp(data, &exp);
      result |= uint64_t(data < 0) << 63;
      result |= uint64_t(exp + exp_bias) << 52;
      result |= uint64_t(max_sig * 2 * (std::abs(fsig) - 0.5));
      return result;
    }

    /**
     * Decodes the normal little-endian, IEEE-754 encoding to a double
     * @param normal little-endian, IEEE-754 value to be decoded
     * @return data decoded float-point value
     */
    static inline double DecodeNormal(uint64_t data)
    {
      const int8_t sgn = 1 - 2 * int8_t(data >> 63);
      const int16_t exp = int32_t((data >> 52) & max_exp) - exp_bias;
      const double sig = sgn * (double(data & max_sig) / (2 * max_sig) + 0.5);
      return std::ldexp(sig, exp);
    }

    /**
     * Formats the byte order to ensure little-endian ordering
     * @param data value to be formated
     */
    static inline void Format(uint64_t& data)
    {
      if (ShouldReverse()) Reverse(data);
    }

    /**
     * Reverses the byte order of the given 64-bit unsigned integer in-place
     * @param data value for which byte order will be reversed in-place
     */
    static inline void Reverse(uint64_t& data)
    {
      uint8_t* buffer = reinterpret_cast<uint8_t*>(&data);
      std::reverse(buffer, buffer + sizeof(uint64_t));
    }

    /**
     * Checks if byte-arrays should be reversed for big-endian systems
     * @return whether
     */
    static inline bool ShouldReverse()
    {
      union
      {
        uint32_t i;
        uint8_t c[4];
      } value = {0x01000002};

      return value.c[0] == 1;
    }
};

/**
 * Handles encoding of byte arrays as base-64 strings, which can be used when
 * writting and reading large amounts of data to/from text files
 */
class Base64
{
  protected:

    /** Mapping from 6-bit value to base-64 encoding */
    static const uint8_t encoding_map[];

    /** Mapping from base-64 encoding to 6-bit value */
    static const uint8_t decoding_map[];

    /** Number of bits represented by a single base-64 character */
    static const uint8_t encoding_bits = 6;

    /** Number of bits represented by a single byte */
    static const uint8_t decoding_bits = 8;

  public:

    /**
     * Encodes the given byte-array as a base-64 string
     * @param data input byte-array to be encoded
     * @return base-64 encoding of given byte-array
     */
    static inline std::string Encode(const std::vector<uint8_t>& data)
    {
      return Encode(data.data(), data.size());
    }

    /**
     * Encodes the given byte-array as a base-64 string
     * @param data input byte-array to be encoded
     * @param count number of bytes to be encoded
     * @return base-64 encoding of given byte-array
     */
    static std::string Encode(const uint8_t* data, size_t count)
    {
      // allocate output buffer

      const size_t size = GetEncodingSize(count);
      std::string result(size, 'A');

      // encode all complete 3-byte tuples

      for (size_t i = 0; i < count / 3; ++i)
      {
        const uint8_t v0 = data[3 * i + 0];
        const uint8_t v1 = data[3 * i + 1];
        const uint8_t v2 = data[3 * i + 2];
        result[4 * i + 0] = encoding_map[v0 >> 2];
        result[4 * i + 1] = encoding_map[(v0 << 4 | v1 >> 4) & 0x3F];
        result[4 * i + 2] = encoding_map[(v1 << 2 | v2 >> 6) & 0x3F];
        result[4 * i + 3] = encoding_map[v2 & 0x3F];
      }

      // encode any partial 3-byte tuples

      if (count % 3 == 1)
      {
        const uint8_t v0 = data[count - 1];
        result[size - 2] = encoding_map[v0 >> 2];
        result[size - 1] = encoding_map[(v0 << 4) & 0x3F];
      }
      else if (count % 3 == 2)
      {
        const uint8_t v0 = data[count - 2];
        const uint8_t v1 = data[count - 1];
        result[size - 3] = encoding_map[v0 >> 2];
        result[size - 2] = encoding_map[(v0 << 4 | v1 >> 4) & 0x3F];
        result[size - 1] = encoding_map[(v1 << 2) & 0x3F];
      }

      return result;
    }

    /**
     * Decodes the given base-64 string into a byte-array
     * @param data base-64 string to be decoded
     * @return byte-array decoded from given base-64 string
     */
    static inline std::vector<uint8_t> Decode(const std::string& data)
    {
      return Decode(data.data(), data.size());
    }

    /**
     * Decodes the given base-64 string into a byte-array
     * @param data base-64 string to be decoded
     * @return byte-array decoded from given base-64 string
     */
    static inline std::vector<uint8_t> Decode(const std::vector<char>& data)
    {
      return Decode(data.data(), data.size());
    }

    /**
     * Decodes the given base-64 string into a byte-array
     * @param data base-64 string to be decoded
     * @param count number of characters to be decoded
     * @return byte-array decoded from given base-64 string
     */
    static std::vector<uint8_t> Decode(const char* data, size_t count)
    {
      // allocate output buffer

      const size_t size = GetDecodingSize(count);
      std::vector<uint8_t> result(size);

      // decode all complete 4-char tuples

      for (size_t i = 0; i < count / 4; ++i)
      {
        const uint8_t v0 = decoding_map[uint8_t(data[4 * i + 0])];
        const uint8_t v1 = decoding_map[uint8_t(data[4 * i + 1])];
        const uint8_t v2 = decoding_map[uint8_t(data[4 * i + 2])];
        const uint8_t v3 = decoding_map[uint8_t(data[4 * i + 3])];
        result[3 * i + 0] = v0 << 2 | v1 >> 4;
        result[3 * i + 1] = v1 << 4 | v2 >> 2;
        result[3 * i + 2] = v2 << 6 | v3 >> 0;
      }

      // decode any partial 4-char tuples

      if (count % 4 == 1)
      {
        const uint8_t v0 = decoding_map[uint8_t(data[count - 1])];
        result[size - 1] = v0 << 2;
      }
      else if (count % 4 == 2)
      {
        const uint8_t v0 = decoding_map[uint8_t(data[count - 2])];
        const uint8_t v1 = decoding_map[uint8_t(data[count - 1])];
        result[size - 2] = v0 << 2 | v1 >> 4;
        result[size - 1] = v1 << 4;
      }
      else if (count % 4 == 3)
      {
        const uint8_t v0 = decoding_map[uint8_t(data[count - 3])];
        const uint8_t v1 = decoding_map[uint8_t(data[count - 2])];
        const uint8_t v2 = decoding_map[uint8_t(data[count - 1])];
        result[size - 3] = v0 << 2 | v1 >> 4;
        result[size - 2] = v1 << 4 | v2 >> 2;
        result[size - 1] = v2 << 6;
      }

      return result;
    }

    /**
     * Computes the number of base-64 characters required to encode the
     * specified number of bytes
     * @param bytes number of bytes to be encoded
     * @return number of characters required for encoding
     */
    static inline size_t GetEncodingSize(size_t bytes)
    {
      return (decoding_bits * bytes + encoding_bits - 1) / encoding_bits;
    }

    /**
     * Computes the number of bytes required to decode the specified number of
     * base-64 characters
     * @param chars number of base-64 characters to be decoded
     * @return number of bytes required for decoding
     */
    static inline size_t GetDecodingSize(size_t chars)
    {
      return (encoding_bits * chars + decoding_bits - 1) / decoding_bits;
    }
};

class Base64Encoder : public Base64
{
  public:

    Base64Encoder()
    {
    }

    template <typename T>
    inline Base64Encoder& operator<<(const T& matrix)
    {
      // allocate encoding buffer

      std::vector<uint8_t> buffer(sizeof(uint64_t) * matrix.size());
      uint64_t* pointer = reinterpret_cast<uint64_t*>(buffer.data());

      // encode each cell of matrix

      for (int col = 0; col < matrix.cols(); ++col)
      {
        for (int row = 0; row < matrix.rows(); ++row)
        {
          *pointer = DoubleEncoder::Encode(matrix(row, col));
          ++pointer;
        }
      }

      // write encoding to stream

      stream_ << Base64::Encode(buffer);

      return *this;
    }

    inline std::string str() const
    {
      return stream_.str();
    }

  protected:

    std::ostringstream stream_;
};

class Base64Decoder: public Base64
{
  public:

    Base64Decoder(const std::string data) :
      stream_(data)
    {
    }

    template <typename T>
    inline Base64Decoder& operator>>(T& matrix)
    {
      const size_t bytes = sizeof(uint64_t) * matrix.size();
      const size_t chars = Base64::GetEncodingSize(bytes);

      std::vector<char> encoding(chars);
      stream_.read(encoding.data(), chars);

      const std::vector<uint8_t> data = Base64::Decode(encoding);
      const uint64_t* pointer = reinterpret_cast<const uint64_t*>(data.data());

      // decode each cell of matrix

      for (int col = 0; col < matrix.cols(); ++col)
      {
        for (int row = 0; row < matrix.rows(); ++row)
        {
          matrix(row, col) = DoubleEncoder::Decode(*pointer);
          ++pointer;
        }
      }

      return *this;
    }

  protected:

    std::istringstream stream_;
};

} // namespace calibu