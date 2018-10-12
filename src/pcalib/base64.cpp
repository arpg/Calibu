#include <calibu/pcalib/base64.h>
#include <Eigen/Eigen>

namespace calibu
{

////////////////////////////////////////////////////////////////////////////////

const uint8_t Base64::encoding_map[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

const uint8_t Base64::decoding_map[] =
{
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,62, 0, 0, 0,63,
  52,53,54,55,56,57,58,59,60,61, 0, 0, 0, 0, 0, 0,
   0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,
  15,16,17,18,19,20,21,22,23,24,25, 0, 0, 0, 0, 0,
   0,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
  41,42,43,44,45,46,47,48,49,50,51, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

////////////////////////////////////////////////////////////////////////////////

Base64Encoder::Base64Encoder()
{
}

template <>
Base64Encoder& Base64Encoder::operator<<(const Eigen::MatrixXd& value)
{
  // allocate encoding buffer

  std::vector<uint8_t> buffer(sizeof(uint64_t) * value.size());
  uint64_t* pointer = reinterpret_cast<uint64_t*>(buffer.data());

  // encode each cell of matrix

  for (int col = 0; col < value.cols(); ++col)
  {
    for (int row = 0; row < value.rows(); ++row)
    {
      *pointer = DoubleEncoder::Encode(value(row, col));
      ++pointer;
    }
  }

  // write encoding to stream

  stream_ << Base64::Encode(buffer);

  return *this;
}

std::string Base64Encoder::str() const
{
  return stream_.str();
}

////////////////////////////////////////////////////////////////////////////////

Base64Decoder::Base64Decoder(const std::string data) :
  stream_(data)
{
}

template <>
Base64Decoder& Base64Decoder::operator>>(Eigen::MatrixXd& value)
{
  const size_t bytes = sizeof(uint64_t) * value.size();
  const size_t chars = Base64::GetEncodingSize(bytes);

  std::vector<char> encoding(chars);
  stream_.read(encoding.data(), chars);

  const std::vector<uint8_t> data = Base64::Decode(encoding);
  const uint64_t* pointer = reinterpret_cast<const uint64_t*>(data.data());

  // decode each cell of matrix

  for (int col = 0; col < value.cols(); ++col)
  {
    for (int row = 0; row < value.rows(); ++row)
    {
      value(row, col) = DoubleEncoder::Decode(*pointer);
      ++pointer;
    }
  }

  return *this;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace calibu