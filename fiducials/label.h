#include <cvd/image.h>
#include <cvd/rgb.h>

#include "rectangle.h"

struct PixelClass
{
  int equiv;
  IRectangle bbox;
  int size;
};

void Label(
  const CVD::BasicImage<CVD::byte>& I,
  CVD::BasicImage<short>& label,
  std::vector<PixelClass>& labels
);
