#ifndef VOYAHSLAM_IO_READ_IMAGE_H
#define VOYAHSLAM_IO_READ_IMAGE_H

#include <istream>
#include <voyahslam/images/image.h>

namespace voyahslam::io {

voyahslam::images::TImageU8Gray ReadImageFromStream(std::istream &stream);
voyahslam::images::TImageF32Gray ReadImageFromStream32F(std::istream &file);

} // namespace voyahslam::io

#endif // VOYAHSLAM_IO_READ_IMAGE_H
