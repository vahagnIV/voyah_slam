#ifndef VOYAHSLAM_IMAGES_IMAGE_H
#define VOYAHSLAM_IMAGES_IMAGE_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <utility>

namespace voyahslam::images {

// A C-channel image of T, stored row-major and interleaved: element (y, x, c)
// of a compact image lives at data[(y * width + x) * C + c].
//
// An image made with Image(width, height) owns a fresh buffer. A view made with
// Image(parent, x, y, w, h) (or with operator[]) SHARES (aliases) the parent's
// buffer: it keeps the buffer alive, and a write through either the view or the
// parent is visible in both. A view is described by Offset() (element index of
// its first pixel in the buffer), Width(), Height() and Stride() (the parent's
// row length in PIXELS, not elements). Every operation below reads and writes
// only the view's own rows and columns, whatever the stride.
//
// A default-constructed image is empty: all sizes are 0 and GetData() is null.
// A moved-from image is empty too.
//
// Image(width, height) throws std::length_error when width * height * C does
// not fit in size_t (the sizes may come from a caller, e.g. the JNI binding).
// For C == 1 on a 64-bit target width * height of two unsigned values always
// fits in size_t, so the check cannot fire there; a byte count the allocator
// cannot serve is then reported by new[] itself (std::bad_array_new_length or
// std::bad_alloc). Either way no size wraps silently.
//
// Offsets are size_t end to end (a view into a buffer of more than 2^32
// elements keeps its offset).
template <typename T, int C> class Image {
public:
  typedef T TScalar;
  static const int Channel = C;

  Image() = default;

  Image(unsigned width, unsigned height)
      : offset_(0), width_(width), height_(height), stride_(width),
        data_(AllocateElements(width, height)) {}

  Image(Image<T, C> &other, unsigned start_x, unsigned start_y, unsigned width,
        unsigned height)
      : Image(other,
              other.offset_ +
                  (static_cast<size_t>(start_y) * other.stride_ + start_x) * C,
              width, height) {}

  [[nodiscard]] unsigned Width() const { return width_; }
  [[nodiscard]] unsigned Height() const { return height_; }
  [[nodiscard]] unsigned Channels() const { return C; }
  [[nodiscard]] unsigned Stride() const { return stride_; }
  [[nodiscard]] size_t Offset() const { return offset_; }

  T *GetData() { return data_.get(); }
  const T *GetData() const { return data_.get(); }

  // Shape-dependent indexing, so that img[y][x][c] reads At(y, x, c):
  // on an image with several rows, img[i] is row i (a width x 1 view);
  // on a single-row image, img[i] is pixel i (a 1x1 view);
  // on a 1x1 image, img[i] is channel i (a 1x1 view of that element).
  Image<T, C> operator[](size_t i) {
    if (height_ == 1) {
      if (width_ == 1) {
        return Image<T, C>(*this, offset_ + i, 1, 1);
      }
      return Image<T, C>(*this, offset_ + i * C, 1, 1);
    }
    return Image<T, C>(*this, offset_ + i * stride_ * C, width_, 1);
  }

  const T &At(size_t y, size_t x, size_t c) const {
    size_t offset = offset_ + y * stride_ * C + x * C + c;
    return data_[offset];
  }

  void Set(size_t y, size_t x, size_t c, const T &value) {
    size_t offset = offset_ + y * stride_ * C + x * C + c;
    data_[offset] = value;
  }

  // A fresh compact image of the view's size, every element cast to K, in the
  // same (y, x, c) order. An empty image gives an empty image.
  template <typename K> Image<K, C> Convert() const {
    if (width_ == 0 || height_ == 0) {
      return Image<K, C>();
    }
    Image<K, C> result(width_, height_);
    K *dst = result.GetData();
    const size_t row_elements = static_cast<size_t>(width_) * C;
    for (size_t y = 0; y < height_; ++y) {
      const T *src = data_.get() + offset_ + y * stride_ * C;
      for (size_t i = 0; i < row_elements; ++i) {
        dst[y * row_elements + i] = static_cast<K>(src[i]);
      }
    }
    return result;
  }

  // In place, row by row over the view: only the view's pixels change.
  Image<T, C> &operator*=(T n) {
    const size_t row_elements = static_cast<size_t>(width_) * C;
    for (size_t y = 0; y < height_; ++y) {
      T *row = data_.get() + offset_ + y * stride_ * C;
      for (size_t i = 0; i < row_elements; ++i) {
        row[i] *= n;
      }
    }
    return *this;
  }

  // A fresh compact image of the view's size, every element multiplied by val.
  template <typename K>
  Image<decltype(std::declval<T>() + std::declval<K>()), C>
  operator*(K val) const {
    using Result = Image<decltype(std::declval<T>() + std::declval<K>()), C>;
    if (width_ == 0 || height_ == 0) {
      return Result();
    }
    Result result(width_, height_);
    auto *dst = result.GetData();
    const size_t row_elements = static_cast<size_t>(width_) * C;
    for (size_t y = 0; y < height_; ++y) {
      const T *src = data_.get() + offset_ + y * stride_ * C;
      for (size_t i = 0; i < row_elements; ++i) {
        dst[y * row_elements + i] = src[i] * val;
      }
    }
    return result;
  }

  Image<T, C> &operator=(float value) {
    data_[offset_] = value;
    return *this;
  }

  // The source is left empty.
  Image &operator=(Image<T, C> &&other) noexcept {
    if (this != &other) {
      data_ = std::move(other.data_);
      stride_ = std::exchange(other.stride_, 0);
      width_ = std::exchange(other.width_, 0);
      height_ = std::exchange(other.height_, 0);
      offset_ = std::exchange(other.offset_, 0);
    }
    return *this;
  }

  Image(Image<T, C> &&other) noexcept
      : offset_(std::exchange(other.offset_, 0)),
        width_(std::exchange(other.width_, 0)),
        height_(std::exchange(other.height_, 0)),
        stride_(std::exchange(other.stride_, 0)),
        data_(std::move(other.data_)) {}

  operator T() const { return data_[offset_]; }

  // Copies this view's pixels into `other` (same width and height), row by
  // row when either side is strided.
  void CopyTo(Image<T, C> &other) const {
    assert(other.Width() == Width());
    assert(other.Height() == Height());
    if (width_ == 0 || height_ == 0) {
      return;
    }
    if (width_ == stride_ && other.stride_ == stride_) {
      std::memcpy(other.GetData() + other.offset_, data_.get() + offset_,
             static_cast<size_t>(width_) * height_ * C * sizeof(T));
    } else {
      for (size_t y = 0; y < height_; ++y) {
        std::memcpy(other.GetData() + other.offset_ + y * other.stride_ * C,
               data_.get() + offset_ + y * stride_ * C,
               static_cast<size_t>(width_) * C * sizeof(T));
      }
    }
  }

private:
  Image(Image<T, C> &other, size_t offset, unsigned width, unsigned height)
      : offset_(offset), width_(width), height_(height), stride_(other.stride_),
        data_(other.data_) {}

  // Element count computed in size_t; throws instead of allocating a wrapped
  // size (see the class comment).
  static std::shared_ptr<T[]> AllocateElements(unsigned width, unsigned height) {
    constexpr size_t kMax = std::numeric_limits<size_t>::max();
    if (width != 0 && height > kMax / width) {
      throw std::length_error("Image size overflows");
    }
    const size_t pixels = static_cast<size_t>(width) * height;
    if (pixels > kMax / C) {
      throw std::length_error("Image size overflows");
    }
    return std::shared_ptr<T[]>(new T[pixels * C]);
  }

private:
  size_t offset_ = 0;
  unsigned width_ = 0;
  unsigned height_ = 0;
  unsigned stride_ = 0;
  std::shared_ptr<T[]> data_;
};

typedef Image<float, 1> TImageF32Gray;
typedef Image<uint8_t, 1> TImageU8Gray;
// Image type feature extraction works on: 8-bit grayscale. Detection and
// descriptors only compare intensities, so float adds memory traffic without
// precision; a GPU backend would read 8-bit textures as normalized floats.
typedef TImageU8Gray TWorkingImage;

template <typename K>
std::ostream &operator<<(std::ostream &stream, const Image<K, 1> &image) {
  for (size_t y = 0; y < image.Height(); ++y) {
    for (size_t x = 0; x < image.Width(); ++x) {
      stream << image.At(y, x, 0) << "\t";
    }
    stream << std::endl;
  }
  return stream;
}

} // namespace voyahslam::images

#endif // VOYAHSLAM_IMAGES_IMAGE_H
