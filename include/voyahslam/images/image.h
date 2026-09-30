#ifndef VOYAHSLAM_IMAGES_IMAGE_H
#define VOYAHSLAM_IMAGES_IMAGE_H

#include <cassert>
#include <memory>
#include <ostream>

namespace voyahslam::images {

template <typename T, int C> class Image {
public:
  typedef T TScalar;
  static const int Channel = C;

  Image() = default;

  Image(unsigned width, unsigned height)
      : offset_(0), width_(width), height_(height), stride_(width),
        data_(std::shared_ptr<T[]>(new T[width * height * C])) {}

  Image(Image<T, C> &other, unsigned start_x, unsigned start_y, unsigned width,
        unsigned height)
      : Image(other, other.offset_ + (start_y * other.stride_ + start_x) * C,
              width, height) {}

  [[nodiscard]] unsigned Width() const { return width_; }
  [[nodiscard]] unsigned Height() const { return height_; }
  [[nodiscard]] unsigned Channels() const { return C; }
  [[nodiscard]] unsigned Stride() const { return stride_; }
  [[nodiscard]] unsigned Offset() const { return offset_; }

  T *GetData() { return data_.get(); }
  const T *GetData() const { return data_.get(); }

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

  void Set(size_t y, size_t x, size_t c, const T &value) const {
    size_t offset = offset_ + y * stride_ * C + x * C + c;
    data_[offset] = value;
  }

  template <typename K> Image<K, C> Convert() const {
    Image<K, C> result(width_, height_);
    auto image_data_ptr = result.GetData();
    auto data_ptr = GetData();
    for (int i = 0; i < width_ * height_; ++i) {
      image_data_ptr[i] = data_ptr[i];
    }
    return result;
  }

  Image<T, C> &operator*=(T n) {
    for (int i = 0; i < width_ * height_ * C; ++i) {
      data_[i] *= n;
    }
    return *this;
  }

  template <typename K>
  Image<decltype(std::declval<T>() + std::declval<K>()), C>
  operator*(K val) const {
    Image<decltype(std::declval<T>() + std::declval<K>()), C> result(width_,
                                                                     height_);
    auto data = result.GetData();
    for (int i = 0; i < width_ * height_ * C; ++i) {
      data[i] = data_[i] * val;
    }
    return result;
  }

  Image<T, C> &operator=(float value) {
    data_[offset_] = value;
    return *this;
  }

  Image &operator=(Image<T, C> &&other) {
    data_ = std::move(other.data_);
    stride_ = other.stride_;
    width_ = other.width_;
    height_ = other.height_;
    offset_ = other.offset_;
    return *this;
  }

  Image(Image<T, C> &&other) noexcept {
    data_ = std::move(other.data_);
    stride_ = other.stride_;
    width_ = other.width_;
    height_ = other.height_;
    offset_ = other.offset_;
  }

  operator T() const { return data_[offset_]; }

  void CopyTo(Image<T, C> &other) const {
    assert(other.Width() == Width());
    assert(other.Height() == Height());
    if (width_ == stride_ && other.stride_ == stride_) {
      memcpy(other.GetData() + other.offset_, data_.get() + offset_,
             width_ * height_ * C * sizeof(T));
    } else {
      for (size_t y = 0; y < height_; ++y) {
        memcpy(other.GetData() + other.offset_ + y * other.stride_ * C,
               data_.get() + offset_ + y * stride_ * C, width_ * C);
      }
    }
  }

private:
  Image(Image<T, C> &other, unsigned offset, unsigned width, unsigned height)
      : offset_(offset), width_(width), height_(height), stride_(other.stride_),
        data_(other.data_) {}

private:
  size_t offset_;
  unsigned width_;
  unsigned height_;
  unsigned stride_;
  std::shared_ptr<T[]> data_;
};

typedef Image<float, 1> TImageF32Gray;
typedef Image<uint8_t, 1> TImageU8Gray;
typedef TImageF32Gray TWorkingImage;

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
