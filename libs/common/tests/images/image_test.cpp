#include <voyahslam/images/image.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace voyahslam::images {
namespace {

// A parent whose every (y, x, c) holds a distinct value (mod 251, a prime
// below 256, so uint8 values do not repeat along a row).
template <typename T = uint8_t> Image<T, 3> MakeParent(unsigned width, unsigned height) {
  Image<T, 3> parent(width, height);
  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      for (unsigned c = 0; c < 3; ++c) {
        parent.Set(y, x, c, static_cast<T>((y * width * 3 + x * 3 + c) % 251));
      }
    }
  }
  return parent;
}

TEST(ImageTest, DefaultIsEmpty) {
  // Default-initialise on a buffer full of garbage, so members without an
  // initialiser show up as non-zero instead of being zeroed by luck.
  alignas(Image<uint8_t, 3>) unsigned char storage[sizeof(Image<uint8_t, 3>)];
  std::memset(storage, 0xAB, sizeof(storage));
  auto *img = new (storage) Image<uint8_t, 3>;  // default-init, not value-init
  EXPECT_EQ(img->Width(), 0u);
  EXPECT_EQ(img->Height(), 0u);
  EXPECT_EQ(img->Stride(), 0u);
  EXPECT_EQ(img->Offset(), 0u);
  EXPECT_EQ(img->GetData(), nullptr);
  img->~Image();
}

TEST(ImageTest, ConvertOfViewReadsOnlyTheView) {
  auto parent = MakeParent(8, 6);
  Image<uint8_t, 3> view(parent, 2, 1, 4, 3);
  Image<float, 3> converted = view.Convert<float>();

  ASSERT_EQ(converted.Width(), 4u);
  ASSERT_EQ(converted.Height(), 3u);
  EXPECT_EQ(converted.Stride(), 4u);
  for (unsigned y = 0; y < 3; ++y) {
    for (unsigned x = 0; x < 4; ++x) {
      for (unsigned c = 0; c < 3; ++c) {
        EXPECT_FLOAT_EQ(converted.At(y, x, c), static_cast<float>(view.At(y, x, c)))
            << "y=" << y << " x=" << x << " c=" << c;
        EXPECT_FLOAT_EQ(converted.At(y, x, c), static_cast<float>(parent.At(y + 1, x + 2, c)));
      }
    }
  }
}

TEST(ImageTest, ConvertOfBorderViewIncludesLastRowAndColumn) {
  auto parent = MakeParent(8, 6);
  Image<uint8_t, 3> view(parent, 5, 4, 3, 2);  // touches the right and bottom borders
  Image<float, 3> converted = view.Convert<float>();

  ASSERT_EQ(converted.Width(), 3u);
  ASSERT_EQ(converted.Height(), 2u);
  for (unsigned y = 0; y < 2; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      for (unsigned c = 0; c < 3; ++c) {
        EXPECT_FLOAT_EQ(converted.At(y, x, c), static_cast<float>(parent.At(y + 4, x + 5, c)));
      }
    }
  }
}

TEST(ImageTest, ConvertOfEmptyIsEmpty) {
  Image<uint8_t, 3> empty;
  Image<float, 3> converted = empty.Convert<float>();
  EXPECT_EQ(converted.Width(), 0u);
  EXPECT_EQ(converted.Height(), 0u);
}

TEST(ImageTest, MultiplyAssignOnViewChangesOnlyTheView) {
  auto parent = MakeParent<float>(8, 6);
  auto reference = MakeParent<float>(8, 6);
  Image<float, 3> view(parent, 2, 1, 4, 3);
  view *= 2.0f;

  for (unsigned y = 0; y < 6; ++y) {
    for (unsigned x = 0; x < 8; ++x) {
      const bool inside = y >= 1 && y < 4 && x >= 2 && x < 6;
      for (unsigned c = 0; c < 3; ++c) {
        const float expected = inside ? 2.0f * reference.At(y, x, c) : reference.At(y, x, c);
        EXPECT_FLOAT_EQ(parent.At(y, x, c), expected) << "y=" << y << " x=" << x << " c=" << c;
      }
    }
  }
}

TEST(ImageTest, MultiplyOfViewReturnsScaledView) {
  auto parent = MakeParent<float>(8, 6);
  Image<float, 3> view(parent, 5, 2, 3, 4);  // touches the right border
  Image<float, 3> scaled = view * 3;

  ASSERT_EQ(scaled.Width(), 3u);
  ASSERT_EQ(scaled.Height(), 4u);
  EXPECT_EQ(scaled.Stride(), 3u);
  EXPECT_EQ(scaled.Offset(), 0u);
  for (unsigned y = 0; y < 4; ++y) {
    for (unsigned x = 0; x < 3; ++x) {
      for (unsigned c = 0; c < 3; ++c) {
        EXPECT_FLOAT_EQ(scaled.At(y, x, c), 3.0f * parent.At(y + 2, x + 5, c));
      }
    }
  }
  // The source is not modified.
  EXPECT_FLOAT_EQ(parent.At(2, 5, 0), MakeParent<float>(8, 6).At(2, 5, 0));
}

TEST(ImageTest, MultiplyOfEmptyIsEmptyAndMultiplyAssignLeavesItEmpty) {
  Image<float, 3> empty;
  Image<float, 3> product = empty * 2;
  EXPECT_EQ(product.Width(), 0u);
  EXPECT_EQ(product.Height(), 0u);
  empty *= 2.0f;
  EXPECT_EQ(empty.Width(), 0u);
  EXPECT_EQ(empty.GetData(), nullptr);
}

TEST(ImageTest, CopyToFloatViewCopiesWholeRows) {
  auto source_parent = MakeParent<float>(8, 6);
  auto target_parent = Image<float, 3>(8, 6);
  for (unsigned y = 0; y < 6; ++y)
    for (unsigned x = 0; x < 8; ++x)
      for (unsigned c = 0; c < 3; ++c) target_parent.Set(y, x, c, -1.0f);

  Image<float, 3> source(source_parent, 2, 1, 4, 3);  // stride 8 != width 4
  Image<float, 3> target(target_parent, 3, 2, 4, 3);
  source.CopyTo(target);

  for (unsigned y = 0; y < 6; ++y) {
    for (unsigned x = 0; x < 8; ++x) {
      const bool inside = y >= 2 && y < 5 && x >= 3 && x < 7;
      for (unsigned c = 0; c < 3; ++c) {
        const float expected = inside ? source_parent.At(y - 2 + 1, x - 3 + 2, c) : -1.0f;
        EXPECT_FLOAT_EQ(target_parent.At(y, x, c), expected)
            << "y=" << y << " x=" << x << " c=" << c;
      }
    }
  }
}

TEST(ImageTest, CopyToBetweenEmptyImagesIsANoOp) {
  Image<float, 3> a;
  Image<float, 3> b;
  a.CopyTo(b);
  EXPECT_EQ(b.Width(), 0u);
  EXPECT_EQ(b.GetData(), nullptr);
}

TEST(ImageTest, SizeOverflowThrows) {
  constexpr unsigned kMax = 0xFFFFFFFFu;
  // width * height * 3 does not fit in size_t: rejected before any allocation.
  EXPECT_THROW((Image<uint8_t, 3>(kMax, kMax)), std::length_error);
  // width * height fits, the byte count (4 per float) does not: reported by
  // new[] itself, never a silent wrap. (Single-channel uint8 would ask the
  // allocator for ~2^64 bytes, which AddressSanitizer reports as fatal, so
  // float is used for this half of the check.)
  EXPECT_THROW((Image<float, 1>(kMax, kMax)), std::bad_alloc);
}

TEST(ImageTest, MovedFromIsEmpty) {
  Image<uint8_t, 3> a(4, 3);
  Image<uint8_t, 3> b(std::move(a));
  EXPECT_EQ(a.Width(), 0u);
  EXPECT_EQ(a.Height(), 0u);
  EXPECT_EQ(a.Stride(), 0u);
  EXPECT_EQ(a.Offset(), 0u);
  EXPECT_EQ(a.GetData(), nullptr);
  EXPECT_EQ(b.Width(), 4u);
  EXPECT_NE(b.GetData(), nullptr);

  Image<uint8_t, 3> c;
  c = std::move(b);
  EXPECT_EQ(b.Width(), 0u);
  EXPECT_EQ(b.Height(), 0u);
  EXPECT_EQ(b.Stride(), 0u);
  EXPECT_EQ(b.Offset(), 0u);
  EXPECT_EQ(b.GetData(), nullptr);
  EXPECT_EQ(c.Width(), 4u);
  EXPECT_EQ(c.Height(), 3u);
}

TEST(ImageTest, IndexChainMatchesAt) {
  auto parent = MakeParent(8, 6);
  Image<uint8_t, 3> view(parent, 1, 1, 4, 3);
  for (size_t y = 0; y < 3; ++y) {
    for (size_t x = 0; x < 4; ++x) {
      for (size_t c = 0; c < 3; ++c) {
        const uint8_t via_chain = view[y][x][c];
        EXPECT_EQ(via_chain, view.At(y, x, c)) << "y=" << y << " x=" << x << " c=" << c;
        EXPECT_EQ(via_chain, parent.At(y + 1, x + 1, c));
      }
    }
  }
}

TEST(ImageTest, OffsetIsSizeT) {
  static_assert(std::is_same_v<decltype(std::declval<const Image<uint8_t, 3> &>().Offset()),
                               size_t>);
  auto parent = MakeParent(8, 6);
  Image<uint8_t, 3> outer(parent, 2, 1, 5, 4);
  EXPECT_EQ(outer.Offset(), (1u * 8u + 2u) * 3u);
  Image<uint8_t, 3> inner(outer, 1, 2, 2, 2);
  // parent offset + (y * stride + x) * C, the stride being the parent's 8.
  EXPECT_EQ(inner.Offset(), outer.Offset() + (2u * 8u + 1u) * 3u);
  EXPECT_EQ(inner.Offset(), (3u * 8u + 3u) * 3u);
  EXPECT_EQ(inner.At(0, 0, 0), parent.At(3, 3, 0));
}

// Review CR-02: a view outside its parent used to read and write outside the
// parent's buffer in release builds; the constructor throws in every build.
TEST(ImageTest, ViewOutsideParentThrows) {
  auto parent = MakeParent(8, 6);
  EXPECT_THROW((Image<uint8_t, 3>(parent, 5, 0, 4, 1)), std::out_of_range);  // x + w = 9 > 8
  EXPECT_THROW((Image<uint8_t, 3>(parent, 0, 4, 1, 3)), std::out_of_range);  // y + h = 7 > 6
  EXPECT_THROW((Image<uint8_t, 3>(parent, 8, 0, 1, 1)), std::out_of_range);  // starts at the border
  EXPECT_THROW((Image<uint8_t, 3>(parent, 0, 0, 9, 6)), std::out_of_range);  // wider than the parent
  // The sums are computed in size_t, so a huge start cannot wrap around.
  EXPECT_THROW((Image<uint8_t, 3>(parent, 0xFFFFFFFFu, 0, 2, 1)), std::out_of_range);
  EXPECT_THROW((Image<uint8_t, 3>(parent, 0, 0xFFFFFFFFu, 1, 2)), std::out_of_range);

  // The bound is the immediate parent: (4, 0, 2, 1) of the outer view lies
  // inside the parent's buffer (columns 6 and 7) but outside the outer view.
  Image<uint8_t, 3> outer(parent, 2, 1, 5, 4);
  EXPECT_THROW((Image<uint8_t, 3>(outer, 4, 0, 2, 1)), std::out_of_range);
}

TEST(ImageTest, ViewsInsideTheBoundsAreAccepted) {
  auto parent = MakeParent(8, 6);
  EXPECT_NO_THROW((Image<uint8_t, 3>(parent, 5, 4, 3, 2)));  // touches right and bottom borders
  EXPECT_NO_THROW((Image<uint8_t, 3>(parent, 0, 0, 8, 6)));  // the whole parent
  EXPECT_NO_THROW((Image<uint8_t, 3>(parent, 8, 6, 0, 0)));  // empty, on the far corner
  Image<uint8_t, 3> empty_view(parent, 8, 6, 0, 0);
  EXPECT_EQ(empty_view.Width(), 0u);
  EXPECT_EQ(empty_view.Height(), 0u);

  Image<uint8_t, 3> outer(parent, 2, 1, 5, 4);
  EXPECT_NO_THROW((Image<uint8_t, 3>(outer, 3, 2, 2, 2)));  // touches the outer view's borders
}

TEST(ImageTest, ViewOfAnEmptyImage) {
  Image<uint8_t, 3> empty;
  Image<uint8_t, 3> same(empty, 0, 0, 0, 0);
  EXPECT_EQ(same.Width(), 0u);
  EXPECT_EQ(same.Height(), 0u);
  EXPECT_THROW((Image<uint8_t, 3>(empty, 0, 0, 1, 1)), std::out_of_range);
}

// Review CR-02: a CopyTo into a smaller image used to overflow the target's
// buffer in release builds (the size check was an assert).
TEST(ImageTest, CopyToSizeMismatchThrows) {
  constexpr uint8_t kSentinel = 0xEE;
  auto fill = [&](Image<uint8_t, 3> &image) {
    for (unsigned y = 0; y < image.Height(); ++y)
      for (unsigned x = 0; x < image.Width(); ++x)
        for (unsigned c = 0; c < 3; ++c) image.Set(y, x, c, kSentinel);
  };
  auto untouched = [&](const Image<uint8_t, 3> &image) {
    for (unsigned y = 0; y < image.Height(); ++y)
      for (unsigned x = 0; x < image.Width(); ++x)
        for (unsigned c = 0; c < 3; ++c)
          if (image.At(y, x, c) != kSentinel) return false;
    return true;
  };

  auto source = MakeParent(4, 3);  // no value of MakeParent(4, 3) equals kSentinel
  for (const auto &[w, h] : {std::pair{4u, 2u}, std::pair{3u, 3u}, std::pair{5u, 3u}}) {
    Image<uint8_t, 3> target(w, h);
    fill(target);
    EXPECT_THROW(source.CopyTo(target), std::invalid_argument) << w << "x" << h;
    EXPECT_TRUE(untouched(target)) << w << "x" << h;
  }

  // An empty source into a non-empty target is a mismatch too (the early
  // return for empty images must not hide it).
  Image<uint8_t, 3> empty;
  Image<uint8_t, 3> target(2, 2);
  fill(target);
  EXPECT_THROW(empty.CopyTo(target), std::invalid_argument);
  EXPECT_TRUE(untouched(target));
}

template <typename I>
concept CanSet =requires(I &image) { image.Set(0, 0, 0, uint8_t{0}); };

TEST(ImageTest, SetOnConstDoesNotCompile) {
  static_assert(CanSet<Image<uint8_t, 3>>);
  static_assert(!CanSet<const Image<uint8_t, 3>>);
  SUCCEED();
}

}  // namespace
}  // namespace voyahslam::images
