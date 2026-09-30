#include <vector>
#include <fstream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "voyahslam/io/read_image.h"

namespace voyahslam::io {

namespace {

std::vector<uint8_t> ReadData(std::istream &stream) {
  stream.seekg(0, std::ios::end);
  std::streamsize size = stream.tellg();
  stream.seekg(0, std::ios::beg);
  std::vector<uint8_t> buffer(size);
  stream.read((char *)buffer.data(), size);
  return buffer;
}

} // namespace

voyahslam::images::TImageU8Gray ReadImageFromStream(std::istream &file) {
  auto data = ReadData(file);
  int width, height, channels;
  auto decompressed = stbi_load_from_memory(data.data(), data.size(), &width, &height, &channels, 1);
  images::TImageU8Gray result(width, height);
  memcpy(result.GetData(), decompressed, width * height);
  stbi_image_free(decompressed);
  return result;
}

voyahslam::images::TImageF32Gray ReadImageFromStream32F(std::istream &file) {
  auto data = ReadData(file);
  int width, height, channels;
  auto decompressed = stbi_loadf_from_memory(data.data(), data.size(), &width, &height, &channels, 1);
  images::TImageF32Gray result(width, height);
  memcpy(result.GetData(), decompressed, width * height * sizeof(float));
  stbi_image_free(decompressed);
  return result;
}

} // namespace voyahslam::io
