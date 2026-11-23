#include "coretech/vision/engine/camera.h"
#include "coretech/vision/engine/imageBuffer/imageBuffer.h"
#include "coretech/vision/engine/markerDetector.h"
#include <cstdio>
#include <string>
#include <vector>

using namespace Anki;
using namespace Anki::Vision;

static bool decode(const char* filename, const Image& image) {
  Camera camera(0);
  MarkerDetector detector(camera);
  std::list<ObservedMarker> markers;
  if (detector.Init(image.GetNumRows(), image.GetNumCols()) != RESULT_OK ||
      detector.Detect(image, markers) != RESULT_OK) return false;
  std::printf("%s %dx%d %zu", filename, image.GetNumCols(), image.GetNumRows(), markers.size());
  for (const auto& marker : markers) std::printf(" %s", marker.GetCodeName());
  std::puts("");
  return true;
}

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    Image image;
    if (std::string(argv[i]).find(".nv12") == std::string::npos) {
      if (image.Load(argv[i]) != RESULT_OK || !decode(argv[i], image)) return 1;
      continue;
    }
    constexpr int width = 1280, height = 720;
    std::vector<u8> bytes(width * height * 3 / 2);
    auto file = std::fopen(argv[i], "rb");
    if (!file) return 2;
    const bool complete = std::fread(bytes.data(), 1, bytes.size(), file) == bytes.size() && std::fgetc(file) == EOF;
    std::fclose(file);
    if (!complete) return 2;
    ImageBuffer buffer(bytes.data(), height * 3 / 2, width, ImageEncoding::YUV420sp, 1, 1);
    buffer.SetSensorResolution(height, width);
    for (auto size : {ImageCacheSize::Half, ImageCacheSize::Quarter}) {
      if (!buffer.GetGray(image, size) || !decode(argv[i], image)) return 3;
    }
  }
  return 0;
}
