#include "../include/mmWaveRadar_imp.h"

data_header_t dataHeader;
data_tl_t dataTL;
detected_object_t detectedObject;
std::vector<detected_object_t> detectedObjects;
data_complete_t dataComplete;

std::mutex mtx;

float PackRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t color_uint = ((uint32_t)r << 16 | (uint32_t) g << 8 | (uint32_t)b);
  return *reinterpret_cast<float*>(&color_uint);
}
