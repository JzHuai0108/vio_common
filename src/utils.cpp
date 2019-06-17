#include "vio/utils.h"
#include <cmath>

#include <algorithm>  // transform
#include <fstream>
#include <iostream>
#include <sstream>

namespace vio {
bool to_bool(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  std::istringstream is(str);
  bool b;
  is >> std::boolalpha >> b;
  return b;
}

/// Round up to next higher power of 2 (return x if it's already a power
/// of 2).
int pow2roundup(int x) {
  if (x < 0) return 0;
  --x;
  x |= x >> 1;
  x |= x >> 2;
  x |= x >> 4;
  x |= x >> 8;
  x |= x >> 16;
  return x + 1;
}

// maxEdge maximum edge length in pixel units
int getDownScale(int w, int h, int maxEdge) {
  float ds = float(std::max(w, h)) / maxEdge;
  return pow2roundup((int)std::ceil(ds));
}

bool isHeaderLine(const std::string& line) {
  int index = 0;
  for (auto it = line.begin(); it < line.end(); ++it, ++index) {
    if (*it == ' ' || *it == ',') break;
  }
  const char* p = line.substr(0, index).c_str();

  char* end;
  double val = std::strtod(p, &end);
  if (val == 0 && end == p) {  // no conversion
    return true;
  } else if (val == HUGE_VAL || val == HUGE_VALF || val == HUGE_VALL) {
    return true;
  } else {
    if (end != p + index)
      return true;
    else
      return false;
  }
}

/**
 * @brief countHeaderLines count number of header lines in an ASCII file,
 *     a header line is identified as beginning with a non digit
 * @param filename
 * @return number of header lines
 */
int countHeaderLines(const std::string& filename) {
  std::ifstream stream(filename);
  if (!stream.is_open()) {
    std::cerr << "Unable to open " << filename << std::endl;
    return -1;
  }
  int count = countHeaderLines(&stream);
  stream.close();
  return count;
}

int countHeaderLines(std::istream* stream) {
  int count = 0;
  std::string line;
  while (std::getline(*stream, line)) {
    if (isHeaderLine(line)) {
      ++count;
    } else {
      break;
    }
  }
  return count;
}

double nanoIntToSecDouble(const int64_t timeNanos) {
  int64_t scale = 1000000000;
  int32_t integral = (int)(timeNanos / scale);
  int32_t fraction = (int)(timeNanos % scale);
  return integral + fraction / (double)scale;
}

/**
 * @brief isVideoFile
 * @param datasetPath
 * @return true if the datasetPath is an video file, false otherwise
 */
bool isVideoFile(const std::string& datasetPath) {
  // datasetPath contains .mp4, .MP4
  size_t len = datasetPath.length();
  if (len < 5) {
    return false;
  }
  if (datasetPath[len - 4] == '.' || datasetPath[len - 5] == '.') {
    return true;
  } else {
    return false;
  }
}

bool isTimeInNanos(double dn) {
  double integral;
  double fractional = std::modf(dn, &integral);
  return fractional < 1e-9;
}

}  // namespace vio
