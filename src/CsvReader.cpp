
#include "vio/CsvReader.h"
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace vio {
bool CsvReader::getNextObservation() {
  std::string tempStr;
  if (!stream_.eof()) {
    getline(stream_, tempStr);
    if (stream_.fail()) {
      return false;
    }
    std::stringstream line_str(tempStr);
    line_str >> (*pattern_);
    return true;
  } else {
    return false;
  }
}

CsvReader::CsvReader(const std::string file,
                     std::shared_ptr<LinePattern> pattern, int headerLines)
    : stream_(file.c_str()), pattern_(pattern) {
  if (!stream_.is_open()) {
    std::cerr << "CsvReader failed to open file:" << file << ".\n";
  } else {
    for (int i = 0; i < headerLines; ++i) {
      std::string tempStr;
      getline(stream_, tempStr);
    }
  }
}

CsvReader::~CsvReader() {}

}  // namespace vio
