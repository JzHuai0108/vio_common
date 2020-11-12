
#ifndef CSVREADER_H
#define CSVREADER_H

#include <fstream>
#include <memory>
#include <sys/stat.h>

namespace vio {
class LinePattern {
  virtual std::ostream &print(std::ostream &) const = 0;
  virtual std::istream &read(std::istream &) = 0;

public:
  friend std::ostream &operator<<(std::ostream &os, const LinePattern &rhs) {
    return rhs.print(os);
  }
  friend std::istream &operator>>(std::istream &is, LinePattern &rhs) {
    return rhs.read(is);
  }
  virtual ~LinePattern() {}

  virtual double timestamp() const { return 0; };
};

class CsvReader {
public:
  CsvReader(const std::string file, std::shared_ptr<LinePattern> pattern,
            int headerLines = 1);

  virtual ~CsvReader();

  bool getNextObservation();

  std::shared_ptr<const LinePattern> currentRow() { return pattern_; }

private:
  std::ifstream stream_;
  std::shared_ptr<LinePattern> pattern_;
};

// https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
inline bool fileExists(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

}  // namespace vio

#endif  // CSVREADER_H
