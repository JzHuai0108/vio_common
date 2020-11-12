
#ifndef CSVREADER_H
#define CSVREADER_H

#include <Eigen/Core>
#include <fstream>
#include <memory>
#include <vector>

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
}  // namespace vio

#endif  // CSVREADER_H
