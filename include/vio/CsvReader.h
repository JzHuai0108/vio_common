
#ifndef CSVREADER_H
#define CSVREADER_H

#include <fstream>
#include <memory>
#include <sys/stat.h>

namespace vio {
class LinePattern {
  virtual std::ostream &print(std::ostream &) const = 0;
  virtual std::istream &read(std::istream &) = 0;
protected:
  char delim_; // delimiter used in the line.
public:
  LinePattern(char delim = ' ') : delim_(delim) {}
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

// https://stackoverflow.com/questions/18100097/portable-way-to-check-if-directory-exists-windows-linux-c
inline bool validDir(const std::string &pathname) {
  struct stat info;
  if (stat(pathname.c_str(), &info) != 0) {
    return false;
  } else if (info.st_mode & S_IFDIR) { // S_ISDIR() doesn't exist on my windows
    return true;
  } else {
    return false;
  }
}

inline bool validFile(const std::string &pathname) {
  struct stat info;
  if (stat(pathname.c_str(), &info) != 0) {
    return false;
  } else if (info.st_mode & S_IFDIR) { // S_ISDIR() doesn't exist on my windows
    return false;
  } else {
    return true;
  }
}

inline bool endswith(std::string const &fullString, std::string const &ending) {
  if (fullString.length() >= ending.length()) {
    return (0 == fullString.compare(fullString.length() - ending.length(),
                                    ending.length(), ending));
  } else {
    return false;
  }
}

/**
 * @brief dirname
 * @param str
 * @return directory name without trailing slashes.
 */
inline std::string dirname(const std::string &str) {
  size_t found;
  found = str.find_last_of("/\\");
  if (found == std::string::npos) {
    return std::string();
  } else {
    return str.substr(0, found);
  }
}

/**
 * @brief filename
 * @param str
 * @return filename including extension.
 */
inline std::string filename(const std::string &str) {
  size_t found;
  found = str.find_last_of("/\\");
  if (found == std::string::npos) {
    return str;
  } else {
    return str.substr(found + 1);
  }
}

/**
 * @brief basename
 * @param str
 * @return basename without extension.
 */
inline std::string basename(const std::string &str) {
  size_t found;
  found = str.find_last_of("/\\");
  size_t found2 = str.find_last_of(".");
  if (found == std::string::npos) {
    if (found2 == std::string::npos) {
      return str;
    } else {
      return str.substr(0, found2);
    }
  } else {
    if (found2 == std::string::npos) {
      return str.substr(found + 1);
    } else {
      return str.substr(found + 1, found2 - found - 1);
    }
  }
}

}  // namespace vio

#endif  // CSVREADER_H
