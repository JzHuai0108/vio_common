#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>

namespace vio {
bool to_bool(std::string str);
/// Round up to next higher power of 2 (return x if it's already a power
/// of 2).
int pow2roundup(int x);
int getDownScale(int w, int h, int maxEdge = 2000);
bool isHeaderLine(const std::string& line);
int countHeaderLines(const std::string& filename);

/**
 * @brief countHeaderLines
 * @param stream note stream will be modified within the function
 * @return
 */
int countHeaderLines(std::istream* stream);

double nanoIntToSecDouble(const int64_t timeNanos);

bool isVideoFile(const std::string& datasetPath);

bool isTimeInNanos(double dn);

bool pathExist(const std::string& name);

bool dirExist(const std::string& name);

void displayArgs(const int argc, char** argv);

void locateSubstring(const std::string& test_str,
                     const std::vector<std::string>& subs,
                     std::vector<bool>* found);

}  // namespace vio
#endif
