#ifndef VIO_TINYTIMER_H
#define VIO_TINYTIMER_H

#include <chrono>

namespace vio {
    struct DummyTimer {
      void tic() {}
      long toc() { return 0; }
    };

    struct TinyTimer {
      TinyTimer() { tic(); }
      void tic() { tstart = std::chrono::high_resolution_clock::now(); }
      long toc() {
        auto tfinish = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(tfinish - tstart)
                .count();
      }
      std::chrono::high_resolution_clock::time_point tstart;
    };
} // namespace vio

#endif // VIO_TINYTIMER_H
