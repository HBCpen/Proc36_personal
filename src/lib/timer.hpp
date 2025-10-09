#pragma once

#include <chrono>

namespace proc36 {

class Timer {
public:
    using clock = std::chrono::steady_clock;

    Timer() : start_(clock::now()) {}

    void reset() noexcept { start_ = clock::now(); }

    [[nodiscard]] double elapsed_ms() const noexcept {
        return std::chrono::duration<double, std::milli>(clock::now() - start_).count();
    }

    [[nodiscard]] double elapsed_sec() const noexcept {
        return std::chrono::duration<double>(clock::now() - start_).count();
    }

private:
    clock::time_point start_;
};

}  // namespace proc36
