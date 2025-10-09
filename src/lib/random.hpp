#pragma once

#include <chrono>
#include <cstdint>
#include <random>

namespace proc36 {

class Random {
public:
    using engine_type = std::mt19937_64;

    Random() : engine_(seed()) {}

    explicit Random(std::uint64_t s) : engine_(s) {}

    template <class Int>
    [[nodiscard]] Int next_int(Int l, Int r) {
        std::uniform_int_distribution<Int> dist(l, r);
        return dist(engine_);
    }

    template <class Real>
    [[nodiscard]] Real next_real(Real l, Real r) {
        std::uniform_real_distribution<Real> dist(l, r);
        return dist(engine_);
    }

    [[nodiscard]] engine_type &engine() noexcept { return engine_; }

private:
    static std::uint64_t seed() {
        return static_cast<std::uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    }

    engine_type engine_;
};

}  // namespace proc36
