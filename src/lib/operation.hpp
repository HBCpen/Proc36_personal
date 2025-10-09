#pragma once

#include <cstddef>
#include <string>

namespace proc36 {

struct Operation {
    std::size_t x{};
    std::size_t y{};
    std::size_t size{};

    [[nodiscard]] bool is_valid(std::size_t field_size) const noexcept {
        if (size < 2 || size > field_size) {
            return false;
        }
        if (x + size > field_size || y + size > field_size) {
            return false;
        }
        return true;
    }

    [[nodiscard]] std::string to_string() const {
        return "{" + std::string("\"x\":") + std::to_string(x) +
               ",\"y\":" + std::to_string(y) +
               ",\"n\":" + std::to_string(size) + "}";
    }
};

}  // namespace proc36
