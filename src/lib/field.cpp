#include "lib/field.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace proc36 {

namespace {
[[nodiscard]] std::uint64_t splitmix64(std::uint64_t x) {
    x += 0x9e3779b97f4a7c15ull;
    x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ull;
    x = (x ^ (x >> 27U)) * 0x94d049bb133111ebull;
    x = x ^ (x >> 31U);
    return x;
}
}  // namespace

Field::Field(std::size_t size, std::vector<int> cells)
    : size_(size), cells_(std::move(cells)) {
    if (size_ == 0) {
        throw std::invalid_argument("Field size must be positive");
    }
    if (cells_.size() != size_ * size_) {
        throw std::invalid_argument("Field cells size mismatch");
    }
}

int Field::at(std::size_t x, std::size_t y) const {
    if (!in_bounds(x, y)) {
        throw std::out_of_range("Field::at: position out of bounds");
    }
    return cells_[y * size_ + x];
}

void Field::set(std::size_t x, std::size_t y, int value) {
    if (!in_bounds(x, y)) {
        throw std::out_of_range("Field::set: position out of bounds");
    }
    cells_[y * size_ + x] = value;
}

bool Field::in_bounds(std::size_t x, std::size_t y) const noexcept {
    return x < size_ && y < size_;
}

bool Field::is_valid_operation(const Operation& op) const noexcept {
    return op.is_valid(size_);
}

void Field::apply(const Operation& op) {
    if (!is_valid_operation(op)) {
        throw std::invalid_argument("Invalid rotation operation");
    }
    const auto k = op.size;
    std::vector<int> original(k * k);
    for (std::size_t dy = 0; dy < k; ++dy) {
        for (std::size_t dx = 0; dx < k; ++dx) {
            original[dy * k + dx] = at(op.x + dx, op.y + dy);
        }
    }

    for (std::size_t dy = 0; dy < k; ++dy) {
        for (std::size_t dx = 0; dx < k; ++dx) {
            const std::size_t src_row = k - 1 - dx;
            const std::size_t src_col = dy;
            const auto value = original[src_row * k + src_col];
            set(op.x + dx, op.y + dy, value);
        }
    }
}

Field Field::applied(const Operation& op) const {
    Field next = *this;
    next.apply(op);
    return next;
}

std::vector<Position> Field::positions_of(int value) const {
    std::vector<Position> res;
    res.reserve(2);
    for (std::size_t y = 0; y < size_; ++y) {
        for (std::size_t x = 0; x < size_; ++x) {
            if (cells_[y * size_ + x] == value) {
                res.push_back(Position{x, y});
            }
        }
    }
    return res;
}

PairStatus Field::evaluate_pairs() const {
    std::unordered_map<int, Position> first_occurrence;
    first_occurrence.reserve(size_ * size_ / 2);
    std::size_t matched = 0;
    std::size_t unmatched = 0;

    for (std::size_t y = 0; y < size_; ++y) {
        for (std::size_t x = 0; x < size_; ++x) {
            const auto value = cells_[y * size_ + x];
            auto it = first_occurrence.find(value);
            if (it == first_occurrence.end()) {
                first_occurrence.emplace(value, Position{x, y});
                continue;
            }
            const auto& first = it->second;
            const auto dist = static_cast<int>(std::abs(static_cast<int>(first.x) - static_cast<int>(x))) +
                              static_cast<int>(std::abs(static_cast<int>(first.y) - static_cast<int>(y)));
            if (dist == 1) {
                ++matched;
            } else {
                ++unmatched;
            }
        }
    }

    return PairStatus{matched, unmatched};
}

bool Field::is_goal_state() const {
    const auto status = evaluate_pairs();
    return status.unmatched == 0 && status.matched * 2 == size_ * size_;
}

std::uint64_t Field::zobrist_hash() const {
    std::uint64_t hash = 0;
    for (std::size_t idx = 0; idx < cells_.size(); ++idx) {
        const auto mixed = splitmix64(static_cast<std::uint64_t>(cells_[idx]) * 1'000'003ULL + idx);
        hash ^= mixed + 0x9e3779b97f4a7c15ULL + (hash << 6U) + (hash >> 2U);
    }
    return hash;
}

std::string Field::to_string() const {
    std::ostringstream oss;
    for (std::size_t y = 0; y < size_; ++y) {
        for (std::size_t x = 0; x < size_; ++x) {
            if (x) {
                oss << ' ';
            }
            oss << at(x, y);
        }
        if (y + 1 != size_) {
            oss << '\n';
        }
    }
    return oss.str();
}

}  // namespace proc36
