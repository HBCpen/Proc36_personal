#include "lib/field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

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
    return evaluate_pair_metrics().status;
}

PairMetrics Field::evaluate_pair_metrics() const {
    PairMetrics metrics;
    metrics.unmatched_mask.assign(cells_.size(), 0);

    const auto initial_pairs = cells_.size() / 2;
    const Position sentinel{size_, size_};
    std::vector<Position> first_positions(initial_pairs, sentinel);
    std::vector<std::size_t> first_indices(initial_pairs, std::numeric_limits<std::size_t>::max());

    auto ensure_capacity = [&](std::size_t value) {
        if (value >= first_positions.size()) {
            const auto new_size = value + 1;
            first_positions.resize(new_size, sentinel);
            first_indices.resize(new_size, std::numeric_limits<std::size_t>::max());
        }
    };

    for (std::size_t idx = 0; idx < cells_.size(); ++idx) {
        const auto value = cells_[idx];
        if (value < 0) {
            continue;  // ignore invalid negatives defensively
        }
        const auto uvalue = static_cast<std::size_t>(value);
        ensure_capacity(uvalue);

        if (first_indices[uvalue] == std::numeric_limits<std::size_t>::max()) {
            const auto x = idx % size_;
            const auto y = idx / size_;
            first_positions[uvalue] = Position{x, y};
            first_indices[uvalue] = idx;
            continue;
        }

        const auto first_index = first_indices[uvalue];
        const auto first_pos = first_positions[uvalue];
        const auto x = idx % size_;
        const auto y = idx / size_;
        const auto distance = static_cast<std::size_t>(
            std::abs(static_cast<int>(first_pos.x) - static_cast<int>(x)) +
            std::abs(static_cast<int>(first_pos.y) - static_cast<int>(y)));

        if (distance == 1) {
            ++metrics.status.matched;
        } else {
            ++metrics.status.unmatched;
            metrics.total_unmatched_distance += distance;
            metrics.max_unmatched_distance = std::max(metrics.max_unmatched_distance, distance);
            metrics.unmatched_mask[first_index] = 1;
            metrics.unmatched_mask[idx] = 1;
        }
    }

    return metrics;
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
