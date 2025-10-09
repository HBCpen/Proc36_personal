#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "lib/operation.hpp"

namespace proc36 {

struct Position {
    std::size_t x{};
    std::size_t y{};

    [[nodiscard]] bool operator==(const Position&) const = default;
};

struct PairStatus {
    std::size_t matched{};
    std::size_t unmatched{};
};

class Field {
public:
    Field() = default;
    Field(std::size_t size, std::vector<int> cells);

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] std::size_t cell_count() const noexcept { return cells_.size(); }

    [[nodiscard]] int at(std::size_t x, std::size_t y) const;
    void set(std::size_t x, std::size_t y, int value);

    [[nodiscard]] bool in_bounds(std::size_t x, std::size_t y) const noexcept;
    [[nodiscard]] bool is_valid_operation(const Operation& op) const noexcept;

    void apply(const Operation& op);
    Field applied(const Operation& op) const;

    [[nodiscard]] std::vector<Position> positions_of(int value) const;
    [[nodiscard]] PairStatus evaluate_pairs() const;

    [[nodiscard]] bool is_goal_state() const;

    [[nodiscard]] std::uint64_t zobrist_hash() const;

    [[nodiscard]] std::string to_string() const;

private:
    std::size_t size_{};
    std::vector<int> cells_;
};

}  // namespace proc36
