#pragma once

#include <cstddef>
#include <vector>

#include "lib/field.hpp"
#include "lib/operation.hpp"
#include "lib/problem.hpp"
#include "lib/random.hpp"
#include "lib/timer.hpp"

namespace proc36 {

struct BeamStackSearchConfig {
    std::size_t beam_width = 64;
    std::size_t max_depth = 20;
    std::size_t max_nodes = 50'000;
    double time_limit_ms = 4500.0;  // stop a bit before 5 minutes for safety
    double match_weight = 10.0;
    double unmatched_penalty = 8.0;
    double depth_penalty = 0.05;
    double operation_penalty = 0.1;
    std::size_t max_children_per_node = 32;
    std::vector<std::size_t> rotation_sizes = {2, 3, 4};
    bool use_global_hash = true;
};

struct BeamStackSearchResult {
    std::vector<Operation> operations;
    PairStatus status{};
    bool solved = false;
    std::size_t explored_nodes = 0;
    double elapsed_ms = 0.0;
};

class BeamStackSearchSolver {
public:
    explicit BeamStackSearchSolver(BeamStackSearchConfig config = {});

    [[nodiscard]] BeamStackSearchResult solve(const Problem& problem);

private:
    struct Node {
        Field field;
        std::vector<Operation> operations;
        PairStatus status{};
        double score = 0.0;
        std::size_t depth = 0;
    };

    [[nodiscard]] double evaluate(const Node& node) const;
    [[nodiscard]] std::vector<Operation> generate_operations(const Field& field, const std::vector<Operation>& history) const;
    void update_best(const Node& node, BeamStackSearchResult& best_result, double& best_score) const;

    BeamStackSearchConfig config_;
    mutable Random random_;
};

}  // namespace proc36
