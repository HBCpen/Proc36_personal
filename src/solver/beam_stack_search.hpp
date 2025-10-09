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
    std::size_t beam_width = 160;
    std::size_t max_depth = 64;
    std::size_t max_nodes = 350'000;
    double time_limit_ms = 9800.0;  // allow longer exploration while guarding against 10s limit
    double match_weight = 11.0;
    double unmatched_penalty = 13.0;
    double depth_penalty = 0.025;
    double operation_penalty = 0.05;
    double total_distance_penalty = 0.26;
    double max_distance_penalty = 0.075;
    std::size_t max_children_per_node = 80;
    std::vector<std::size_t> rotation_sizes = {2, 3, 4, 5, 6, 7, 8, 10, 12};
    bool use_global_hash = true;
    bool adaptive_limits = true;
    std::size_t beam_width_cap = 4096;
    std::size_t max_iterations = 11;
    std::size_t refinement_attempts = 320;
    std::size_t refinement_sample = 160;
    double refinement_time_budget_ms = 1500.0;
    std::size_t shake_attempts = 4;
    std::size_t shake_max_length = 10;
    double shake_time_ratio = 0.85;  // only shake while within 85% of time budget
    double shake_accept_equal_probability = 0.2;
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
        PairMetrics metrics;
        double score = 0.0;
        std::size_t depth = 0;
    };

    struct SearchLimits {
        std::size_t beam_width{};
        std::size_t max_depth{};
        std::size_t max_nodes{};
        std::size_t max_children_per_node{};
    };

    struct IterationOutcome {
        bool solved = false;
        bool reached_limit = false;
        Node best_unsolved;
        bool has_best_unsolved = false;
    };

    [[nodiscard]] double evaluate(const Node& node) const;
    [[nodiscard]] std::vector<Operation> generate_operations(const Field& field, const std::vector<Operation>& history,
                                                             const PairMetrics& metrics) const;
    void update_best(const Node& node, BeamStackSearchResult& best_result, double& best_score) const;
    [[nodiscard]] SearchLimits derive_limits(std::size_t board_size) const;
    IterationOutcome run_search_iteration(const Node& root, const SearchLimits& limits, Timer& timer,
                                          BeamStackSearchResult& result, double& best_score) const;
    bool greedy_refinement(const Problem& problem, BeamStackSearchResult& result, Timer& timer, double& best_score) const;
    bool apply_shake(Node& node, BeamStackSearchResult& result, Timer& timer, double& best_score) const;

    BeamStackSearchConfig config_;
    mutable Random random_;
};

}  // namespace proc36
