#include "solver/beam_stack_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

namespace proc36 {

BeamStackSearchSolver::BeamStackSearchSolver(BeamStackSearchConfig config)
    : config_(std::move(config)) {}

std::vector<Operation> BeamStackSearchSolver::generate_operations(const Field& field,
                                                                  const std::vector<Operation>& history,
                                                                  const PairMetrics& metrics) const {
    const auto board_size = field.size();
    std::vector<Operation> operations;
    operations.reserve(board_size * board_size);

    struct Candidate {
        Operation op;
        std::size_t impact;
    };
    std::vector<Candidate> candidates;

    const Operation* last_op = history.empty() ? nullptr : &history.back();
    const bool use_mask = metrics.status.unmatched > 0 && metrics.unmatched_mask.size() == field.cell_count();
    std::vector<std::size_t> prefix;

    auto area_sum = [&](std::size_t x0, std::size_t y0, std::size_t k) -> std::size_t {
        if (!use_mask) {
            return 1;  // treat as impactful to avoid pruning everything
        }
        const std::size_t stride = board_size + 1;
        const std::size_t x1 = x0 + k;
        const std::size_t y1 = y0 + k;
        return prefix[y1 * stride + x1] - prefix[y0 * stride + x1] - prefix[y1 * stride + x0] + prefix[y0 * stride + x0];
    };

    if (use_mask) {
        prefix.assign((board_size + 1) * (board_size + 1), 0);
        const std::size_t stride = board_size + 1;
        for (std::size_t y = 0; y < board_size; ++y) {
            for (std::size_t x = 0; x < board_size; ++x) {
                const auto value = static_cast<std::size_t>(metrics.unmatched_mask[y * board_size + x]);
                prefix[(y + 1) * stride + (x + 1)] = value + prefix[y * stride + (x + 1)] +
                                                     prefix[(y + 1) * stride + x] - prefix[y * stride + x];
            }
        }
        candidates.reserve(board_size * board_size);
    }

    for (auto size : config_.rotation_sizes) {
        if (size < 2 || size > board_size) {
            continue;
        }
        for (std::size_t y = 0; y + size <= board_size; ++y) {
            for (std::size_t x = 0; x + size <= board_size; ++x) {
                Operation op{x, y, size};
                if (!field.is_valid_operation(op)) {
                    continue;
                }
                if (last_op != nullptr && last_op->x == op.x && last_op->y == op.y && last_op->size == op.size) {
                    continue;  // avoid immediately re-applying the same rotation
                }
                const auto impact = area_sum(x, y, size);
                if (use_mask && impact == 0) {
                    continue;  // skip operations that don't touch any unmatched cells
                }
                if (use_mask) {
                    candidates.push_back(Candidate{op, impact});
                } else {
                    operations.push_back(op);
                }
            }
        }
    }

    if (use_mask) {
        std::stable_sort(candidates.begin(), candidates.end(),
                         [](const Candidate& a, const Candidate& b) { return a.impact > b.impact; });
        operations.reserve(candidates.size());
        for (const auto& candidate : candidates) {
            operations.push_back(candidate.op);
        }
    }

    return operations;
}

double BeamStackSearchSolver::evaluate(const Node& node) const {
    const auto& status = node.metrics.status;
    const double matched_score = config_.match_weight * static_cast<double>(status.matched);
    const double unmatched_penalty = config_.unmatched_penalty * static_cast<double>(status.unmatched);
    const double total_distance_penalty =
        config_.total_distance_penalty * static_cast<double>(node.metrics.total_unmatched_distance);
    const double max_distance_penalty =
        config_.max_distance_penalty * static_cast<double>(node.metrics.max_unmatched_distance);
    const double depth_penalty = config_.depth_penalty * static_cast<double>(node.depth);
    const double op_penalty = config_.operation_penalty * static_cast<double>(node.operations.size());
    const double jitter = random_.next_real(0.0, 1.0) * 1e-3;
    double score = matched_score - unmatched_penalty - total_distance_penalty - max_distance_penalty - depth_penalty -
                   op_penalty + jitter;
    if (status.unmatched == 0) {
        score += 1e6;  // strongly prefer solved states
    }
    return score;
}

void BeamStackSearchSolver::update_best(const Node& node, BeamStackSearchResult& best_result, double& best_score) const {
    const double score = node.score;
    if (score > best_score) {
        best_score = score;
        best_result.operations = node.operations;
        best_result.status = node.metrics.status;
        best_result.solved = node.metrics.status.unmatched == 0;
    }
}

BeamStackSearchSolver::SearchLimits BeamStackSearchSolver::derive_limits(std::size_t board_size) const {
    SearchLimits limits{};

    const double normalized = std::max(1.0, static_cast<double>(board_size) / 8.0);
    const double size_scale = config_.adaptive_limits ? std::pow(normalized, 1.2) : 1.0;
    const double depth_scale = config_.adaptive_limits ? std::pow(normalized, 1.1) : 1.0;
    const double node_scale = config_.adaptive_limits ? std::pow(normalized, 2.6) : 1.0;
    const double child_scale = config_.adaptive_limits ? std::pow(normalized, 0.9) : 1.0;

    auto scale_up = [](std::size_t base, double factor) -> std::size_t {
        if (base == 0) {
            return 0;
        }
        const double scaled = std::ceil(static_cast<double>(base) * factor);
        const double clamped = std::min(scaled, static_cast<double>(std::numeric_limits<std::size_t>::max()));
        return static_cast<std::size_t>(std::max(static_cast<double>(base), clamped));
    };

    limits.beam_width = scale_up(config_.beam_width, size_scale);
    if (config_.beam_width_cap > 0) {
        limits.beam_width = std::min(limits.beam_width, config_.beam_width_cap);
    }
    if (limits.beam_width == 0) {
        limits.beam_width = 1;
    }

    limits.max_depth = scale_up(config_.max_depth, depth_scale);
    limits.max_nodes = scale_up(config_.max_nodes, node_scale);
    limits.max_children_per_node = config_.max_children_per_node > 0 ?
                                       scale_up(config_.max_children_per_node, child_scale) :
                                       0;

    if (config_.adaptive_limits && board_size <= 8) {
        limits.max_depth = std::max<std::size_t>(limits.max_depth, 40);
        limits.max_nodes = std::max<std::size_t>(limits.max_nodes, 250'000);
        limits.max_children_per_node = std::max<std::size_t>(limits.max_children_per_node, 56);
    }

    return limits;
}

BeamStackSearchSolver::IterationOutcome BeamStackSearchSolver::run_search_iteration(const Node& root,
                                                                                   const SearchLimits& limits,
                                                                                   Timer& timer,
                                                                                   BeamStackSearchResult& result,
                                                                                   double& best_score) const {
    IterationOutcome outcome;

    if (root.metrics.status.unmatched > 0) {
        outcome.best_unsolved = root;
        outcome.has_best_unsolved = true;
    }

    std::vector<Node> current_layer;
    current_layer.reserve(std::max<std::size_t>(1, limits.beam_width));
    current_layer.push_back(root);

    std::unordered_set<std::uint64_t> visited;
    if (config_.use_global_hash) {
        visited.insert(root.field.zobrist_hash());
    }

    const bool enforce_node_limit = limits.max_nodes > 0;
    bool reached_limit = false;

    for (std::size_t relative_depth = 0; relative_depth < limits.max_depth && !current_layer.empty(); ++relative_depth) {
        if (config_.time_limit_ms > 0.0 && timer.elapsed_ms() > config_.time_limit_ms) {
            outcome.reached_limit = true;
            break;
        }

        std::vector<Node> next_layer;
        next_layer.reserve(limits.beam_width * 2 + 1);

        for (const auto& node : current_layer) {
            if (config_.time_limit_ms > 0.0 && timer.elapsed_ms() > config_.time_limit_ms) {
                outcome.reached_limit = true;
                reached_limit = true;
                break;
            }
            if (enforce_node_limit && result.explored_nodes >= limits.max_nodes) {
                outcome.reached_limit = true;
                reached_limit = true;
                break;
            }
            if (limits.max_depth > 0 && node.depth >= limits.max_depth) {
                continue;
            }

            auto candidate_ops = generate_operations(node.field, node.operations, node.metrics);
            if (candidate_ops.empty()) {
                continue;
            }

            std::vector<Node> children;
            children.reserve(candidate_ops.size());

            for (const auto& op : candidate_ops) {
                if (config_.time_limit_ms > 0.0 && timer.elapsed_ms() > config_.time_limit_ms) {
                    outcome.reached_limit = true;
                    reached_limit = true;
                    break;
                }
                if (enforce_node_limit && result.explored_nodes >= limits.max_nodes) {
                    outcome.reached_limit = true;
                    reached_limit = true;
                    break;
                }

                Node child;
                child.field = node.field;
                child.field.apply(op);
                if (config_.use_global_hash) {
                    const auto hash = child.field.zobrist_hash();
                    if (visited.find(hash) != visited.end()) {
                        continue;
                    }
                    visited.insert(hash);
                    const std::size_t visited_cap = enforce_node_limit ? limits.max_nodes * 4 : 0;
                    if (visited_cap > 0 && visited.size() > visited_cap) {
                        visited.clear();
                        visited.insert(hash);
                    }
                }

                child.operations = node.operations;
                child.operations.push_back(op);
                child.depth = node.depth + 1;
                child.metrics = child.field.evaluate_pair_metrics();
                child.score = evaluate(child);

                update_best(child, result, best_score);
                ++result.explored_nodes;

                if (child.metrics.status.unmatched == 0) {
                    outcome.solved = true;
                    children.push_back(std::move(child));
                    goto iteration_finished;
                }

                if (!outcome.has_best_unsolved ||
                    child.metrics.status.unmatched < outcome.best_unsolved.metrics.status.unmatched ||
                    (child.metrics.status.unmatched == outcome.best_unsolved.metrics.status.unmatched &&
                     child.metrics.total_unmatched_distance < outcome.best_unsolved.metrics.total_unmatched_distance)) {
                    outcome.best_unsolved = child;
                    outcome.has_best_unsolved = true;
                }

                children.push_back(std::move(child));
            }

            if (reached_limit) {
                break;
            }

            if (children.empty()) {
                continue;
            }

            if (limits.max_children_per_node > 0 && children.size() > limits.max_children_per_node) {
                std::partial_sort(children.begin(),
                                  children.begin() + static_cast<std::ptrdiff_t>(limits.max_children_per_node), children.end(),
                                  [](const Node& a, const Node& b) { return a.score > b.score; });
                children.resize(limits.max_children_per_node);
            }

            for (auto& child : children) {
                if (child.metrics.status.unmatched == 0) {
                    outcome.solved = true;
                    update_best(child, result, best_score);
                    next_layer.push_back(std::move(child));
                    goto iteration_finished;
                }
                next_layer.push_back(std::move(child));
            }
        }

        if (reached_limit) {
            break;
        }

        if (next_layer.empty()) {
            break;
        }

        if (next_layer.size() > limits.beam_width) {
            std::partial_sort(next_layer.begin(),
                              next_layer.begin() + static_cast<std::ptrdiff_t>(limits.beam_width), next_layer.end(),
                              [](const Node& a, const Node& b) { return a.score > b.score; });
            next_layer.resize(limits.beam_width);
        }

        current_layer = std::move(next_layer);
    }

iteration_finished:
    return outcome;
}

BeamStackSearchResult BeamStackSearchSolver::solve(const Problem& problem) {
    BeamStackSearchResult result;
    Timer timer;

    result.elapsed_ms = 0.0;
    result.explored_nodes = 0;

    SearchLimits base_limits = derive_limits(problem.size);

    Node current_root;
    current_root.field = problem.make_field();
    current_root.metrics = current_root.field.evaluate_pair_metrics();
    current_root.depth = 0;
    current_root.operations.clear();
    current_root.score = evaluate(current_root);

    double best_score = -1e18;
    update_best(current_root, result, best_score);

    const std::size_t max_iterations = config_.adaptive_limits ? 6 : 1;
    std::size_t iteration = 0;

    while ((config_.time_limit_ms <= 0.0 || timer.elapsed_ms() < config_.time_limit_ms) && iteration < max_iterations) {
        SearchLimits iter_limits = base_limits;
        if (iteration > 0) {
            const double widen_factor = 1.0 + 0.45 * static_cast<double>(iteration);
            const double node_factor = 1.0 + 0.6 * static_cast<double>(iteration);
            const std::size_t depth_bonus = static_cast<std::size_t>(6 * iteration);
            const std::size_t child_bonus = static_cast<std::size_t>(std::max<std::size_t>(4, iteration * 4));

            iter_limits.beam_width = static_cast<std::size_t>(std::ceil(iter_limits.beam_width * widen_factor));
            if (config_.beam_width_cap > 0) {
                iter_limits.beam_width = std::min(iter_limits.beam_width, config_.beam_width_cap);
            }
            if (iter_limits.max_nodes > 0) {
                iter_limits.max_nodes = static_cast<std::size_t>(std::min<double>(
                    std::numeric_limits<std::size_t>::max(),
                    std::ceil(static_cast<double>(iter_limits.max_nodes) * node_factor)));
            }
            if (iter_limits.max_depth > 0) {
                iter_limits.max_depth += depth_bonus;
            }
            if (iter_limits.max_children_per_node > 0) {
                iter_limits.max_children_per_node += child_bonus;
            }
        }

        update_best(current_root, result, best_score);
        auto outcome = run_search_iteration(current_root, iter_limits, timer, result, best_score);

        if (result.solved || outcome.solved) {
            break;
        }

        if (!outcome.has_best_unsolved) {
            break;
        }

        const auto current_unmatched = current_root.metrics.status.unmatched;
        const auto current_distance = current_root.metrics.total_unmatched_distance;
        const auto next_unmatched = outcome.best_unsolved.metrics.status.unmatched;
        const auto next_distance = outcome.best_unsolved.metrics.total_unmatched_distance;

        if (next_unmatched > current_unmatched) {
            break;
        }

        if (next_unmatched == current_unmatched && next_distance >= current_distance) {
            break;
        }

        current_root = outcome.best_unsolved;
        current_root.score = evaluate(current_root);
        base_limits = iter_limits;
        ++iteration;
    }

    result.elapsed_ms = timer.elapsed_ms();
    return result;
}

}  // namespace proc36
