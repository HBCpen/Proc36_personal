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
    const double size_scale = config_.adaptive_limits ? std::pow(normalized, 1.35) : 1.0;
    const double depth_scale = config_.adaptive_limits ? std::pow(normalized, 1.25) : 1.0;
    const double node_scale = config_.adaptive_limits ? std::pow(normalized, 3.0) : 1.0;
    const double child_scale = config_.adaptive_limits ? std::pow(normalized, 1.1) : 1.0;

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
        limits.max_depth = std::max<std::size_t>(limits.max_depth, 48);
        limits.max_nodes = std::max<std::size_t>(limits.max_nodes, 280'000);
        limits.max_children_per_node = std::max<std::size_t>(limits.max_children_per_node, 64);
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

            if (!children.empty()) {
                std::size_t node_child_limit = limits.max_children_per_node;
                if (node_child_limit > 0 && children.size() > node_child_limit) {
                    const auto unmatched = node.metrics.status.unmatched;
                    const std::size_t adaptive_bonus = unmatched * 2 + std::max<std::size_t>(1, limits.beam_width / 8);
                    const std::size_t max_cap = limits.beam_width > 0 ? (limits.beam_width * 3) / 2 + 32 : children.size();
                    node_child_limit = std::min<std::size_t>({children.size(), node_child_limit + adaptive_bonus, max_cap});

                    std::partial_sort(children.begin(),
                                      children.begin() + static_cast<std::ptrdiff_t>(node_child_limit), children.end(),
                                      [](const Node& a, const Node& b) { return a.score > b.score; });
                    children.resize(node_child_limit);
                }
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

bool BeamStackSearchSolver::apply_shake(Node& node, BeamStackSearchResult& result, Timer& timer,
                                        double& best_score) const {
    if (config_.shake_attempts == 0 || config_.shake_max_length == 0) {
        return false;
    }
    if (config_.time_limit_ms > 0.0 &&
        timer.elapsed_ms() > config_.time_limit_ms * config_.shake_time_ratio) {
        return false;
    }

    Node candidate = node;
    const auto original_unmatched = candidate.metrics.status.unmatched;
    const auto original_distance = candidate.metrics.total_unmatched_distance + candidate.metrics.max_unmatched_distance;

    const std::size_t steps = static_cast<std::size_t>(
        random_.next_int<std::size_t>(1, std::max<std::size_t>(1, config_.shake_max_length)));
    std::size_t applied = 0;

    for (; applied < steps; ++applied) {
        if (config_.time_limit_ms > 0.0 && timer.elapsed_ms() > config_.time_limit_ms * config_.shake_time_ratio) {
            break;
        }

        auto candidate_ops = generate_operations(candidate.field, candidate.operations, candidate.metrics);
        if (candidate_ops.empty()) {
            break;
        }

        const std::size_t sample = std::min<std::size_t>(candidate_ops.size(), static_cast<std::size_t>(64));
        const std::size_t index = static_cast<std::size_t>(random_.next_int<std::size_t>(0, sample - 1));
        const auto op = candidate_ops[index];

        candidate.field.apply(op);
        candidate.operations.push_back(op);
        candidate.depth = candidate.operations.size();
        candidate.metrics = candidate.field.evaluate_pair_metrics();
        candidate.score = evaluate(candidate);

        update_best(candidate, result, best_score);
        ++result.explored_nodes;

        if (candidate.metrics.status.unmatched == 0) {
            node = std::move(candidate);
            return true;
        }
    }

    if (applied == 0) {
        return false;
    }

    const auto new_unmatched = candidate.metrics.status.unmatched;
    const auto new_distance = candidate.metrics.total_unmatched_distance + candidate.metrics.max_unmatched_distance;

    const bool strict_improvement = new_unmatched < original_unmatched ||
                                    (new_unmatched == original_unmatched && new_distance < original_distance);
    const bool equal_accept = new_unmatched == original_unmatched && new_distance == original_distance &&
                              random_.next_real(0.0, 1.0) < config_.shake_accept_equal_probability;

    if (strict_improvement || equal_accept) {
        node = std::move(candidate);
        return true;
    }

    return false;
}

bool BeamStackSearchSolver::greedy_refinement(const Problem& problem, BeamStackSearchResult& result, Timer& timer,
                                              double& best_score) const {
    if (result.solved) {
        return false;
    }

    Node state;
    state.field = problem.make_field();
    state.operations = result.operations;
    for (const auto& op : state.operations) {
        state.field.apply(op);
    }
    state.metrics = state.field.evaluate_pair_metrics();
    state.depth = state.operations.size();
    state.score = evaluate(state);

    update_best(state, result, best_score);

    if (state.metrics.status.unmatched == 0) {
        result.solved = true;
        result.status = state.metrics.status;
        return true;
    }

    PairMetrics best_metrics = state.metrics;
    std::vector<Operation> best_ops = state.operations;
    bool improved = false;

    const std::size_t max_attempts = std::max<std::size_t>(1, config_.refinement_attempts);
    const std::size_t sample_limit = std::max<std::size_t>(1, config_.refinement_sample);
    const double refinement_start_ms = timer.elapsed_ms();

    for (std::size_t attempt = 0; attempt < max_attempts; ++attempt) {
        if ((config_.time_limit_ms > 0.0 && timer.elapsed_ms() > config_.time_limit_ms) ||
            (config_.refinement_time_budget_ms > 0.0 &&
             timer.elapsed_ms() - refinement_start_ms > config_.refinement_time_budget_ms)) {
            break;
        }

        auto candidate_ops = generate_operations(state.field, state.operations, state.metrics);
        if (candidate_ops.empty()) {
            break;
        }

        const std::size_t inspect = std::min<std::size_t>(candidate_ops.size(), sample_limit);
        Node best_child;
        bool has_child = false;

        for (std::size_t idx = 0; idx < inspect; ++idx) {
            const auto& op = candidate_ops[idx];

            Node child;
            child.field = state.field;
            child.field.apply(op);
            child.operations = state.operations;
            child.operations.push_back(op);
            child.depth = child.operations.size();
            child.metrics = child.field.evaluate_pair_metrics();

            ++result.explored_nodes;

            const auto child_unmatched = child.metrics.status.unmatched;
            const auto state_unmatched = state.metrics.status.unmatched;

            if (child_unmatched > state_unmatched) {
                continue;
            }

            const auto child_distance = child.metrics.total_unmatched_distance + child.metrics.max_unmatched_distance;
            const auto state_distance = state.metrics.total_unmatched_distance + state.metrics.max_unmatched_distance;

            if (child_unmatched == state_unmatched && child_distance >= state_distance) {
                continue;
            }

            if (!has_child ||
                child_unmatched < best_child.metrics.status.unmatched ||
                (child_unmatched == best_child.metrics.status.unmatched &&
                 child.metrics.total_unmatched_distance < best_child.metrics.total_unmatched_distance)) {
                best_child = std::move(child);
                has_child = true;
            }
        }

        if (!has_child) {
            break;
        }

        best_child.score = evaluate(best_child);
        state = std::move(best_child);
        state.metrics = state.field.evaluate_pair_metrics();

        update_best(state, result, best_score);
        improved = true;

        if (state.metrics.status.unmatched < best_metrics.status.unmatched ||
            (state.metrics.status.unmatched == best_metrics.status.unmatched &&
             state.metrics.total_unmatched_distance < best_metrics.total_unmatched_distance)) {
            best_metrics = state.metrics;
            best_ops = state.operations;
        }

        if (state.metrics.status.unmatched == 0) {
            break;
        }
    }

    if (improved) {
        result.operations = best_ops;
        result.status = best_metrics.status;
        result.solved = best_metrics.status.unmatched == 0;
        state.depth = state.operations.size();
    }

    return improved;
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

    const std::size_t max_iterations = config_.adaptive_limits ? std::max<std::size_t>(1, config_.max_iterations) : 1;
    std::size_t iteration = 0;
    std::size_t shakes_used = 0;

    while ((config_.time_limit_ms <= 0.0 || timer.elapsed_ms() < config_.time_limit_ms) && iteration < max_iterations) {
        SearchLimits iter_limits = base_limits;
        if (iteration > 0) {
            const double widen_factor = 1.0 + 0.45 * static_cast<double>(iteration);
            const double node_factor = 1.0 + 0.6 * static_cast<double>(iteration);
            const std::size_t depth_bonus = static_cast<std::size_t>(10 * iteration);
            const std::size_t child_bonus = static_cast<std::size_t>(std::max<std::size_t>(8, iteration * 5));

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

        const bool improvement = next_unmatched < current_unmatched ||
                                 (next_unmatched == current_unmatched && next_distance < current_distance);

        if (!improvement) {
            const bool can_shake = config_.shake_attempts > 0 && shakes_used < config_.shake_attempts &&
                                   (config_.time_limit_ms <= 0.0 ||
                                    timer.elapsed_ms() < config_.time_limit_ms * config_.shake_time_ratio);
            if (can_shake) {
                Node shaken = current_root;
                if (apply_shake(shaken, result, timer, best_score)) {
                    current_root = std::move(shaken);
                    current_root.score = evaluate(current_root);
                    base_limits = iter_limits;
                    ++shakes_used;
                    continue;
                }
            }
            if (iteration + 1 < max_iterations) {
                base_limits = iter_limits;
                ++iteration;
                shakes_used = 0;
                continue;
            }
            break;
        }

        current_root = outcome.best_unsolved;
        current_root.score = evaluate(current_root);
        base_limits = iter_limits;
        shakes_used = 0;
        ++iteration;
    }

    if (!result.solved && (config_.time_limit_ms <= 0.0 || timer.elapsed_ms() < config_.time_limit_ms)) {
        greedy_refinement(problem, result, timer, best_score);
    }

    result.elapsed_ms = timer.elapsed_ms();
    return result;
}

}  // namespace proc36
