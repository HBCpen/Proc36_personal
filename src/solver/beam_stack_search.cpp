#include "solver/beam_stack_search.hpp"

#include <algorithm>
#include <unordered_set>
#include <utility>

namespace proc36 {

BeamStackSearchSolver::BeamStackSearchSolver(BeamStackSearchConfig config)
    : config_(std::move(config)) {}

std::vector<Operation> BeamStackSearchSolver::generate_operations(const Field& field,
                                                                  const std::vector<Operation>& history) const {
    std::vector<Operation> operations;
    operations.reserve(field.size() * field.size());

    const Operation* last_op = history.empty() ? nullptr : &history.back();

    for (auto size : config_.rotation_sizes) {
        if (size < 2 || size > field.size()) {
            continue;
        }
        for (std::size_t y = 0; y + size <= field.size(); ++y) {
            for (std::size_t x = 0; x + size <= field.size(); ++x) {
                Operation op{x, y, size};
                if (!field.is_valid_operation(op)) {
                    continue;
                }
                if (last_op != nullptr && last_op->x == op.x && last_op->y == op.y && last_op->size == op.size) {
                    continue;  // avoid immediately re-applying the same rotation
                }
                operations.push_back(op);
            }
        }
    }

    return operations;
}

double BeamStackSearchSolver::evaluate(const Node& node) const {
    const double matched_score = config_.match_weight * static_cast<double>(node.status.matched);
    const double unmatched_penalty = config_.unmatched_penalty * static_cast<double>(node.status.unmatched);
    const double depth_penalty = config_.depth_penalty * static_cast<double>(node.depth);
    const double op_penalty = config_.operation_penalty * static_cast<double>(node.operations.size());
    const double jitter = random_.next_real(0.0, 1.0) * 1e-3;
    double score = matched_score - unmatched_penalty - depth_penalty - op_penalty + jitter;
    if (node.status.unmatched == 0) {
        score += 1e6;  // strongly prefer solved states
    }
    return score;
}

void BeamStackSearchSolver::update_best(const Node& node, BeamStackSearchResult& best_result, double& best_score) const {
    const double score = node.score;
    if (score > best_score) {
        best_score = score;
        best_result.operations = node.operations;
        best_result.status = node.status;
        best_result.solved = node.status.unmatched == 0;
    }
}

BeamStackSearchResult BeamStackSearchSolver::solve(const Problem& problem) {
    BeamStackSearchResult result;
    Timer timer;

    result.elapsed_ms = 0.0;
    result.explored_nodes = 0;

    Node root;
    root.field = problem.make_field();
    root.status = root.field.evaluate_pairs();
    root.depth = 0;
    root.score = evaluate(root);

    double best_score = -1e18;
    update_best(root, result, best_score);

    std::vector<Node> current_layer;
    current_layer.reserve(config_.beam_width);
    current_layer.push_back(root);

    std::unordered_set<std::uint64_t> visited;
    if (config_.use_global_hash) {
        visited.insert(root.field.zobrist_hash());
    }

    for (std::size_t depth = 0; depth < config_.max_depth && !current_layer.empty(); ++depth) {
        if (config_.time_limit_ms > 0.0 && timer.elapsed_ms() > config_.time_limit_ms) {
            break;
        }

        std::vector<Node> next_layer;
        next_layer.reserve(config_.beam_width * 2);
        bool reached_limit = false;

        for (const auto& node : current_layer) {
            if (config_.time_limit_ms > 0.0 && timer.elapsed_ms() > config_.time_limit_ms) {
                reached_limit = true;
                break;
            }
            if (result.explored_nodes >= config_.max_nodes) {
                reached_limit = true;
                break;
            }

            auto candidate_ops = generate_operations(node.field, node.operations);
            std::vector<Node> children;
            children.reserve(candidate_ops.size());

            for (const auto& op : candidate_ops) {
                if (config_.time_limit_ms > 0.0 && timer.elapsed_ms() > config_.time_limit_ms) {
                    reached_limit = true;
                    break;
                }
                if (result.explored_nodes >= config_.max_nodes) {
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
                }

                child.operations = node.operations;
                child.operations.push_back(op);
                child.depth = node.depth + 1;
                child.status = child.field.evaluate_pairs();
                child.score = evaluate(child);

                children.push_back(std::move(child));
                ++result.explored_nodes;
            }

            if (reached_limit) {
                break;
            }

            if (children.empty()) {
                continue;
            }

            if (config_.max_children_per_node > 0 && children.size() > config_.max_children_per_node) {
                std::partial_sort(children.begin(),
                                  children.begin() + static_cast<std::ptrdiff_t>(config_.max_children_per_node), children.end(),
                                  [](const Node& a, const Node& b) { return a.score > b.score; });
                children.resize(config_.max_children_per_node);
            }

            for (const auto& child : children) {
                update_best(child, result, best_score);
                next_layer.push_back(child);
                if (child.status.unmatched == 0) {
                    goto finished;  // found a solution
                }
            }
        }

        if (reached_limit) {
            break;
        }

        if (next_layer.empty()) {
            break;
        }

        if (next_layer.size() > config_.beam_width) {
            std::partial_sort(next_layer.begin(), next_layer.begin() + static_cast<std::ptrdiff_t>(config_.beam_width),
                              next_layer.end(), [](const Node& a, const Node& b) { return a.score > b.score; });
            next_layer.resize(config_.beam_width);
        }

        current_layer = std::move(next_layer);
    }

finished:
    result.elapsed_ms = timer.elapsed_ms();
    return result;
}

}  // namespace proc36
