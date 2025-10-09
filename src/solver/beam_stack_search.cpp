#include "solver/beam_stack_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_set>

#include "lib/timer.hpp"

namespace proc36 {

BeamStackSearchSolver::BeamStackSearchSolver(const BeamStackSearchConfig& config)
    : config_(config) {}

double BeamStackSearchSolver::evaluate(const Field& field, std::size_t num_operations) const {
    const auto status = field.evaluate_pairs();
    
    // すべてのペアが揃っている場合は高スコア
    if (status.unmatched == 0 && status.matched * 2 == field.cell_count()) {
        return 10000000.0 - static_cast<double>(num_operations) * config_.operation_penalty;
    }
    
    // マッチしたペアの数を基本スコアとする
    double score = static_cast<double>(status.matched) * 10000.0;
    
    // アンマッチのペアの距離を計算（近いほど良い）
    const auto field_size = field.size();
    const auto max_value = field.cell_count() / 2;
    double distance_penalty = 0.0;
    
    for (std::size_t value = 0; value < max_value; ++value) {
        auto positions = field.positions_of(static_cast<int>(value));
        if (positions.size() == 2) {
            const auto& p1 = positions[0];
            const auto& p2 = positions[1];
            const auto dx = (p1.x > p2.x) ? (p1.x - p2.x) : (p2.x - p1.x);
            const auto dy = (p1.y > p2.y) ? (p1.y - p2.y) : (p2.y - p1.y);
            const auto manhattan_dist = dx + dy;
            
            if (manhattan_dist == 1) {
                // 既にペアになっている（マッチ済み）
                continue;
            } else {
                // マンハッタン距離が遠いほどペナルティ
                distance_penalty += static_cast<double>(manhattan_dist) * 50.0;
            }
        }
    }
    
    score -= distance_penalty;
    
    // 操作回数のペナルティ
    score -= static_cast<double>(num_operations) * config_.operation_penalty;
    
    return score;
}

std::vector<BeamStackSearchSolver::Node> BeamStackSearchSolver::generate_children(const Node& parent) const {
    std::vector<Node> children;
    children.reserve(config_.max_children_per_node);
    
    const auto field_size = parent.field.size();
    
    // すべての可能な回転操作を生成
    std::vector<Operation> candidate_ops;
    for (const auto rot_size : config_.rotation_sizes) {
        if (rot_size > field_size) {
            continue;
        }
        
        for (std::size_t y = 0; y + rot_size <= field_size; ++y) {
            for (std::size_t x = 0; x + rot_size <= field_size; ++x) {
                candidate_ops.push_back(Operation{x, y, rot_size});
            }
        }
    }
    
    // 各操作を試す
    for (const auto& op : candidate_ops) {
        Node child;
        child.field = parent.field.applied(op);
        child.operations = parent.operations;
        child.operations.push_back(op);
        child.hash = child.field.zobrist_hash();
        child.score = evaluate(child.field, child.operations.size());
        
        children.push_back(std::move(child));
        
        if (children.size() >= config_.max_children_per_node) {
            break;
        }
    }
    
    return children;
}

void BeamStackSearchSolver::select_beam(std::vector<Node>& nodes) const {
    if (nodes.size() <= config_.beam_width) {
        return;
    }
    
    // スコアの高い順にソート
    std::partial_sort(nodes.begin(), 
                     nodes.begin() + static_cast<long>(config_.beam_width),
                     nodes.end());
    
    // 上位beam_width個のみを残す
    nodes.resize(config_.beam_width);
}

BeamStackSearchResult BeamStackSearchSolver::solve(const Problem& problem) {
    const auto start_time = std::chrono::high_resolution_clock::now();
    
    BeamStackSearchResult result;
    result.explored_nodes = 0;
    
    // 初期状態
    Node initial;
    initial.field = problem.make_field();
    initial.hash = initial.field.zobrist_hash();
    initial.score = evaluate(initial.field, 0);
    
    // 初期状態がゴールかチェック
    if (initial.field.is_goal_state()) {
        result.solved = true;
        result.status = initial.field.evaluate_pairs();
        const auto end_time = std::chrono::high_resolution_clock::now();
        result.elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        return result;
    }
    
    // ビームサーチのメインループ
    std::vector<Node> current_beam;
    current_beam.push_back(std::move(initial));
    
    Node best_node = current_beam[0];
    
    for (std::size_t depth = 0; depth < config_.max_depth; ++depth) {
        // 時間制限チェック
        const auto current_time = std::chrono::high_resolution_clock::now();
        const double elapsed = std::chrono::duration<double, std::milli>(current_time - start_time).count();
        if (elapsed >= config_.time_limit_ms) {
            break;
        }
        
        std::vector<Node> next_beam;
        std::unordered_set<std::uint64_t> visited_hashes;
        
        // 各ノードから子ノードを生成
        for (const auto& node : current_beam) {
            auto children = generate_children(node);
            result.explored_nodes += children.size();
            
            for (auto& child : children) {
                // 重複チェック
                if (visited_hashes.count(child.hash) > 0) {
                    continue;
                }
                visited_hashes.insert(child.hash);
                
                // ゴール状態をチェック
                if (child.field.is_goal_state()) {
                    result.operations = std::move(child.operations);
                    result.status = child.field.evaluate_pairs();
                    result.solved = true;
                    
                    const auto end_time = std::chrono::high_resolution_clock::now();
                    result.elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                    return result;
                }
                
                // ベストノードの更新
                if (child.score > best_node.score) {
                    best_node = child;
                }
                
                next_beam.push_back(std::move(child));
            }
            
            // ノード数制限チェック
            if (result.explored_nodes >= config_.max_nodes) {
                break;
            }
        }
        
        if (next_beam.empty()) {
            break;
        }
        
        // ビーム選択
        select_beam(next_beam);
        current_beam = std::move(next_beam);
        
        // ノード数制限チェック
        if (result.explored_nodes >= config_.max_nodes) {
            break;
        }
    }
    
    // ゴールに到達しなかった場合は、最良ノードを返す
    result.operations = std::move(best_node.operations);
    result.status = best_node.field.evaluate_pairs();
    result.solved = best_node.field.is_goal_state();
    
    const auto end_time = std::chrono::high_resolution_clock::now();
    result.elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    return result;
}

}  // namespace proc36
