#pragma once

#include <cstddef>
#include <vector>

#include "lib/field.hpp"
#include "lib/operation.hpp"
#include "lib/problem.hpp"

namespace proc36 {

struct BeamStackSearchConfig {
    // 探索パラメータ
    std::size_t beam_width = 100;           // ビーム幅
    std::size_t max_depth = 100;            // 最大探索深度
    std::size_t max_nodes = 10'000'000;     // 最大探索ノード数
    std::size_t max_children_per_node = 50; // 1ノードあたりの最大子ノード数
    
    // 回転操作のサイズ
    std::vector<std::size_t> rotation_sizes = {2, 3, 4, 5};
    
    // 評価関数パラメータ
    double operation_penalty = 0.1;  // 操作回数のペナルティ係数
    
    // 時間制限（5分 = 300秒 = 300,000ミリ秒）
    double time_limit_ms = 290'000.0;  // タイムリミット（ミリ秒）、少し余裕を持たせる
};

struct BeamStackSearchResult {
    std::vector<Operation> operations;  // 見つかった操作列
    PairStatus status;                  // 最終的なペア状態
    std::size_t explored_nodes = 0;     // 探索したノード数
    double elapsed_ms = 0.0;            // 経過時間（ミリ秒）
    bool solved = false;                // すべてのペアが揃ったか
};

class BeamStackSearchSolver {
public:
    explicit BeamStackSearchSolver(const BeamStackSearchConfig& config);
    
    BeamStackSearchResult solve(const Problem& problem);

private:
    struct Node {
        Field field;
        std::vector<Operation> operations;
        double score = 0.0;
        std::uint64_t hash = 0;
        
        bool operator<(const Node& other) const {
            return score > other.score;  // スコアの高い順
        }
    };
    
    // 評価関数
    double evaluate(const Field& field, std::size_t num_operations) const;
    
    // 子ノードの生成
    std::vector<Node> generate_children(const Node& parent) const;
    
    // ビームの選択（上位beam_width個を選択）
    void select_beam(std::vector<Node>& nodes) const;
    
    BeamStackSearchConfig config_;
};

}  // namespace proc36
