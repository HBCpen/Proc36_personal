#pragma once

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <thread>
#include <type_traits>
#include <vector>

#include "lib/field.hpp"
#include "lib/operation.hpp"
#include "lib/problem.hpp"

namespace proc36 {

struct BeamStackSearchConfig {
    // 探索パラメータ
    std::size_t beam_width = 100;           // ビーム幅
    std::size_t max_depth = 1000;            // 最大探索深度
    std::size_t max_nodes = 10'000'000;     // 最大探索ノード数
    std::size_t max_children_per_node = 50; // 1ノードあたりの最大子ノード数

    // 並列探索パラメータ
    std::size_t max_parallel_tasks = 0;     // 0の場合はハードウェア並列数を使用
    
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

    template <typename Range, typename Func>
    void parallel_for_each(Range&& range, Func&& func) const {
        using std::begin;
        using std::end;

        auto first = begin(range);
        auto last = end(range);
        using Iterator = decltype(first);
        using Category = typename std::iterator_traits<Iterator>::iterator_category;

        const auto total = static_cast<std::size_t>(std::distance(first, last));
        if (total == 0) {
            return;
        }

        std::size_t task_count = config_.max_parallel_tasks;
        if (task_count == 0) {
            task_count = std::thread::hardware_concurrency();
        }
        if (task_count == 0) {
            task_count = 1;
        }
        task_count = std::min(task_count, total);

        if constexpr (!std::is_base_of_v<std::random_access_iterator_tag, Category>) {
            for (auto it = first; it != last; ++it) {
                func(*it);
            }
        } else {
            if (task_count <= 1) {
                for (auto it = first; it != last; ++it) {
                    func(*it);
                }
                return;
            }

            const auto chunk_size = (total + task_count - 1) / task_count;
            std::vector<std::thread> workers;
            workers.reserve(task_count);

            for (std::size_t task = 0; task < task_count; ++task) {
                const std::size_t start = task * chunk_size;
                if (start >= total) {
                    break;
                }
                const std::size_t finish = std::min(total, start + chunk_size);
                workers.emplace_back([start, finish, first, &func]() {
                    for (std::size_t index = start; index < finish; ++index) {
                        func(*(first + static_cast<std::ptrdiff_t>(index)));
                    }
                });
            }

            for (auto& worker : workers) {
                worker.join();
            }
        }
    }
    
    // ビームの選択（上位beam_width個を選択）
    void select_beam(std::vector<Node>& nodes) const;
    
    BeamStackSearchConfig config_;
};

}  // namespace proc36
