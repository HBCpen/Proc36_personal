#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include <sstream>

#include "lib/field.hpp"
#include "lib/problem.hpp"

namespace {

std::size_t find_matching_bracket(const std::string& s, std::size_t open_idx) {
    std::size_t depth = 0;
    for (std::size_t i = open_idx; i < s.size(); ++i) {
        if (s[i] == '[') {
            ++depth;
        } else if (s[i] == ']') {
            if (depth == 0) {
                throw std::runtime_error("Malformed JSON: stray closing bracket");
            }
            --depth;
            if (depth == 0) {
                return i;
            }
        }
    }
    throw std::runtime_error("Malformed JSON: bracket not closed");
}

std::size_t parse_size_t_after_colon(const std::string& s, std::size_t key_pos) {
    auto colon = s.find(':', key_pos);
    if (colon == std::string::npos) {
        throw std::runtime_error("Malformed JSON: colon not found");
    }
    std::size_t idx = colon + 1;
    while (idx < s.size() && std::isspace(static_cast<unsigned char>(s[idx]))) {
        ++idx;
    }
    std::size_t end = idx;
    while (end < s.size() && std::isdigit(static_cast<unsigned char>(s[end]))) {
        ++end;
    }
    if (idx == end) {
        throw std::runtime_error("Malformed JSON: integer expected");
    }
    return static_cast<std::size_t>(std::stoul(s.substr(idx, end - idx)));
}

std::vector<proc36::Operation> parse_operations(const std::string& json) {
    using proc36::Operation;
    std::vector<Operation> ops;
    constexpr std::string_view key = "\"ops\"";
    auto pos = json.find(key);
    if (pos == std::string::npos) {
        return ops;
    }
    auto start = json.find('[', pos + key.size());
    if (start == std::string::npos) {
        throw std::runtime_error("Malformed ops JSON: missing array");
    }
    const auto end = find_matching_bracket(json, start);

    std::size_t cursor = start;
    while (true) {
        auto x_pos = json.find("\"x\"", cursor);
        if (x_pos == std::string::npos || x_pos > end) {
            break;
        }
        std::size_t x = parse_size_t_after_colon(json, x_pos);

        auto y_pos = json.find("\"y\"", x_pos + 3);
        if (y_pos == std::string::npos || y_pos > end) {
            throw std::runtime_error("Malformed ops JSON: missing y");
        }
        std::size_t y = parse_size_t_after_colon(json, y_pos);

        auto n_pos = json.find("\"n\"", y_pos + 3);
        if (n_pos == std::string::npos || n_pos > end) {
            throw std::runtime_error("Malformed ops JSON: missing n");
        }
        std::size_t n = parse_size_t_after_colon(json, n_pos);

        ops.push_back(Operation{x, y, n});
        cursor = n_pos + 3;
    }

    return ops;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2 || argc > 3) {
            std::cerr << "Usage: local_runner <problem.json> [ops.json]\n";
            return EXIT_FAILURE;
        }

        const std::string problem_path = argv[1];
        const auto problem = proc36::Problem::load_from_file(problem_path);
        auto field = problem.make_field();

        std::cout << "Initial field (size=" << problem.size << "):\n";
        std::cout << field.to_string() << "\n";
        const auto initial_status = field.evaluate_pairs();
        std::cout << "Matched pairs: " << initial_status.matched
                  << ", Unmatched pairs: " << initial_status.unmatched << "\n";

        if (argc == 3) {
            const std::string ops_path = argv[2];
            std::ifstream ops_file(ops_path);
            if (!ops_file) {
                throw std::runtime_error("Failed to open ops file: " + ops_path);
            }
            std::ostringstream oss;
            oss << ops_file.rdbuf();
            auto operations = parse_operations(oss.str());
            std::cout << "Applying " << operations.size() << " operations...\n";
            for (std::size_t i = 0; i < operations.size(); ++i) {
                const auto& op = operations[i];
                if (!field.is_valid_operation(op)) {
                    throw std::runtime_error("Invalid operation at index " + std::to_string(i));
                }
                field.apply(op);
            }

            std::cout << "Final field:\n" << field.to_string() << "\n";
            const auto final_status = field.evaluate_pairs();
            std::cout << "Matched pairs: " << final_status.matched
                      << ", Unmatched pairs: " << final_status.unmatched << "\n";
            std::cout << (field.is_goal_state() ? "All pairs aligned." : "Pairs still unmatched.") << "\n";
        }

        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
