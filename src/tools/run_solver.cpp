#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "lib/problem.hpp"
#include "solver/beam_stack_search.hpp"

namespace {

void write_ops_to_file(const std::string& path, const std::vector<proc36::Operation>& ops) {
    std::ofstream ofs(path);
    if (!ofs) {
        throw std::runtime_error("Failed to open output file: " + path);
    }
    ofs << proc36::Problem::serialize_answer(ops) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2 || argc > 3) {
            std::cerr << "Usage: beam_solver <problem.json> [output.json]\n";
            return 1;
        }

        const std::string problem_path = argv[1];
        const auto problem = proc36::Problem::load_from_file(problem_path);

        proc36::BeamStackSearchConfig config;
        if (problem.size > 8) {
            config.rotation_sizes = {2, 3, 4, 5};
        }

        proc36::BeamStackSearchSolver solver(config);
        const auto result = solver.solve(problem);

        std::cout << "BeamStackSearch result:\n";
        std::cout << "  explored nodes: " << result.explored_nodes << '\n';
        std::cout << "  elapsed ms: " << result.elapsed_ms << '\n';
        std::cout << "  matched pairs: " << result.status.matched << '\n';
        std::cout << "  unmatched pairs: " << result.status.unmatched << '\n';
        std::cout << "  operations: " << result.operations.size() << '\n';
        std::cout << (result.solved ? "  status: SOLVED" : "  status: PARTIAL") << '\n';

        if (argc == 3) {
            write_ops_to_file(argv[2], result.operations);
            std::cout << "Operations written to " << argv[2] << '\n';
        } else {
            std::cout << "Serialized answer:\n";
            std::cout << proc36::Problem::serialize_answer(result.operations) << '\n';
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
