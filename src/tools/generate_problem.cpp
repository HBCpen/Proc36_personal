#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    try {
        if (argc < 3 || argc > 4) {
            std::cerr << "Usage: generate_problem <size> <output.json> [seed]\n";
            return 1;
        }

        const int size = std::stoi(argv[1]);
        if (size % 2 != 0 || size < 4 || size > 24) {
            std::cerr << "Size must be an even integer between 4 and 24.\n";
            return 1;
        }

        std::uint64_t seed = std::random_device{}();
        if (argc == 4) {
            seed = static_cast<std::uint64_t>(std::stoull(argv[3]));
        }

        const std::size_t cell_count = static_cast<std::size_t>(size) * static_cast<std::size_t>(size);
        const std::size_t pair_count = cell_count / 2;

        std::vector<int> values(cell_count);
        for (std::size_t v = 0; v < pair_count; ++v) {
            values[2 * v] = static_cast<int>(v);
            values[2 * v + 1] = static_cast<int>(v);
        }

        std::mt19937_64 rng(seed);
        std::shuffle(values.begin(), values.end(), rng);

        std::ofstream ofs(argv[2]);
        if (!ofs) {
            std::cerr << "Failed to open output file: " << argv[2] << "\n";
            return 1;
        }

        ofs << "{\n";
        ofs << "  \"startsAt\": 0,\n";
        ofs << "  \"problem\": {\n";
        ofs << "    \"field\": {\n";
        ofs << "      \"size\": " << size << ",\n";
        ofs << "      \"entities\": [\n";
        for (int y = 0; y < size; ++y) {
            ofs << "        [";
            for (int x = 0; x < size; ++x) {
                const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(size) + static_cast<std::size_t>(x);
                ofs << values[idx];
                if (x + 1 != size) {
                    ofs << ", ";
                }
            }
            ofs << "]";
            if (y + 1 != size) {
                ofs << ",";
            }
            ofs << "\n";
        }
        ofs << "      ]\n";
        ofs << "    }\n";
        ofs << "  }\n";
        ofs << "}\n";

        std::cout << "Generated problem of size " << size << " to " << argv[2] << " (seed=" << seed << ")\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
