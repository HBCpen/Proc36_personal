#pragma once

#include <cstddef>
#include <istream>
#include <string>
#include <vector>

#include "lib/field.hpp"
#include "lib/operation.hpp"

namespace proc36 {

struct Problem {
    std::size_t size{};
    std::vector<int> entities;  // row-major, length = size * size

    [[nodiscard]] Field make_field() const;

    static Problem load_from_stream(std::istream& is);
    static Problem load_from_file(const std::string& path);
    static Problem from_json_string(const std::string& json);

    static std::string serialize_answer(const std::vector<Operation>& ops);
};

}  // namespace proc36
