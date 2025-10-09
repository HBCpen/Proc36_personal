#include "lib/problem.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace proc36 {

namespace {

std::size_t parse_size(const std::string& json) {
    constexpr std::string_view key = "\"size\"";
    const auto pos = json.find(key);
    if (pos == std::string::npos) {
        throw std::runtime_error("Problem JSON does not contain size");
    }
    const auto colon = json.find(':', pos + key.size());
    if (colon == std::string::npos) {
        throw std::runtime_error("Problem JSON: malformed size field");
    }
    std::size_t idx = colon + 1;
    while (idx < json.size() && std::isspace(static_cast<unsigned char>(json[idx]))) {
        ++idx;
    }
    std::size_t end = idx;
    while (end < json.size() && std::isdigit(static_cast<unsigned char>(json[end]))) {
        ++end;
    }
    if (idx == end) {
        throw std::runtime_error("Problem JSON: size value missing");
    }
    return static_cast<std::size_t>(std::stoul(json.substr(idx, end - idx)));
}

std::vector<int> parse_entities(const std::string& json, std::size_t size) {
    constexpr std::string_view key = "\"entities\"";
    const auto pos = json.find(key);
    if (pos == std::string::npos) {
        throw std::runtime_error("Problem JSON does not contain entities");
    }
    const auto start = json.find('[', pos + key.size());
    if (start == std::string::npos) {
        throw std::runtime_error("Problem JSON: entities array missing");
    }
    std::size_t depth = 0;
    std::size_t end = start;
    for (; end < json.size(); ++end) {
        const auto ch = json[end];
        if (ch == '[') {
            ++depth;
        } else if (ch == ']') {
            if (depth == 0) {
                throw std::runtime_error("Problem JSON: unmatched closing bracket");
            }
            --depth;
            if (depth == 0) {
                break;
            }
        }
    }
    if (depth != 0) {
        throw std::runtime_error("Problem JSON: entities array not closed");
    }

    std::vector<int> values;
    values.reserve(size * size);
    std::string number;
    for (std::size_t i = start; i <= end; ++i) {
        const char ch = json[i];
        if (std::isdigit(static_cast<unsigned char>(ch))) {
            number.push_back(ch);
        } else if (ch == '-' && number.empty()) {
            number.push_back(ch);
        } else {
            if (!number.empty()) {
                values.push_back(std::stoi(number));
                number.clear();
            }
        }
    }
    if (!number.empty()) {
        values.push_back(std::stoi(number));
        number.clear();
    }

    if (values.size() != size * size) {
        throw std::runtime_error("Problem JSON: entities count mismatch size");
    }
    return values;
}

}  // namespace

Field Problem::make_field() const {
    return Field(size, entities);
}

Problem Problem::load_from_stream(std::istream& is) {
    std::ostringstream oss;
    oss << is.rdbuf();
    return from_json_string(oss.str());
}

Problem Problem::load_from_file(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) {
        throw std::runtime_error("Failed to open problem file: " + path);
    }
    return load_from_stream(ifs);
}

Problem Problem::from_json_string(const std::string& json) {
    const auto size = parse_size(json);
    auto entities = parse_entities(json, size);
    return Problem{size, std::move(entities)};
}

std::string Problem::serialize_answer(const std::vector<Operation>& ops) {
    std::ostringstream oss;
    oss << "{\n  \"ops\": [";
    for (std::size_t i = 0; i < ops.size(); ++i) {
        if (i != 0) {
            oss << ",";
        }
        oss << "\n    " << ops[i].to_string();
    }
    if (!ops.empty()) {
        oss << '\n';
    }
    oss << "  ]\n}";
    return oss.str();
}

}  // namespace proc36
