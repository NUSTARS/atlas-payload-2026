#include "csv_utils.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

static inline std::string trim(const std::string& s) {
    size_t i = 0, j = s.size();
    while (i < j && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1]))) --j;
    return s.substr(i, j - i);
}

static std::vector<std::string> splitCSVLine(const std::string& line) {
    // Simple CSV split: supports commas and double-quoted fields that may contain commas.
    std::vector<std::string> out;
    std::string cur;
    bool in_quotes = false;

    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];

        if (c == '"') {
            // Escaped quote inside quoted field: "" -> "
            if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
                cur.push_back('"');
                ++i;
            } else {
                in_quotes = !in_quotes;
            }
        } else if (c == ',' && !in_quotes) {
            out.push_back(trim(cur));
            cur.clear();
        } else {
            cur.push_back(c);
        }
    }
    out.push_back(trim(cur));
    return out;
}

static double toDoubleOrThrow(const std::string& s, const std::string& field_name, size_t row_idx) {
    try {
        size_t pos = 0;
        double val = std::stod(s, &pos);
        if (pos != s.size()) throw std::runtime_error("trailing characters");
        return val;
    } catch (...) {
        std::ostringstream oss;
        oss << "Failed to parse field '" << field_name << "' at row " << row_idx
            << " with value '" << s << "'";
        throw std::runtime_error(oss.str());
    }
}

std::vector<Row> readCSV(const std::string& path,
                         const std::string& time_col,
                         const std::string& phi_col,
                         const std::string& omega_col,
                         const std::string& alpha_col)
{
    std::ifstream fin(path);
    if (!fin) throw std::runtime_error("Could not open input CSV: " + path);

    std::string header;
    if (!std::getline(fin, header)) throw std::runtime_error("Input CSV is empty.");

    const auto cols = splitCSVLine(header);

    std::unordered_map<std::string, int> idx;
    idx.reserve(cols.size());
    for (int i = 0; i < static_cast<int>(cols.size()); ++i) {
        idx[cols[i]] = i;
    }

    auto requireCol = [&](const std::string& name) -> int {
        auto it = idx.find(name);
        if (it == idx.end()) {
            std::ostringstream oss;
            oss << "Missing required column '" << name << "'. Found columns: ";
            for (size_t k = 0; k < cols.size(); ++k) {
                oss << cols[k] << (k + 1 < cols.size() ? ", " : "");
            }
            throw std::runtime_error(oss.str());
        }
        return it->second;
    };

    const int it = requireCol(time_col);
    const int ip = requireCol(phi_col);
    const int io = requireCol(omega_col);
    const int ia = requireCol(alpha_col);

    std::vector<Row> rows;
    std::string line;
    size_t row_idx = 1; // data starts at row 1 (header is row 0)
    while (std::getline(fin, line)) {
        if (trim(line).empty()) { ++row_idx; continue; }

        const auto fields = splitCSVLine(line);

        const int need = std::max({it, ip, io, ia});
        if (static_cast<int>(fields.size()) <= need) {
            std::ostringstream oss;
            oss << "Row " << row_idx << " has too few columns (need index " << need
                << ", got " << fields.size() << ").";
            throw std::runtime_error(oss.str());
        }

        Row r;
        r.t = toDoubleOrThrow(fields[it], time_col, row_idx);
        r.phi_meas = toDoubleOrThrow(fields[ip], phi_col, row_idx);
        r.omega_meas = toDoubleOrThrow(fields[io], omega_col, row_idx);
        r.alpha_meas = toDoubleOrThrow(fields[ia], alpha_col, row_idx);

        rows.push_back(r);
        ++row_idx;
    }

    if (rows.size() < 2) throw std::runtime_error("Need at least 2 data rows to compute dt.");

    return rows;
}

void writeOutputHeader(std::ostream& out) {
    out << "time,phi_meas,omega_meas,alpha_filtered,phi_hat,omega_hat,alpha_hat\n";
}

void writeOutputRow(std::ostream& out,
                    double t,
                    double phi_meas,
                    double omega_meas,
                    double alpha_meas,
                    double phi_hat,
                    double omega_hat,
                    double alpha_hat)
{
    // Use high precision so you don't quantize states/measurements.
    out << std::setprecision(10)
        << t << ","
        << phi_meas << ","
        << omega_meas << ","
        << alpha_meas << ","
        << phi_hat << ","
        << omega_hat << ","
        << alpha_hat << "\n";
}
