#pragma once

#include <string>
#include <vector>

/**
 * A single input data row.
 * Assumes your input CSV contains:
 *   time, phi_meas, omega_meas
 */
struct Row {
    double t;
    double phi_meas;
    double omega_meas;
    double alpha_meas;
};

/**
 * Read a CSV file and return parsed rows.
 *
 * @param path
 * @param time_col
 * @param phi_col
 * @param omega_col
 * @param alpha_col
 * @throws std::runtime_error on any parse/IO error
 */
std::vector<Row> readCSV(const std::string& path,
                         const std::string& time_col,
                         const std::string& phi_col,
                         const std::string& omega_col,
                         const std::string& alpha_col);

/**
 * Write the output CSV header.
 */
void writeOutputHeader(std::ostream& out);

/**
 * Write one output row: time, measurements, and KF estimates.
 */
void writeOutputRow(std::ostream& out,
                    double t,
                    double phi_meas,
                    double omega_meas,
                    double alpha_meas,
                    double phi_hat,
                    double omega_hat,
                    double alpha_hat);
