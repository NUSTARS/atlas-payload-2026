#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>

#include <chrono>
#include <algorithm>

#include "csv_utils.h"
#include "KalmanFilter.h"

struct PCovDiagnostics {
    bool ok = true;
    double symmetry_inf = 0.0;
    double min_eig = 0.0;
    double max_eig = 0.0;
    double cond_est = 0.0;
};

inline PCovDiagnostics diagnoseP(const Eigen::Matrix3d& P) {
    PCovDiagnostics d;

    // finite check
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const double v = P(r,c);
            if (!std::isfinite(v)) {
                d.ok = false;
                return d;
            }
        }
    }

    d.symmetry_inf = (P - P.transpose()).cwiseAbs().maxCoeff();

    // eigenvalues of symmetrized P (best practice for covariance checks)
    const Eigen::Matrix3d Ps = 0.5 * (P + P.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Ps);
    if (es.info() != Eigen::Success) {
        d.ok = false;
        return d;
    }

    const auto evals = es.eigenvalues();
    d.min_eig = evals(0);
    d.max_eig = evals(2);

    const double eps = 1e-15;
    d.cond_est = (std::abs(d.min_eig) > eps) ? (d.max_eig / d.min_eig) : std::numeric_limits<double>::infinity();

    // "ok" if PSD-ish and symmetric-ish
    // tweak thresholds to your noise scales
    if (d.symmetry_inf > 1e-9) d.ok = false;
    if (d.min_eig < -1e-9)     d.ok = false;

    return d;
}

inline bool shouldUseJoseph(const Eigen::Matrix3d& P) {
    const auto d = diagnoseP(P);
    // If it is drifting non-symmetric or not PSD -> Joseph is safer
    return (!d.ok);
}


double benchmark_kf(
    KalmanFilter& kf,
    const std::vector<Row>& data,
    int repeats = 200,
    int warmup = 10)
{
    std::vector<double> times_ms;
    times_ms.reserve(repeats);

    volatile double sink = 0.0; // prevents dead-code elimination

    for (int r = 0; r < repeats + warmup; ++r) {
        auto t0 = std::chrono::steady_clock::now();

        for (size_t k = 0; k < data.size(); ++k) {
            if (k > 0) kf.predict();
            //Eigen::Vector2d z(data[k].phi_meas, data[k].omega_meas);
            //kf.update(z);
            kf.update(data[k].phi_meas, data[k].omega_meas);
            if (k == data.size() - 1) {
                auto diag = diagnoseP(kf.covariance());
                std::cout << "k=" << k
                        << " symInf=" << diag.symmetry_inf
                        << " minEig=" << diag.min_eig
                        << " cond~=" << diag.cond_est
                        << " ok=" << diag.ok << "\n";
            }  

            sink += kf.state()(0);
        }

        auto t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> dt = t1 - t0;

        if (r >= warmup) times_ms.push_back(dt.count());
    }

    std::sort(times_ms.begin(), times_ms.end());
    const double median = times_ms[times_ms.size() / 2];

    std::cout << "Median time over " << repeats << " runs: "
              << median << " ms (sink=" << sink << ")\n";

    return median;
}

int main() {
    const std::string input_path  = "VNYMR_input.csv";
    const std::string output_path = "VNYMR_output.csv";

    const std::string TIME_COL  = "time";
    const std::string PHI_COL   = "phi_meas";
    const std::string OMEGA_COL = "omega_meas";
    const std::string ALPHA_COL = "alpha_filtered";

    const auto data = readCSV(input_path, TIME_COL, PHI_COL, OMEGA_COL, ALPHA_COL);
    if (data.size() < 2) {
        std::cerr << "Not enough data rows.\n";
        return 1;
    }

    std::ofstream fout(output_path);

    double var_process = 1e3;
    double var_phi = 1e0;
    double var_omega = 1e1;
    double damping = -0.1;
    double accel_passthrough = 0.95;
    double dt0 = data[1].t - data[0].t;

    Eigen::Matrix3d P0;
    P0 << 0.1, 0, 0,
          0, 0.1, 0,
          0, 0, 100;

    Eigen::Vector3d x0;
    x0 << data[0].phi_meas, data[0].omega_meas, data[0].alpha_meas;

    KalmanFilter kf(var_process, var_phi, var_omega,
                    damping, accel_passthrough, dt0, x0, P0);

    benchmark_kf(kf, data, 200, 10);

    return 0;
}
