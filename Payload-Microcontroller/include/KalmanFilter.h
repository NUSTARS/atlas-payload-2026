#pragma once
#include <ArduinoEigenDense.h>

class KalmanFilter {
public:
    KalmanFilter(
        double var_process,
        double var_phi,
        double var_omega,
        double damping,
        double accel_passthrough,
        double dt0,
        const Eigen::Vector3d& x0,
        const Eigen::Matrix3d& P0
    );

    void update_timestep(double dt);
    void predict();
    void update(double phi_meas, double omega_meas);

    const Eigen::Vector3d& state() const { return x_hat; }
    const Eigen::Matrix3d& covariance() const { return P; }

private:
    Eigen::Matrix3d F, FT, Q, P, I;
    Eigen::Matrix2d R, S;
    Eigen::Matrix<double, 2, 3> H;
    Eigen::Matrix<double, 3, 2> HT, PHT, K;
    Eigen::Vector3d x_hat;

    double var_process;
    double damping;
    double accel_passthrough;
};

