#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(
    double var_process,
    double var_phi,
    double var_omega,
    double damping,
    double accel_passthrough,
    double dt0,
    const Eigen::Vector3d& x0,
    const Eigen::Matrix3d& P0)
    : var_process(var_process),
      damping(damping),
      accel_passthrough(accel_passthrough)
{
    I.setIdentity();

    x_hat = x0;
    P = P0;

    H << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0;
    HT = H.transpose();
    R << var_phi, 0.0, 0.0, var_omega;
    update_timestep(dt0);
}

void KalmanFilter::predict() {
    x_hat.noalias() = F * x_hat;
    Eigen::Matrix3d FP;
    FP.noalias() = F * P;
    P.noalias() = FP * FT;
    P += Q;
}

//void KalmanFilter::update(const Eigen::Vector2d& z) {
void KalmanFilter::update(double phi_meas, double omega_meas) {
    PHT.col(0) = P.col(0);
    PHT.col(1) = P.col(1);
    S(0,0) = P(0,0) + R(0,0);
    S(0,1) = P(0,1) + R(0,1);
    S(1,0) = P(1,0) + R(1,0);
    S(1,1) = P(1,1) + R(1,1);
    const double a = S(0,0), b = S(0,1), c = S(1,0), d = S(1,1);
    const double det = a*d - b*c;
    const double inv_det = 1.0 / det;
    const double Sinv00 =  d * inv_det;
    const double Sinv01 = -b * inv_det;
    const double Sinv10 = -c * inv_det;
    const double Sinv11 =  a * inv_det;
    K.col(0).noalias() = PHT.col(0) * Sinv00 + PHT.col(1) * Sinv10;
    K.col(1).noalias() = PHT.col(0) * Sinv01 + PHT.col(1) * Sinv11;
    const double y0 = phi_meas   - x_hat(0);
    const double y1 = omega_meas - x_hat(1);
    x_hat.noalias() += K.col(0) * y0 + K.col(1) * y1;
    const Eigen::RowVector3d Prow0 = P.row(0);
    const Eigen::RowVector3d Prow1 = P.row(1);
    P.noalias() -= K.col(0) * Prow0;
    P.noalias() -= K.col(1) * Prow1;
}

void KalmanFilter::update_timestep(double dt) {
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double dt5 = dt4 * dt;
    // state transition matrix
    F << 1.0, dt,  0.5*dt2,
         0.0, 1.0, dt,
         0.0, damping, accel_passthrough;
    FT = F.transpose();
    // process covariance matrix
    Q.setZero();
    Q(0,0) = dt5/20.0; Q(0,1) = dt4/8.0;  Q(0,2) = dt3/6.0;
    Q(1,0) = dt4/8.0;  Q(1,1) = dt3/3.0;  Q(1,2) = dt2/2.0;
    Q(2,0) = dt3/6.0;  Q(2,1) = dt2/2.0;  Q(2,2) = dt;
    Q *= var_process;
}