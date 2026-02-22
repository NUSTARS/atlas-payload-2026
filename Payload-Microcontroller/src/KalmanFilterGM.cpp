
#include "KalmanFilterGM.h"
#include <math.h>

// All the same as white noise version except for update_timestep()
// no constructor
KalmanFilterGM::KalmanFilterGM() : initialized(false)
{ 
}

void KalmanFilterGM::init(
    float var_process,
    float var_phi,
    float var_omega,
    float tau_corr,
    float dt0,
    Eigen::Matrix<float,3,1>& x0,
    Eigen::Matrix<float,3,3>& P0) {
        this->tau_corr = tau_corr;
        I.setIdentity();
        H << 1.0f, 0.0f, 0.0f, // measurement model matix
            0.0f, 1.0f, 0.0f; // this basically converts x (state) to compare to measurements
        HT = H.transpose();
        R << var_phi, 0.0f, 0.0f, var_omega; // measurement noise matrix
        x_hat = x0; // state
        P = P0; // state covariance 
        update_timestep(dt0);
        initialized = true;
}

void KalmanFilterGM::predict() {
    // this predicts the new x for this iteration using the state transition matrix F
    // F is basically the dynamics of the system in a matrix
    x_hat.noalias() = F * x_hat;
    // P_k+1 = F * P_k * FT + Q. Basically how correlated uncertainity evolves + process noise
    Eigen::Matrix<float, 3, 3> FP;
    FP.noalias() = F * P;
    P.noalias() = FP * FT;
    P += Q;
}

void KalmanFilterGM::update(Eigen::Vector<float,2> z) {
     // this is the innovation ie the error
    Eigen::Vector<float,2> y;
    y = z - H*x_hat;

    // S isn't an important matrix aside from fine tuning the filter
    // it looks like the P = F*P*FT + Q before so it's like the uncertainty associated with
    // the measurements combined with the current uncertainty
    PHT = P * HT;
    S = H * PHT;
    S += R; 

    // K is the kalman gain. Basically it's a gain that says how much to update x and P based on current performance
    // It has the name of the filter, Kalman, in the name so you know it's important
    K = PHT * S.inverse();
    // S.inverse() might be slower, it might be better to use the following line:
    // K = (S.transpose().ldlt().solve(PHT.transpose())).transpose();
    // solving the system of equations is supposed to be faster from what I read online
    // but it kinda looks like a lot more code so idk

    // state vector update is x_k+1 = x_k + Ky
    x_hat += K * y;

    // covariance update is P_k+1 = (I - K*H) * P_k
    Eigen::Matrix<float,3,3> KH = K * H;
    Eigen::Matrix<float,3,3> I_KH = I - KH;
    Eigen::Matrix<float,3,3> Pnew = I_KH * P;
    // alternatively, use the Joseph Form which is more numerically stable. I didn't for speed but
    // I imagine running the filter for longer may lead to this instability. P must be symmetric and
    // and positive semi-definite. Uncomment the code below if it seems like that would be beneficial. 
    // in main.cpp, printing the auto P = kf.covariance() helps with this.

    // joseph form (uncomment if using)
    // Eigen::Matrix<float,3,3> Pnew = I_KH * P;
    // Pnew = Pnew * I_KH.transpose();
    // Pnew += K * R * K.transpose();

    P = 0.5f * (Pnew + Pnew.transpose()); // this is to maintain symmetry 
}

void KalmanFilterGM::update_timestep(float dt) {
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;
    const float dt5 = dt4 * dt;
    /*
    // Gauss-Markov noise model
    This is from my perspective the more physically motivated noise model. It's actually a pretty simple concept.
    The core idea is that the noise in the system is not perfectly white/normal as was previously modelled, so to
    fix this we restate the model in terms of a correlation time
        alphadot = -1/tau * alpha + white noise 
        integrating gives...
        alpha_k+1 = e^(-dt/tau) * alpha + white noise
    The disturbances in alpha decay but with white noise. 
    As a result long tau means longer disturbance durations and short tau means it gets closer to normal white noise
    */
    float a = exp(- dt / tau_corr);
    float g1 = tau_corr * dt - tau_corr * tau_corr * (1 - a*a);
    float g2 = tau_corr * (1 - a);
    float g3 = 1;
    // q_alpha is the variance in alpha through time and is less than var_alpha
    float q_alpha = var_alpha * (1 - a*a);
    // state transition matrix
    // as you can see it looks very similar to the previous one, with that a term now having more interpretability
    // when you integrate further to get omega and phi, the g1 and g2 terms become clear
    F << 1.0f,   dt, g1,
         0.0f, 1.0f, g2,
         0.0f, 0.0f, a;
    FT = F.transpose();
    // noise injector vector
    // this is basically the vector of noise that comes from alpha and how it propogates
    Eigen::Vector<float, 3> Gamma;
    Gamma << g1, g2, g3;
    // process covariance matrix:
    // This is the full matrix and includes the q_alpha variance
    Q = Gamma * Gamma.transpose();
    Q *= q_alpha;
}
