
#include <ArduinoEigen.h>

// Original white-noise based Kalman Filter
class KalmanFilterWN {
    private:
        // all datatypes are floats for speed. Defining matrices that will be used
        Eigen::Matrix<float, 3, 3> F, FT, Q, P, I;
        Eigen::Matrix<float, 2, 2> R, S;
        Eigen::Matrix<float, 2, 3> H;
        Eigen::Matrix<float, 3, 2> HT, PHT, K;
        Eigen::Matrix<float, 3, 1> x_hat;

        const float var_phi = 0.2f * 0.2f;
        const float noise_density = 0.0035f; // deg/s/sqrt(Hz)
        const float BW = 265.0f;
        const float var_omega = noise_density * noise_density * BW;
        const float var_process = 1e1f;
        
        const float damping = -0.1f;
        const float accel_passthrough = 0.95f;
        bool initialized;

public:
    // no constructor
    KalmanFilterWN();

    // using init function instead with kalman parameters and initial state + covariance matrix
    void init(
        float dt0, 
        Eigen::Matrix<float,3,1>& x0, 
        Eigen::Matrix<float,3,3>& P0);

    // function that sets up the time dependent matrices. Likely only used once in init
    // but good to have if we want to play around with variable timesteps
    void update_timestep(float dt);
    // update x_hat and P_hat
    void predict();
    // update x_hat and P_hat based on new measurements, z
    void update(Eigen::Vector<float,2> z);

    const Eigen::Matrix<float,3,1> state() const { return x_hat; } // getter for state vector
    const Eigen::Matrix<float,3,3> covariance() const { return P; } // getter for state covariance matrix
};
