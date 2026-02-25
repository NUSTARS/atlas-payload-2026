
#include <ArduinoEigen.h>

// Gauss-Markov based Kalman Filter
class KalmanFilterGM {
    private:
        // all datatypes are floats for speed. Defining matrices that will be used
        Eigen::Matrix<float, 3, 3> F, FT, Q, P, I;
        Eigen::Matrix<float, 2, 2> R, S;
        Eigen::Matrix<float, 2, 3> H;
        Eigen::Matrix<float, 3, 2> HT, PHT, K;
        Eigen::Matrix<float, 3, 1> x_hat;

        float var_alpha;
        float var_phi;
        float var_omega;
        float tau_corr;
        bool initialized;

public:
    // no constructor
    KalmanFilterGM();

    // using init function instead with kalman parameters and initial state + covariance matrix
    void init(
        float var_alpha,
        float var_phi,
        float var_omega,
        float tau_corr,
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
