
#include <ArduinoEigen.h>

// Original white-noise based Kalman Filter
class KalmanFilter {
    private:
        // all datatypes are floats for speed. Defining matrices that will be used
        Eigen::Matrix<float, 3, 3> F, FT, Q, P, I;
        Eigen::Matrix<float, 2, 2> R, S;
        Eigen::Matrix<float, 2, 3> H;
        Eigen::Matrix<float, 3, 2> HT, PHT, K;
        Eigen::Matrix<float, 3, 1> x_hat, B;
        Eigen::Vector<float,2> y;

        const float var_phi = 0.2f * 0.2f; //1e-5;
        const float noise_density = 0.0035f; // deg/s/sqrt(Hz)
        const float BW = 265.0f;
        const float var_omega = noise_density * noise_density * BW; // 1e-6;

        const float var_process = 1e10f;
        const float damping = -0.1f;
        const float accel_passthrough = 0.95f;
        const float MOI = 1; // FIXME: add actual moment of inertia about roll axis
        bool initialized;

public:
    // no constructor
    KalmanFilter();

    // using init function instead with kalman parameters and initial state + covariance matrix
    void init(
        float dt0, 
        Eigen::Vector<float,2> z0);

    // function that sets up the time dependent matrices. Likely only used once in init
    // but good to have if we want to play around with variable timesteps
    void update_timestep(float dt);
    // update x_hat and P_hat
    void predict(float u);
    // update x_hat and P_hat based on new measurements, z
    void update(Eigen::Vector<float,2> z);

    const Eigen::Matrix<float,3,1> state() const { return x_hat; } // getter for state vector
    const Eigen::Matrix<float,3,3> covariance() const { return P; } // getter for state covariance matrix
    const Eigen::Matrix<float,3,3> getF() const { return F; } // getter for F matrix
    const Eigen::Matrix<float,3,3> getQ() const { return Q; } // getter for state covariance matrix
    const Eigen::Matrix<float,3,2> getK() const { return K; } // getter for state covariance matrix
    const Eigen::Vector<float,2> innovation() const { return y; }
};

Eigen::Vector<float,2> meas_vector(float roll_heading, float roll_velocity);