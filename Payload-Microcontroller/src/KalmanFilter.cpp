
#include "KalmanFilter.h"

static float wrapAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// no constructor
KalmanFilter::KalmanFilter() : initialized(false)
{ 
}

void KalmanFilter::init(float dt0, Eigen::Vector<float,2> z0) {
        // initial states 
        Eigen::Matrix<float,3,3> P0;
        P0 << 0.1, 0, 0,
                0, 0.1, 0,
                0, 0, 100; // acc has higher covariance
        Eigen::Matrix<float,3,1> x0;
        x0 << z0[0], z0[1], 0.0f;

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

void KalmanFilter::predict() { 
    // this predicts the new x for this iteration using the state transition matrix F
    // F is basically the dynamics of the system in a matrix
    x_hat.noalias() = F * x_hat;
    // P_k+1 = F * P_k * FT + Q. Basically how correlated uncertainity evolves + process noise
    Eigen::Matrix<float, 3, 3> FP;
    FP.noalias() = F * P;
    P.noalias() = FP * FT;
    P += Q;
}

void KalmanFilter::update(Eigen::Vector<float,2> z) {
     // this is the innovation ie the error
    // Eigen::Vector<float,2> y;
    y = z - H*x_hat;

    y[0] = wrapAngle180(y[0]);

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

    x_hat[0] = wrapAngle180(x_hat[0]);

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

void KalmanFilter::update_timestep(float dt) {
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;
    const float dt5 = dt4 * dt;
    /*
    state transition matrix. Easy to see how it controls dynamics for phi, omega, alpha

    damping: 
    basically says the acceleration is inversely associated with the speed (damped oscillator)

    acceleration passthrough:
    In my opinion pretty important as its value determines how correlated each alpha is to the previous
    higher values here means the acceleration is thought to be more similar before while lower is
    that the noise is more random/white. This comes up more in the other filter.
    */
    F << 1.0f, dt,  0.5f*dt2,
         0.0f, 1.0f, dt,
         0.0f, damping, accel_passthrough;
    FT = F.transpose();

    /*
    process covariance matrix:
    Clearly the noise of every variable (phi, omega, alpha) are correlated.

    Process noise specifically is very dependent on dt (it's integrated so dt matters) while measurement noise is constant.
    However, while decreasing dt will decrease process covariance Q, it will update state covariance (P) more often meaning 
    the net uncertainty may stay the same. Process noise is essentially the white noise associated with the acceleration of the system
    so very important. It basically is the number that represents everyhting the model doesn't account for like
    wind, torques from your hand moving the IMU, gyro bias, etc. Small normally distributed alpha pertubations (jerks)
    are what this measures so if you give it an added 5 N*m twist, that needs to be covered by this variance for the
    model to perfom ideally. Remember var_process a variance so the standard deviation is its square root. Q, then,
    essentially represents the covariance of the error in the model at each timestep. So how much you trust the model
    */

    Q.setZero();
    Q(0,0) = dt5/20.0f; Q(0,1) = dt4/8.0f; Q(0,2) = dt3/6.0f;
    Q(1,0) = dt4/8.0f; Q(1,1) = dt3/3.0f; Q( 1,2) = dt2/2.0f;
    Q(2,0) = dt3/6.0f; Q(2,1) = dt2/2.0f; Q(2,2) = dt;
    Q *= var_process; // multiple by the factor 
}

Eigen::Vector<float,2> meas_vector(float roll_heading, float roll_velocity) {
    static constexpr bool CONVERT_TO_RADIANS = true;

    if (CONVERT_TO_RADIANS) {
        constexpr float DEG2RAD = 0.01745329252f; // pi / 180
        roll_heading *= DEG2RAD;
        roll_velocity *= DEG2RAD;
    }

    return Eigen::Vector<float,2>{roll_heading, roll_velocity};
}
