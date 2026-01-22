// dsp.h

#pragma once

#include <Eigen.h>

// kalmanFilter: [state] -> [state]
// does the kalman filter :)
float* kalmanFilter(float* estimatedState);


// getAngularAcceleration: float -> float
// takes the derivative of the filtered
// angular velocity signal to 
