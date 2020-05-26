#pragma once

#include <cmath>

// Encapsulates a first order filter with rate limiting
// Can be used to extract a first order time history response.
// Major assumption of use is that the object is called at a fixed sample time
//
// Atributes:
//  dt_: sample time in seconds
//  tau_: time constant in seconds
//  state_: internal state of the filter
//  cmd_: Command value
//  alpha_: derived constant from sample time and time constant
//
//  This filter assumes the following equations:
//  - y[i] is the current output, current state
//  - y[i-1] is the previous output, previous state
//  - u[i] is the current command
//
//      y[0], u[0] = x0
//
//      y[i] =  (1 - alpha) * y[i-1] + alpha * u[i]
//
//      alpha = dt / (tau + dt)
//
//      If y[i] - y[0] exeeds the rate limit, then y[i] is limited to an
//      increment cooresponding to the rate limit
class RateLmtdFirstOrderFilter {
 private:
  float dt_{0.001f};
  float tau_;
  float state_;
  float cmd_;
  float alpha_;
  float rate_limit_;

 public:
  // Constructor
  RateLmtdFirstOrderFilter(float dt, float tau, float x0, float rate_limit)
      : dt_(dt),
        tau_(tau),
        state_(x0),
        cmd_(x0),
        alpha_(dt/(tau+dt)),
        rate_limit_(rate_limit) {}

  RateLmtdFirstOrderFilter() = default;

  void setCommand(float cmd) {
    cmd_ = cmd;
  }

  float runTimestepAndGetOutput() {
    runTimestep();
    return state_;
  }

void runTimestep() {
  // Convenience variables
  float a = alpha_;
  float y_prev = state_;
  float u = cmd_;

  // Update equation
  float y = (1-a)*y_prev + a*u;

  // Enforce rate limit
  float rate = (y-y_prev)/dt_;
  if (std::fabs(rate) > rate_limit_) {
    if (rate > 0.0f) { 
      y = y_prev + rate_limit_*dt_;
    } else {
    y = y_prev - rate_limit_*dt_;
    } 
  }
  state_ = y;
}


float getOutput() const {
  return state_;
}


void resetState(float x0) {
  state_ = x0;
}

};
