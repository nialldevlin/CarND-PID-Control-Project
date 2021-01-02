#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  p[0] = Kp_;
  p[1] = Ki_;
  p[2] = Kd_;

  dp[0] = Kp_ * 0.2;
  dp[1] = Ki_ * 0.2;
  dp[2] = Ki_ * 0.2;
}

double PID::SteeringAngle(double cte) {
  if (!last_cte_init) {
    last_cte = cte;
  }
  sum_cte += cte;
  double angle = (-1 * p[0] * cte) - (p[1] * sum_cte) - (p[2] * (cte - last_cte));
  last_cte = cte;

  if (angle > 1) {
    angle = 1;
  } else if (angle < -1) {
    angle = -1;
  }

  
  return angle;
}

void PID::UpdateParams(double cte) {
  std::cout << "1. Kp: " << p[0] << " Ki: " << p[1] << " Kd: " << p[2] << std::endl;
  if(!best_cte_init) {
    best_cte = cte;
  }
  if ((dp[0] + dp[1] + dp[2]) > twiddle_tolerance) {
    switch( t_state ) {
      case 0:
        best_cte = cte;
        p[t_id] += dp[t_id];
        t_state = 1;
        break;
      case 1:
        if (cte < best_cte) {
          best_cte = cte;
          dp[t_id] *= 1.1;
          t_id  = (t_id + 1) % 3;
          p[t_id] += dp[t_id];
          t_state = 1;
        } else {
          p[t_id] -= 2 *dp[t_id];
          if (p[t_id] < 0) {
            p[t_id] = 0;
            t_id  = (t_id + 1) % 3;
          }
          t_state = 2;
        }
        break;
      case 2:
        if (cte < best_cte) {
          best_cte = cte;
          dp[t_id] *= 1.1;
          t_id  = (t_id + 1) % 3;
          p[t_id] += dp[t_id];
          t_state = 1;
        } else {
          p[t_id] += dp[t_id];
          dp[t_id] *= 0.9;
          t_id  = (t_id + 1) % 3;
          p[t_id] += dp[t_id];
          t_state = 1;
        }
        break;
    }
  }
  std::cout << "2. Kp: " << p[0] << " Ki: " << p[1] << " Kd: " << p[2] << std::endl;
}