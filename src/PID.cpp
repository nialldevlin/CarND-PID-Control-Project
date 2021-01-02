#include "PID.h"
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  this->p[0] = Kp_;
  this->p[1] = Ki_;
  this->p[2] = Kd_;

  this->dp[0] = Kp_ * 0.1;
  this->dp[1] = Ki_ * 0.1;
  this->dp[2] = Ki_ * 0.1;
}

double PID::SteeringAngle(double cte) {
  if (!this->last_cte_init) {
    this->last_cte = cte;
  }
  this->sum_cte += cte;
  double angle = (-1 * this->p[0] * cte) - (this->p[1] * sum_cte) - (this->p[2] * (cte - last_cte));
  this->last_cte = cte;

  if (angle > 1) {
    angle = 1;
  } else if (angle < -1) {
    angle = -1;
  }

  std::cout << "Kp: " << p[0] << " Ki: " << p[1] << " Kd: " << p[2] << std::endl;
  return angle;
}

void PID::UpdateParams(double cte) {
  if(!this->best_cte_init) {
    this->best_cte = cte;
  }
  if ((dp[0] + dp[1] + dp[2]) > this->twiddle_tolerance) {
    switch( this->t_state ) {
      case 0:
        p[this->curr_param] += dp[this->curr_param];
        t_state = 1;
        break;
      case 1:
        if (cte < this->best_cte) {
          this->best_cte = cte;
          dp[this->curr_param] *= 1.1;
          curr_param  = (curr_param + 1) % 3;
          t_state = 0;
        } else {
          p[this->curr_param] -= 2 *dp[this->curr_param];
          t_state = 2;
        }
        break;
      case 2:
        if (cte < this->best_cte) {
          this->best_cte = cte;
          dp[this->curr_param] *= 1.1;
        } else {
          p[this->curr_param] += dp[this->curr_param];
          dp[this->curr_param] *= 0.9;
        }
        t_state = 0;
        curr_param  = (curr_param + 1) % 3;
        break;
    }
  }
}