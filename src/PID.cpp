#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double steeringMin_, double steeringMax_) {
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  this->dp = 0.05;
  this->di = 0.001;
  this->dd = 0.05;

  this->steeringMin = steeringMin_;
  this->steeringMax = steeringMax_;

}

double PID::SteeringAngle(double cte) {
  if (!this->last_cte_init) {
    this->last_cte = cte;
  }
  this->sum_cte += cte;
  double angle = -1 * this->Kp * cte - this->Ki * sum_cte - this->Kd * (cte - last_cte);
  this->last_cte = cte;
  return angle;
}

void PID::UpdateParams(double cte) {
  if ((this->dp + this->di + this->dd) > this->twiddle_tolerance) {
    
  }
}