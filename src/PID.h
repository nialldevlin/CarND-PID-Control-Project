#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   * @param (steeringMin_, steeringMax_) Steering limits
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Calculate steering angle given CTE
   * @param cte The current cross track error
   * @output the steering angle to be sent to the car
   */
  double SteeringAngle(double cte);

  /**
   * Twiddle
   * @params cte The curretn cross track error
   */
  void UpdateParams(double cte);

 private:
  /**
   * PID Errors
   */
  double last_cte;
  double best_cte;
  double sum_cte;

  bool last_cte_init = false; //check if last cte has value
  bool best_cte_init = false; //check if best cte has value

  /**
   * PID Coefficients
   */ 
  double p[3];
  double dp[3];

  /**
   * Twiddle variables
   */
  double twiddle_tolerance = 0.00001;
  int t_id = 0; //which parameter twiddle modifies: 0 - p, 1 - i, 2 - d
  int t_state = 0;  //Which part of the algorithm runs each loop
};

#endif  // PID_H