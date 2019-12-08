#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID(double Kp_, double Ki_, double Kd_);

  /**
   * Destructor.
   */
  //virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  //void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * update the maneuver 
   */
  double Control_out(double cte);

 private:
  /**
   * PID Errors
   */
  double cte;
  double int_cte;
  double diff_cte;
  
  double prev_cte;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H