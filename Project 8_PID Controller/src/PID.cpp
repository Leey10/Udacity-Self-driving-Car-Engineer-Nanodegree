
#include <cstdlib> 
#include "PID.h"
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */


//PID::~PID() {}

PID::PID(double Kp_, double Ki_, double Kd_) {
  
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  this->prev_cte = 0.0;
  this->int_cte = 0.0;
  
}  

void PID::UpdateError(double cte) {
  this->diff_cte = cte - this->prev_cte;
  this->int_cte += cte ;
  this->prev_cte = cte;
}

double PID::Control_out(double cte) {
  
  double u;
  u = - ( this->Kp * cte + this->Ki * int_cte + this->Kd * diff_cte );
  if (abs(u > 0.95)) {
    u = - ( this->Kp * cte + this->Kd * diff_cte );
  }
  if (abs(u > 0.95)) {
    u = u>0? 0.95: -0.95; 
  }
  return u;
  
}