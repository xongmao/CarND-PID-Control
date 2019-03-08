#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double prev_cte_, double int_cte_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
   prev_cte = prev_cte_;
   int_cte = int_cte_;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   double diff_cte = cte - prev_cte;
   p_error = -Kp * cte;
   i_error = -Ki * int_cte;
   d_error = -Kd * diff_cte;

}
