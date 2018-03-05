#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  // index 0 is p, 1 is i, 2 is d for PID control
  double error[3];


  /*
  * Coefficients
  */ 
  // index 0 is p, 1 is i, 2 is d for PID control
  double K[3];

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

};

#endif /* PID_H */
