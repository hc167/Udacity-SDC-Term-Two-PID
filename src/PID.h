#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  // Errors, index 0 is p, 1 is d, 2 is i for PID control
  double error[3];


  // Coefficients
  // index 0 is p, 1 is d, 2 is i for PID control
  double K[3];

  // for twiddle
  double dp[3];

  bool is_twiddle;
  bool first_update;
  double twiddle_total;
  int index;
  int state;
  int sample;
  int iteration;
  double best_err;
  double err_squ;

  std::vector<double> lowpass;

  PID();
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp = 0, double Ki = 0, double Kd = 0);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void twiddle();

};

#endif /* PID_H */
