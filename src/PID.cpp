#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->K[0] = Kp;
  this->K[1] = Ki;
  this->K[2] = Kd;

  error[0] = error[1] = error[2] = 0;
}

void PID::UpdateError(double cte) {
  error[2] = cte - error[0];
  error[0] = cte;
  error[1] += cte;
}

double PID::TotalError() {
  std::cout<<"total error"<<std::endl;

  double sum=0;
  for(int i=0; i<3; ++i)
    sum += K[i] * error[i];

  return sum;
}
