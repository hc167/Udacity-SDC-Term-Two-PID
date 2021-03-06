#include "PID.h"
#include <iostream>
#include <math.h>
#include <numeric>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->K[0] = Kp;
  this->K[1] = Kd;
  this->K[2] = Ki;

  if (K[0] == 0 && K[1] == 0 && K[2] == 0){
    is_twiddle = true;
  }
  else{
    is_twiddle = false;
  }

  error[0] = error[1] = error[2] = 0;
  twiddle_total = 0.05;

  dp[0] = dp[1] = dp[2] = 1;//0.3;
  index = state = 0;
  sample = 1000;
  iteration = 0;
  first_update = true;
}

void PID::UpdateError(double cte) {
  double diff = cte - error[0];

  lowpass.push_back(diff);

  if (lowpass.size() > 3){
    lowpass.erase(lowpass.begin());
  }

  error[1] = std::accumulate( lowpass.begin(), lowpass.end(), 0.0)/lowpass.size();
  error[0] = cte;
  error[2] += cte;

  std::cout<<"K value: "<<K[0]<<" : "<<K[1]<<" : "<<K[2]<<std::endl;

  if ( is_twiddle ){
    twiddle();
  }
}

double PID::TotalError() {

  double sum=0;
  for(int i=0; i<3; ++i)
    sum += -K[i] * error[i];

  return sum;
}

void PID::twiddle() {

  std::cout<<"errors : "<<error[0]<<" : "<<error[1]<<" : "<<error[2]<<std::endl;
  std::cout<<"dp values : "<<dp[0]<<" : "<<dp[1]<<" : "<<dp[2]<<std::endl;
  std::cout<<"index : "<<index<<" state: "<<state<<" best err: "<<best_err<<std::endl;
  
  if ( state ==0 && (dp[0] + dp[1] + dp[2] < twiddle_total) && index == 0 ){
    is_twiddle = false;
    return;
  }

  iteration++;

  if (iteration < sample){
    err_squ = 0;
    return;
  }
  else{
    double square = error[0] * error[0];
    square *= square; // square it again to speed up the computation a little
    err_squ += square;

    // Let's see if we can speed up the twiddle process by checking if err_squ is already larger than best_err
    if (state == 1 && err_squ > best_err){
      K[index] -= 2*dp[index];
      state = 2;
      iteration = 0;
      error[2] = 0;
      return;
    }
    else if (state == 2 && err_squ > best_err){
      K[index] += dp[index];
      dp[index] *=0.9;
      index = (index+1) % 3;
      state = 0;
      iteration = 0;
      error[2] = 0;
      return;
    }
    
    if (iteration < 2*sample)
      return;
  }

  iteration = 0;
  error[2] = 0;

  if (first_update){
    first_update = false;
    best_err = err_squ;
  }


  switch(state){
  case 0:
    K[index] += dp[index];
    state = 1;
    return;
    break;
  case 1:
    if( err_squ < best_err){
      best_err = err_squ;
      dp[index] *= 1.1;
      index = (index+1) % 3;
      state = 0;
      return;
    }
    else{
      K[index] -= 2*dp[index];
      state = 2;
      return;
    }
    break;
  case 2:
    if (err_squ < best_err){
      best_err = err_squ;
      dp[index] *= 1.1;
    }
    else{
      K[index] += dp[index];
      dp[index] *=0.9;
    }
    index = (index+1) % 3;
    state = 0;
    break;
  }
}
