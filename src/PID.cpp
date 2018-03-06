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
  this->K[1] = Kd;
  this->K[2] = Ki;

  error[0] = error[1] = error[2] = 0;
  cal = false;
  num_elements = 20;

  state = 0;
  total_err.clear();
  index = 0;
}

void PID::UpdateError(double cte) {
  error[1] = cte - error[0];
  error[0] = cte;
  error[2] += cte;

      if (cal && abs(cte) > 0.5){
        cal = false;
        state = 0;
        total_err.clear();
        index = 1;
      }


  if (!cal){
    double square = cte*cte;
    double upscale = 1.1;
    double downscale = 0.9;

    if (total_err.size() < num_elements){
      total_err.push_back(square);
    }
    else if (total_err.size() == num_elements){
      double err;


      std::cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
      switch(state){
      case 0:
	best_err = getTotalError();
	if (abs(K[index]) < 0.8){
	  K[index] *= upscale;
	  state = 1;
	}
	else{
	  K[index] *= downscale;
	  state = 3;
	}
	break;
      case 1:
	err = getTotalError();
	if (err < best_err && abs(K[index]) < 0.8){
	  best_err = err;
	  K[index] *= upscale;
	  state = 2;
	}
	else{
	  K[index] /= upscale;
	  K[index] *= downscale;
	  state =3;
	}
	break;
      case 2:
	err = getTotalError();
	if (err < best_err && abs(K[index]) < 0.8){
	  best_err = err;
	  K[index] *= upscale;
	}
	else{
	  K[index] /= upscale;
	  state = 0;
	  index++;
	  if (index > 2)
	    cal = true;
	}
	break;
      case 3:
	err = getTotalError();
	if (err < best_err){
	  best_err = err;
	  K[index] *= downscale;
	}
	else{
	  K[index] /= downscale;
	  state = 0;
	  index++;
	  if (index > 2 )
	    cal = true;
	}
	break;
      }
      total_err.clear();
    }
  }
}

double PID::TotalError() {
  std::cout<<"total error: "<<K[0]<<" : "<<K[1]<<" : "<<K[2]<<" : "<<cal<<" : "<<index<<" : "<<state<<std::endl;

  double sum=0;
  for(int i=0; i<3; ++i)
    sum += K[i] * error[i];

  return sum;
}

double PID::getTotalError() {
  double sum = 0;
  for (std::vector<double>::iterator it = total_err.begin(); it != total_err.end(); ++it)
    sum += *it;
 
  return sum;
}
