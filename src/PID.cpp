#include "PID.h"
#include <algorithm>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Kd = Kd;
  PID::Ki = Ki;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  // Previous cte.
  prev_cte = 0.0;

  // Counters.
  iter = 0;
  errorSum = 0.0;
  minError = std::numeric_limits<double>::max();
  maxError = std::numeric_limits<double>::min();
}

void PID::UpdateError(double cte) {

  p_error = cte;
  d_error = cte - prev_cte;
  i_error += cte;

  prev_cte = cte;
  errorSum += cte;
  iter++;

  if(cte > maxError){
    maxError = cte;
  }
  if(cte < minError){
    minError = cte;
  }
}

double PID::TotalError() {
  t_error = (Kp*p_error + Ki*i_error + Kd*d_error);
  return t_error;
}

double PID::AverageError() {
  return errorSum/iter;
}

double PID::MinError() {
  return minError;
}

double PID::MaxError() {
  return maxError;
}


