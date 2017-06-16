#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

double cte_sum = 0.;
double cte_prev = -1.;

void PID::Init(double Kp, double Ki, double Kd) {

  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  PID::p_error = 0.;
  PID::i_error = 0.;
  PID::d_error = 0.;
}

void PID::UpdateError(double cte) {

  PID::p_error = -Kp*cte;

  if(cte_prev == -1.){

    PID::d_error = 0.;
  }else{

    double cte_diff = cte - cte_prev;
    PID::d_error = -Kd*cte_diff;
  }

  cte_prev = cte;

  cte_sum += cte;
  PID::i_error = -Ki*cte_sum;

}

double PID::TotalError() {

  return (PID::p_error+ PID::d_error+ PID::i_error);
}
