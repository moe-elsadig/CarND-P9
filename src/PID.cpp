#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

// shared/continuous variables
bool is_initialised = false;
int run_steps = 0;
double prev_cte;

void PID::Init(double Kp, double Ki, double Kd) {

  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  PID::p_error = 0.;
  PID::i_error = 0.;
  PID::d_error = 0.;
}

void PID::UpdateError(double cte) {

  // check whether this is the first step to set the prev_cte
  // proportional error for step 1 will be 0.
  if(!is_initialised){

    prev_cte = cte;
    is_initialised = true;
  }

  // run steps counter increment
  run_steps += 1;

  // update proportional error
  PID::p_error = cte;

  // update integral error
  PID::i_error += cte;

  // update differential error
  PID::d_error = cte - prev_cte;


}

double PID::TotalError() {

  return (0.);
}
