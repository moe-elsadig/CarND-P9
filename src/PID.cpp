#include "PID.h"
#include "PID.h"
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

// declaring shared variables
// holds initialization status
bool is_initialised = false;

// holds the previous cte value for error update
double prev_cte;

void PID::Init(double Kp, double Ki, double Kd) {

  // initializes Coefficients to the values passed in
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  // initializes the error values to 0.
  PID::p_error = 0.;
  PID::i_error = 0.;
  PID::d_error = 0.;

}

void PID::UpdateError(double cte) {

  if(!is_initialised){

    prev_cte = cte;
    is_initialised = true;
  }

  // assign the error values based on the given cte
  PID::p_error = cte;
  PID::i_error += cte;
  PID::d_error = cte-prev_cte;
  prev_cte = cte;

}

double PID::TotalError() {

  return 0.;
}
