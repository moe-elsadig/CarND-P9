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

void PID::Twiddle(double angle) {


  double run_err(double[] p_){


    return 0.
  }


  // Twiddle tolerance
  double tol = 0.2;

  // increments
  dp = [1., 1., 1.];

  // init sum of increments
  double dp_sum = 0.;
  for(int i=0; i<dp.sizeof(); i++){ dp_sum += dp[i]; }

  // coefficients
  p = [PID::Kp, PID::Ki, PID::Kd];

  // init best error
  double best_err = run_err(p);

  // loop counter
  int iter = 0;

  cout << "Twiddling...\n\t*******\nPlease wait..." << endl;

  while(dp_sum > tol){

    for(int i=0; i<3 ; i++){

      // add the respective increment
      p[i] += dp[i];

      // get the current error
      double curr_err = run_err(p);

      // check if the new error is better
      if(curr_err < best_err){

        // if yes, continue the positive increment
        best_err = curr_err;
        dp[i] *= 1.1;
      }else{

        // if not, reverse the positive increment and try a negative increment
        p[i] -= 2*dp[i];

        // obtain a new current error
        curr_err = run_err(p);

        // check if the new error is better
        if(curr_err < best_err){

          // if yes, continue the negative increment
          best_err = curr_err;
          dp[i] *= 1.1;
        }else{

          // if not, continue the negative increment but decrease it's value
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }

    // increment the step counter
    iter += 1;
  }

  // reassign the new values to the global coefficients' variables
  PID::Kp, PID::Ki, PID::Kd = p;
}




















//END
