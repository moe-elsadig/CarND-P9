#include "PID.h"
#include <iostream>
#include <math.h>

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

void PID::Twiddle(double cte, double speed, double angle) {

  // Twiddle tolerance
  double tol = 0.2;

  // increments
  double dp [3] = {1., 1., 1.};

  // init sum of increments
  double dp_sum = 0.;
  for(int i=0; i<3; i++){ dp_sum += dp[i]; }

  // coefficients
  double p [3] = {PID::Kp, PID::Ki, PID::Kd};

  // init best error
  double best_err = run_err(p, cte, speed, angle);
  cout << "best_err\t" << best_err << endl;

  // loop counter
  int iter = 0;

  cout << "Twiddling...\n\t*******\nPlease wait..." << endl;

  while(dp_sum > tol){

    for(int i=0; i<3 ; i++){

      // add the respective increment
      p[i] += dp[i];

      // get the current error
      double curr_err = run_err(p, cte, speed, angle);

      // check if the new error is better
      if(curr_err < best_err){

        // if yes, continue the positive increment
        best_err = curr_err;
        dp[i] *= 1.1;
      }else{

        // if not, reverse the positive increment and try a negative increment
        p[i] -= 2*dp[i];

        // obtain a new current error
        curr_err = run_err(p, cte, speed, angle);

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

    for(int i=0; i<3; i++){ dp_sum += dp[i]; }

    if(iter < 10){
    cout << "dp_sum\t" << dp_sum << endl;
    }
    // increment the step counter
    iter += 1;

  }

  // reassign the new values to the global coefficients' variables
  PID::Kp = p[0];
  PID::Ki = p[1];
  PID::Kd = p[2];
}



double PID::run_err(double p_[], double cte, double speed, double angle){

  double err_ = 0.;
  double prev_cte_ = cte;
  double i_error_ = 0.;
  int n_ = 100;
  double cte_ = cte;

  double tol_ = 0.001;
  double max_steering_angle= M_PI / 4.0;
  double length_ = 4.0;
  double speed_ = speed;
  double angle_ = angle;

  if(speed_<4.){
    speed_ = 4.;
  }
  // simulate run()
  for(int i=0; i<n_*2; i++){

    double p_error_ = cte_;
    double d_error_ = cte_ - prev_cte_;
    i_error_ += cte_;
    prev_cte_ = cte_;
    double steer_ = -p_[0]*p_error_ -p_[2]*d_error_ -p_[1]*i_error_;

    // simulate move
    if(steer_ > max_steering_angle){
        steer_ = max_steering_angle;}
    if(steer_ < -max_steering_angle){
        steer_ = -max_steering_angle;}
    if(speed_ < 0.0){
        speed_ = 0.0;}

    double turn_ = tan(steer_)*speed_/length_;
    double abs_turn = turn_;
    if(abs_turn < 0){ abs_turn *= -1;}

    if(abs_turn < tol_){

      // straight line motion
      cte_ += speed_*cos(angle_);
      angle_ = fmod((angle_ + turn_),(2.0*M_PI));
    }else{

      // bicycle motion model assumed
      double radius_ = speed_/turn_;
      double cy_ = cte_ + cos(angle_)*radius_;
      angle_ = fmod((angle_ + turn_),(2.0*M_PI));
      cte_ += cy_ - cos(cte_)*radius_;
    }

    if(i >= n_){

      err_ += pow(cte_, 2);
    }
  }
  return err_/n_;
}



















//END
