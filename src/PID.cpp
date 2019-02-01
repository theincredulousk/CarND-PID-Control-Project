#include "PID.h"
#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID()
{
  Init(0.0,0.0,0.0);
}

PID::PID(double Kp_, double Ki_, double Kd_)
{
  Init(Kp_, Ki_, Kd_);
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) 
{
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  prev_cte = 0.0;
  best_cte = 0.0;
  worst_cte = 0.0; 
}

void PID::UpdateError(double CrossTrackError) {
  p_error = CrossTrackError;
  i_error += CrossTrackError;
  d_error = CrossTrackError - prev_cte;
  
  prev_cte = CrossTrackError;

  if(CrossTrackError < best_cte)
  {
    std::cout << "(DEBUG) New Minimum Crosstrack Error: " << CrossTrackError << std::endl;
    best_cte = CrossTrackError;
  }
  else if(CrossTrackError > worst_cte)
  {
    worst_cte = CrossTrackError;
    std::cout << "(DEBUG) New Maximum Crosstrack Error: " << CrossTrackError << std::endl;
  }
}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

double PID::Steer(double CrossTrackError)
{
    double steering_angle = -(Kp * p_error + Kd * d_error + Ki * i_error);
    bool bounds_exceeded(false);

    if(steering_angle > 1)
    { 
      steering_angle = 1;
    }
    else if(steering_angle < -1)
    {
      steering_angle = -1;
    }

    if(bounds_exceeded)
    {
      std::cout << "(DEBUG) Bounds exceeded in steering correction calculation" << std::endl;
    }

    
    return steering_angle;
}