#include "PID.h"
#include <vector>
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double lrp, double lri, double lrd) 
{
  this -> p_error = 0.0;
  this -> i_error = 0.0;
  this -> d_error = 0.0;
  
  this -> tw_state = 0;
  this -> tw_step = 0;

  this -> p = {Kp, Ki, Kd};
  this -> dpInit  = { lrp, lri, lrd};
  this -> dp = dpInit;

  this -> best_error = 1e6;
}

void PID::UpdateError(double error) 
{
  d_error = error - p_error;
  p_error = error;
  i_error += error;
}

void PID::Twiddle(double accumulated_error)
{
  while (tw_step < 3)
  {
  // state 0 increase gain of factor under consideration and return to run car
    if (tw_state == 0)
    {
      p[tw_step] += dp[tw_step];
      tw_state += 1;
      //cout<<"State 0: p = "<< p[0]<< " " << p[1]<< " "<< p[2]<<endl;
      return;
    }
  // state 1 change probing values
  //if state 0 has resulted in a lower accumulated cross track error increase
  // gain probing value and return to state 0
    if (tw_state == 1)
    {
      if (accumulated_error < best_error)
      {
        best_error = accumulated_error;
        dp[tw_step] *= 1.1;
        tw_state = 0;
      }
   //if state 0 has not improved the accumulated cross track error decrease gain 
   //value to previous best value rerun car and move to state 2
      else
      {
        p[tw_step] -= 2*dp[tw_step];
        tw_state += 1;
        //cout<<"State 1: p = "<< p[0]<< " " << p[1]<< " "<< p[2]<<endl;
        return;
      }      
    }
    //state 2
    //modify magnitude of probing values    
    if (tw_state == 2)
    {
      if (accumulated_error < best_error)
      {
        best_error = accumulated_error;
        dp[tw_step] *= 1.01;
        tw_state = 0;
        tw_step += 1;
        cout<<"New probing values: "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
      }
      else
      {
        p[tw_step] += dp[tw_step];
        dp[tw_step] *= 0.09;
        tw_state = 0;
        tw_step += 1;
        cout<<"New probing values: "<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
      }
    }
  }
  
  tw_step = 0;
  double dpTot = dp[0] + dp[1] + dp[2];
//  cout<<"Tol: "<<tolerance<<" dptot: "<<dpTot<<endl;
  
  //initially7y I had this step in so that once the pid loop had been tuned to
  //specified tolerance the parameters could be increased so that the length of
  //track used for evaluation could be lengthend, the tolerance could be tightened
  //this is quite fidely and helped with intial tuning but became a hinderance later
  //on so I've just commneted the whole thing out
  if (dpTot < tolerance)
  {
    dp[0] = dpInit[0];
    dp[1] = dpInit[1];
    dp[2] = dpInit[2];
    twiddle_complete = true;
    return;
  }
}

double PID::TotalError() 
{
  return p[0]*p_error + p[1]*i_error + p[2]*d_error;
}

void PID::Reset()
{
  this -> p_error = 0.0;
  this -> i_error = 0.0;
  this -> d_error = 0.0;
}

