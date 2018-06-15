#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Gain Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  /*
  * learning rate coefficients
  */
  double lrp;
  double lri;
  double lrd;
  
  /*
  * tuning parameters
  */
  std::vector<double> p;
  std::vector<double> dp;
  std::vector<double> dpInit;
  
  /*
  *state variables
  */
  int tw_state;
  int tw_step;
  
  /*
  *twiddle parameters
  */
  double best_error;
  double accumulated_error;
  double tolerance;
  bool twiddle_complete;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double lrp, double lri, double lrd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double error);

  /*
  * Calculate the total PID error.
  */
  void Twiddle(double accumulated_error);
  
  /*
  * Optimise the PID gain coefficients.
  */
  double TotalError();
  
  /*
  *restart simulator
  */
  void Reset();
};

#endif /* PID_H */
