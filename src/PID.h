#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;


  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double previous_Kp;
  double previous_Ki;
  double previous_Kd;

  double dkp;
  double dki;
  double dkd;


  double previous_dkp;
  double previous_dki;
  double previous_dkd;
  //double dp;
  
  int period_count;
  int period_index;

  double previous_cte;
  double period_error;
  double twiddle_best_err=1000.0;
  //double total_error;

  double tolerance;

  int buffer_size;
  int buffer_index;
  double buffer[];

  bool is_twiddle_on;
  int twiddle_mode;
  int twiddle_param_switch;
  int twiddle_count;




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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle();


};

#endif /* PID_H */
