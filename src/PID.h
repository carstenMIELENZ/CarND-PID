#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;
  double d_error_prev;

  // init
  bool   init_; 

  // tuning - twiddle variables
  bool   tuning_en_;

  double error_;
  double best_error_;
  double tuning_exit_;
  double tuning_p[3];
  double tuning_pd[3];
  int    tuning_state;  
  int    index_;  

  bool   tuning_verbose;

  long long steps_;
  long long tuning_step_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

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

  /*
  * Setup Twiddle 
  */
  void EnableTuning(double t_p[], double t_pd[], double t_exit, double t_steps, bool t_verbose);

  /*
  * Run Twiddle.
  */
  bool RunTwiddle(double cte);

  

};

#endif /* PID_H */
