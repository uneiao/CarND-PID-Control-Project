#ifndef PID_H
#define PID_H

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

  int current_round;
  int cur_p_idx;
  int max_round;
  double tol;
  double best_err;
  bool tuned;
  double dKp;
  double dKi;
  double dKd;
  int sub_round;
  double acc_err;
  int acc_n;
  int state;

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


  void Twiddle(double err);
  double Speed(double sp);
};

#endif /* PID_H */
