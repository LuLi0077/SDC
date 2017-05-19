#ifndef PID_H
#define PID_H

class PID {

public:
  /*
  * Errors
  */
  double int_cte;
  double pre_cte;
  double diff_cte;
  double pre_sp;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

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
  * Update the speed error variables given target speed.
  */
  void UpdateSpeed(double speed);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

};

#endif /* PID_H */
