#ifndef PID_H
#define PID_H
#include <uWS/uWS.h>
#include <iostream>

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

  /*
  * Interal values
  */
  double current_time;
  double previous_time;
  double total_error;
  int num_iterations;
  double stored_error;

  /*
  * Twiddle values
  */
  double dKp;
  double dKi;
  double dKd;
  double best_error;
  unsigned int glob_idx; //global index
  bool ReturnStage_1;
  bool ReturnStage_2;
  bool FirstTwiddle;
  bool TwiddleEnable;
  double speed;


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
  void UpdateError(double cte, double dt);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Restarts the simulator
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  double * RetrieveGain(int Gain_idx);

  double * RetrieveDelta(int Delta_idx);



  /*
  * Stores the cumulative error
  */
  void StoreError(double cte) ;

  /*
  * Implements twiddle
  */
  void TwiddleSticks();


};

#endif /* PID_H */
