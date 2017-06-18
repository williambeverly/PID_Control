#include "PID.h"
#define MIN_NUMBER_ITERATIONS 100
#define TOLERANCE_DELTA 0.1 // change as required
#define DEBUG 0 // debug flag 1 = On, 0 = Off

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	current_time = 0;
  previous_time = 0;
  total_error = 0;
  num_iterations = 0;
  stored_error = 0;

  dKp = 0.1 * Kp; // set relative to proportional gain
  dKi = 0.1 * Ki; // set relative to integral gain
  dKd = 0.1 * Kd; // set relative to deriv gain

  best_error = 1e10;
  glob_idx = 0;
  ReturnStage_1 = true;
  ReturnStage_2 = true;
  FirstTwiddle = true;
  TwiddleEnable = true;
}

void PID::UpdateError(double cte, double dt) {
	d_error = (cte - p_error) / dt;
	p_error = cte;
  i_error += cte * dt;
}

double PID::TotalError() {
	total_error = -Kp * p_error - Kd * d_error - Ki * i_error; // using total_error for storage purposes
	return total_error;
}

void PID::StoreError(double cte)
{
  if(num_iterations > MIN_NUMBER_ITERATIONS)
    stored_error += abs(cte);

  num_iterations += 1;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws)
{
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


void PID::TwiddleSticks()
{
  double err = stored_error / (num_iterations - MIN_NUMBER_ITERATIONS); //simply waits 100 iterations to start recording
  double sum_delta_gain = 0; // declare the sum holder

  // store the first result as the best error
  if(FirstTwiddle)
  {
    best_error = err;
    FirstTwiddle = false;
  }

  // find the sum of the changes to the gain elements
  sum_delta_gain = dKp + dKi + dKd;

  if(DEBUG)
  {
    std::cout << " Delta sum " << sum_delta_gain << " error " << err << " best_err " << best_error << " glob_idx " << glob_idx << std::endl;
    std::cout << " gains: " << Kp << "," << Ki << "," << Kd << std::endl;
    std::cout << "dgains: " << dKp << "," << dKi << "," << dKd << std::endl;
  }



  if(sum_delta_gain > TOLERANCE_DELTA)
  {
      if(ReturnStage_1)
      {
        *RetrieveGain(glob_idx) += *RetrieveDelta(glob_idx); //updates the gain using pointers
        ReturnStage_1 = false;

        if(DEBUG)
        {
          std::cout << " Returning from Stage 1: " << " gains: " << Kp << "," << Ki << "," << Kd << std::endl;
          std::cout << "dgains: " << dKp << "," << dKi << "," << dKd << std::endl;
        }
        return;
      }
      else if(ReturnStage_2)
      {
        if(err < best_error)
        {
          best_error = err;
          *RetrieveDelta(glob_idx) *= 1.1; // update the delta parameter using pointer
          glob_idx = (glob_idx + 1) % 3; // update the global iterator and bound it
          ReturnStage_1 = true;

          if(DEBUG)
          {
            std::cout << " Returning from Stage 2a: " << " gains: " << Kp << "," << Ki << "," << Kd << std::endl;
            std::cout << "dgains: " << dKp << "," << dKi << "," << dKd << std::endl;
          }

          TwiddleSticks();
        }
        else {
            *RetrieveGain(glob_idx) -= 2 * (*RetrieveDelta(glob_idx));
            ReturnStage_1 = false;
            ReturnStage_2 = false;

            if(DEBUG)
            {
              std::cout << " Returning from Stage 2b: " << " gains: " << Kp << "," << Ki << "," << Kd << std::endl;
              std::cout << "dgains: " << dKp << "," << dKi << "," << dKd << std::endl;
            }
            return;
        }
      }
      else
      {
        if (err < best_error)
        {
          best_error = err;
          *RetrieveDelta(glob_idx) *= 1.1;
        }
        else
        {
          *RetrieveGain(glob_idx) += *RetrieveDelta(glob_idx);
          *RetrieveDelta(glob_idx) *= 0.9;
        }
        ReturnStage_1 = true;
        ReturnStage_2 = true;
        glob_idx = (glob_idx + 1) % 3; // update the global iterator and bound it

        if(DEBUG)
        {
          std::cout << " Returning from Stage 3: " << " gains: " << Kp << "," << Ki << "," << Kd << std::endl;
          std::cout << "dgains: " << dKp << "," << dKi << "," << dKd << std::endl;
        }

        TwiddleSticks();
      }
  }
  else
  {
    TwiddleEnable = false; // disables the Twiddle function when the tolerance has been reached
  }
}

// functions to return the correct gain according to the current index of the state machine
double * PID::RetrieveGain(int Gain_idx)
{
  if(Gain_idx == 0)
    return &Kp;

  else if(Gain_idx == 1)
    return &Ki;

  else
    return &Kd;
}

double * PID::RetrieveDelta(int Delta_idx)
{
  if(Delta_idx == 0)
    return &dKp;

  else if(Delta_idx == 1)
    return &dKi;

  else
    return &dKd;
}



