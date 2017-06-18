#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <ctime>
#include <fstream>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
	//pid.Init(0.2,3.0,0.004);
	pid.Init(0.18, 3.3, 0.005); // initialisation of PID
	//pid.Init(0.2, 0.3, 0.004); // initialisation of PID
	pid.TwiddleEnable = false; // by default, this is set to true, so override manually for turning this off
	const int max_iterations = 600; // controls how long to run for before resetting the simulation to the start

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::ofstream myfile;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          pid.current_time = clock(); // get the current time
          double dt = (pid.current_time - pid.previous_time) / CLOCKS_PER_SEC; //utilise CPU clocks to calculate delta time
          pid.previous_time = pid.current_time; // update previous time

          pid.UpdateError(cte, dt); // update the PID errors considering the cross track error, and delta time
          steer_value = pid.TotalError(); // set the steering value to the total error
          if(steer_value > 1) // limit the steering values to within 1 to -1
            steer_value = 1;
          if(steer_value < -1)
            steer_value = -1;

          pid.StoreError(cte); // store the error, and update the iteration number

          if(pid.TwiddleEnable)
          {
            if((abs(cte) > 2.5 ) || (pid.num_iterations > max_iterations))
            {
              pid.TwiddleSticks();
              /*//Utilised for writing debug text files for plotting
              myfile.open("../storage.txt", std::ios_base::app);
              myfile << pid.stored_error << ";";
              myfile << pid.best_error << ";";
              myfile << pid.Kp << ";";
              myfile << pid.Ki << ";";
              myfile << pid.Kd << ";";
              myfile << pid.dKp << ";";
              myfile << pid.dKi << ";";
              myfile << pid.dKd << ";";
              myfile << pid.p_error << ";";
              myfile << pid.i_error << ";";
              myfile << pid.d_error << "\n";
              myfile.close();
	      */
              pid.Restart(ws);
              pid.num_iterations = 0;
              pid.stored_error = 0;
              pid.p_error = 0;
              pid.i_error = 0;
              pid.d_error = 0;
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.5;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
    else
    {
      // Manual driving
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
