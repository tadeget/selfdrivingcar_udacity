#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char  *argv[]) {
  uWS::Hub h;

  PID pid_steer;
  PID pid_vel;
  double steer_prev=0;
  /**
   * TODO: Initialize the pid variable.
   */
  double kp_s,kd_s,ki_s;
  double kp_v,kd_v,ki_v;
  if(argc > 2)
  {
  	 kp_s = atof(argv[1]); //-1.0;
  	 kd_s = atof(argv[2]);
  	 ki_s = atof(argv[3]);
  	 kp_v = atof(argv[4]); //-1.0;
 	 kd_v = atof(argv[5]);
  	 ki_v = atof(argv[6]);
  }
  else
  {
     kp_s = -0.15; //0.1;
  	 kd_s = -3;// -3
  	 ki_s = -0.0002; //-0.0002
  	 kp_v = -0.2; //-1.0;
 	 kd_v = -3;
  	 ki_v = -0.0003;
  }
  pid_steer.Init(kp_s,ki_s,kd_s);
  pid_vel.Init(kp_v,ki_v,kd_v);
  h.onMessage([&pid_steer, &pid_vel, &steer_prev](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          double speed_ref = 35;
          double speed_err = speed - speed_ref;
          double throttle_val;
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
          // limint the value of steering angle
          if(steer_value > 1) 
            steer_value = 1.0;
          else if(steer_value < -1) 
            steer_value = -1.0;
          
          pid_vel.UpdateError(speed_err);
          throttle_val = pid_vel.TotalError();
          // decelerate the vehicel so the steering will be easier
         if(fabs(cte) > 0.8)
            throttle_val = 0.01;
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = 0.8*steer_value+0.2*steer_prev;
          msgJson["throttle"] = throttle_val;
          steer_prev = steer_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}