#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  double max_vel = 49.5;
  double inc_vel = 0.224; //5 m/s

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

 double lane = 1;
 double ref_vel = 0;
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&inc_vel,&max_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
                
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          int prev_size = previous_path_x.size();

          //std::cout << "prev_size startedmake"<<std::endl;

          
          // sensor fusion
          if(prev_size >0)
          {
            car_s = end_path_s;
          }
          // prediction
          bool car_ahead = false;
          bool car_left  = false;
          bool car_right = false;
          int car_lane;
          for(int i=0; i< sensor_fusion.size(); i++)
          {
            // car in lane
            float d = sensor_fusion[i][6];
            if(d>=0 && d<=4)
              car_lane = 0;
            else if (d >4 && d <=8)
              car_lane = 1;
            else if(d >8 && d <=12)
              car_lane = 2;
            else
              continue;
            //if(d < (2+4*lane+2) && d > (2+4*lane-2))
            //{
             double vx = sensor_fusion[i][3];
             double vy = sensor_fusion[i][4];
             double check_speed = sqrt(vx*vx+vy*vy);
             double check_car_s = sensor_fusion[i][5];
              // Estimate car s position after executing previous trajectory.
             check_car_s+=((double)prev_size*0.02*check_speed);
              
            if(car_lane == lane)
            {
               car_ahead |= (check_car_s - car_s) < 30 && check_car_s > car_s;
            }
            else if( (car_lane - lane) == -1)
            {
              car_left |= fabs(check_car_s - car_s) < 30;
            }
            else if( (car_lane - lane) == 1)
            { 
              car_right |= fabs(check_car_s - car_s) < 30;
            }
        
          }
          
          // behaviour planning
          if(car_ahead)
          {
           if(lane !=0 && !car_left)
           {
             lane -= 1;
            // std::cout <<"Lane changed "<< lane << std::endl;
           }
           else if (lane !=2 && !car_right)
           {
             //std::cout <<"car ahead right lane change "<< ref_vel << std::endl;
             lane +=1;
           }
           else
           {
             //std::cout <<"car ahead  decrease velocity "<< ref_vel << std::endl;
             ref_vel -=inc_vel;  // 5 m/smam
           }
          }
          else
          {
            // std::cout <<"free  "<< ref_vel << std::endl;
            if( ref_vel < max_vel)
            	 ref_vel += inc_vel;  // 5 m/smam
          }
           
          // trajecorty generation 
          if(prev_size < 2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
            //std::cout << "car_x "<< car_x <<"car_y " << car_y <<std::endl;
            //std::cout << "ptsx "<< ptsx[0] <<"ptsu[0[] " << ptsy[0] <<std::endl;

            
          } else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size -2];
            double ref_y_prev = previous_path_y[prev_size -2];
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            
          }
          
          // add frenet waypoints
          vector<double> next_wp0= getXY(car_s+30,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1= getXY(car_s+60,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2= getXY(car_s+90,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
    
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          //std::cout << "shift loop"<<std::endl;
          for(int i = 0; i < ptsx.size();i ++)
          {
            // to make the maths simple, shift the vehicle refernce angle to 0 degree
            // Trasnaltion
            //std::cout << "before shift loop "<<ptsx[i]<<" "<<ptsy[i]<<" "<<ref_yaw<<" ref_x "<<ref_x<<" ref_y "<<ref_y<<std::endl;

            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            //std::cout << "shift_x "<<shift_x<<" "<<shift_y<<std::endl;

            // rotation
            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            //std::cout << "shift  loop "<<ptsx[i]<<" "<<ptsy[i]<<" "<<ref_yaw<<std::endl;

          }
          
          // spline
          tk::spline s;
          ////std::cout << "spline loop "<<ptsx[0]<<" "<<ptsy[0]<<std::endl;

          // set(x,y) points to the spline
          s.set_points(ptsx,ptsy);
          
          for(int i = 0; i<previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //std::cout << "prepare for spline loop"<<std::endl;

          
          double target_x = 30.0;
          double target_y = s(target_x);
         //std::cout << "value of y_target"<<std::endl;
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          
          double x_add_on = 0;
          
          /// fill up the rest of the path
          //std::cout << "path loop"<<std::endl;

          for(int i=0; i<= 50 -previous_path_x.size(); i++)
          {
            // speedn has to be converted to m/x from mile/hr (2.24)
            double N=(target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            ////std::cout << "path loop "<<i <<std::endl;
            
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            
            // convert back to gloabl coodinate
            
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
            // //std::cout << "path loop global coordinate"<<i <<std::endl;
            x_point +=ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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