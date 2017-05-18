#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;

  // TODO: Initialize the pid variable.

  //  pid.Init(0.1,0.0,0.0);
    pid.Init(0.48,0.0,5.0);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          json msgJson;
          //msgJson["throttle"] =0.4*(1.0-fabs(steer_value));

          if(fabs(cte)<1.8) msgJson["throttle"] =0.4*(1.0-fabs(cte)/1.8);
          else  msgJson["throttle"] =0;



        pid.UpdateError(cte);
        std::cout << " period_index: " <<pid.period_index<< " twiddle_count:"<<pid.twiddle_count<<"  twiddle_mode:"<<pid.twiddle_mode<<" twiddle_param_switch:"<<pid.twiddle_param_switch<<std::endl;
        steer_value=pid.TotalError();
        std::cout << "CTE:"<<cte<< "  Steering Value: " << steer_value<<  " Kp: " <<pid.Kp<<"  Kd:"<<pid.Kd<<"  Ki:"<<pid.Ki<<std::endl;

        if(cte*steer_value>0){
           std::cout << " positive feedback occurs " <<std::endl;
           steer_value=cte>0?0.0:0.1;
        }


        if(fabs(cte)>2.8||speed<0.1 && pid.is_twiddle_on)
        {

            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            //steer_value=-0.3;
            //msgJson["throttle"] =0.0;

            if(fabs(cte)>2.8)
            std::cout << "RESET PID: too high CTE occurs " <<std::endl;

           //std::cout << " Kp: " <<pid.Kp<<"  Kd:"<<pid.Kd<<"  Ki:"<<pid.Ki<<std::endl;

           //pid.Init(pid.Kp+0.1,0.0,pid.Kd+0.1);
           //if(pid.period_index>300)
           pid.Twiddle();



        }
        if(pid.period_index==pid.period_count)
        {
            pid.Twiddle();
            pid.period_index=0;
            pid.twiddle_count++;
            std::cout << "  twiddle_count:"<<pid.twiddle_count<<"  twiddle_mode"<<pid.twiddle_mode
                         <<"   Kp: " <<pid.Kp<<"  Kd:"<<pid.Kd<<"  Ki:"<<pid.Ki<<std::endl;
        }
        if(pid.twiddle_mode==3)
            std::cout << "final Kp: " << pid.Kp << "   final Ki: " << pid.Ki<<  "    final Kd:" << pid.Kd<<std::endl;


          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value <<  " Speed: " << speed <<std::endl;

          msgJson["steering_angle"] = steer_value;

          //msgJson["throttle"] =0.3;
          //msgJson["throttle"] =0.8*(0.8-fabs(steer_value));
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
