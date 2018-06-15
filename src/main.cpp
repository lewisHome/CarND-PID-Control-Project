#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "PID.h"
#include <math.h>
using namespace std;
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
  
  //two pid loops are used 1 to control steering 1 to control throttle
  PID pid_steer, pid_throttle;

  // Initialize the pid variablse. The first 3 values are for the initial P, I, 
  // &  D gain values respectivly and the second three values are for the initial
  // twiddle probing values
  
  //Hard coded twiddle results after 131 laps
  pid_steer.Init(-0.288287, -0.000091001, -4.48556, 0.01, 0.00001, 0.1);
  pid_steer.tolerance = 1e-6;

  pid_throttle.Init(0.21, -0.00009927, -4.60001, 0.01, 0.00001, 0.1);
  pid_throttle.tolerance = 1e-6;
  
  //pid_steer.Init(-0.22, -0.0001, -4.5, 0.01, 0.00001, 0.1);
  //pid_steer.tolerance = 1e-6;

  //pid_throttle.Init(0.2, -0.0001, -4.5, 0.01, 0.00001, 0.1);
  //pid_throttle.tolerance = 1e-6;
  
  //pid_steer.Init(-0.3002331, 0.00005, 0.002, 0.000001, 0.000001, 0.000001);
  //pid_steer.tolerance = 1e-16;

  //pid_throttle.Init(0.17921, -6.809e-5, -0.003009, 0.000001, 0.000001, 0.000001);
  //pid_throttle.tolerance = 1e-16;
  //  pid_steer.Init(-0.3, 0.00005, 0.002, 0.0001, 0.0001, 0.0001);
  //  pid_steer.tolerance = 0.000001;

  //pid_throttle.Init(0.179, -0.00005, -0.003, 0.0001, 0.0001, 0.0001);
  //pid_throttle.tolerance = 0.000001;

  // throttle 0.17921  -6.809e-5  -0.003009
  // steer  -0.3002331 5e-5  0.002
  
  //these variables are used as flags to aid with the twiddle algorithm
  bool twiddle_reset = true;
  bool error_reset = true;
  bool twiddle_steer = true;
  int measure_start = 50;
  int measure_stop = 1500;
  float accumulated_error = 0;
  float throttle_error = 1e6;
  float steer_error = 1e6;
  float throttle_offset = 0.3;
  int i = 0;
  int lap = 0;

  h.onMessage([&pid_steer, &pid_throttle, &lap,
               &twiddle_reset, &error_reset, &twiddle_steer,
               &measure_start, &measure_stop, &accumulated_error,
               &throttle_error, &steer_error, &throttle_offset, &i](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
//          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
          //The more the car has to steer the slower the car should go
          pid_throttle.UpdateError(1.0 - fabs(steer_value));
          throttle_value = pid_throttle.TotalError();

          //DEBUG
          //cout<<"Steer: "<<steer_value<<endl;
          //cout<<"Throttle: "<<throttle_value<<endl;


          //This step records the accumulated error. The accumulated error is
          //proportional to the cross track error and inversly proportional to
          //vehicle speed so the best result is vehicle in road center at high speed 
          if (i > measure_start)
          {
            if (fabs(cte) < 5.0 && speed > 1.25)
            {
              accumulated_error += double(fabs(cte)/(speed));
            }
            //heavy penalty for going off the track or crashing into something
            else
            {
              accumulated_error += 1000;
            }
          }

          //once the simulator has run for a predefined number of steps new PID
          // values are calculated through twiddle
          if (i > measure_stop && twiddle_reset == true)
          {
            twiddle_reset = false;
            lap += 1;
            cout<<"====Lap "<<lap<<"===="<<endl;
            cout<<"\nSteer P: "<<pid_steer.p[0]
                <<"\nSteer I: "<<pid_steer.p[1]
                <<"\nSteer  D: "<<pid_steer.p[2]<<"\n"<<endl;

            cout<<"Throttle P: "<<pid_throttle.p[0]
                <<"\nThrottle I: "<<pid_throttle.p[1]
                <<"\nThrottle  D: "<<pid_throttle.p[2]<<"\n"<<endl;

            cout<<"Accumulated Error: "<<accumulated_error<<endl;

            if (twiddle_steer == true)
            {
              pid_steer.twiddle_complete = false;
              pid_steer.Twiddle(accumulated_error);
              pid_throttle.Reset();
              pid_steer.Reset();
              //once a twiddle loop has been completed on steer go back to work
              //on throttle
              if (pid_steer.twiddle_complete == true)
              {
                twiddle_steer = false;
                cout<<"+++ Steer Results +++"<<endl;
                cout<<"Steer best error: "<<pid_steer.best_error<<endl;
                cout<<"P: "<<pid_steer.p[0]<<"\nI: "<<pid_steer.p[1]<<"\nD: "<<pid_steer.p[2]<<"\n"<<endl;

                //cout<<"+++ start throttle twiddle +++"<<endl;
              }

              //DEBUG
              //cout<<"Twiddle best error: "<<pid_steer.best_error<<endl;
              //cout<<"Steer Error: "<<accumulated_error<<endl;
              //cout<<"P:"<<pid_steer.p[0]<<"I:"<<pid_steer.p[1]<<"D:"<<pid_steer.p[2]<endl;

            }
            else
            {
              pid_throttle.twiddle_complete = false;
              pid_throttle.Twiddle(accumulated_error);
              pid_throttle.Reset();
              pid_steer.Reset();
              //once a twiddle loop has been completed on throttle go back to 
              //work on steer and make the car go a bit faster
              if (pid_throttle.twiddle_complete == true)
              {
                twiddle_steer = true;
                throttle_offset += 0.01;
                cout<<"+++ Throttle Results +++"<<endl;
                cout<<"Throttle best error: "<<pid_throttle.best_error<<endl;
                cout<<"P:"<<pid_throttle.p[0]<<"I:"<<pid_throttle.p[1]<<"D:"<<pid_throttle.p[2]<<"\n"<<endl;
              }

              //I had this variable increasing to try and make the car go increasingly
              //faster - but it didn't work very well perhaps I was trying to increase
              //the speed too quickly

              //DEBUG
              //cout<<"Throttle CTE: "<<accumulated_error<<endl;
              //cout<<"Throttle Offset: "<<throttle_offset<<endl;              
              //cout<<"P:"<<pid_throttle.p[0]<<"I:"<<pid_throttle.p[1]<<"D:"<<pid_throttle.p[2]<endl;

            }
            
            //once new PID values have been calculated reset simulator
            std::string reset_msg = "42[\"reset\",{}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            accumulated_error = 0;
            i = 0;
          }
          
          i += 1;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value + throttle_offset;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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

  h.onConnection([&h, &twiddle_reset](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    twiddle_reset = true;
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
