#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>

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

  // 
  // pid.Init();
  // I used twiddle to find these values. I can make it better but somehow it crash. So I use these variables to hang in the project.
  pid.Init(0.137611, 0, 2.89388);
  // pid.Init(0.109, 0, 3.189);
  std::vector<double> cte_list;
  std::vector<double> speed_list;

  h.onMessage([&pid, &cte_list, &speed_list](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

	  cte_list.push_back(cte);
	  speed_list.push_back(speed);

	  if ( cte_list.size() > 10 ){
	    cte_list.erase(cte_list.begin());
	    speed_list.erase(speed_list.begin());
	  }

	  unsigned int count=0;
	  for (unsigned int i=0; i<cte_list.size(); ++i){
	    if (cte_list[i] == cte)
	      ++count;
	    if(speed_list[i] < 1)
	      ++count;
	  }
	  if(count == 2*cte_list.size()){
	    std::string msg = "42[\"reset\",{}]";
	    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  }
	  
	  pid.UpdateError(cte);
	  steer_value = pid.TotalError();
          

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Angle: "<<angle<<std::endl;

	  if (cte > 5 || cte < -5){
	    std::string msg = "42[\"reset\",{}]";
	    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  }
	  else{
	    json msgJson;
	    msgJson["steering_angle"] = steer_value;
	    msgJson["throttle"] = 0.5;
	    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
	    std::cout << msg << std::endl;
	    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  }
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
