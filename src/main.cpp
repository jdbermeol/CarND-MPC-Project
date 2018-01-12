#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "MPC.h"
#include "json.hpp"
#include "utils.h"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          vector<double> waypoints_x(ptsx.size(), 0.0);
          vector<double> waypoints_y(ptsx.size(), 0.0);

          // transform waypoints to be from car's perspective
          for (int i = 0; i < ptsx.size(); ++i) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            waypoints_x[i] = dx * cos(-psi) - dy * sin(-psi);
            waypoints_y[i] = dx * sin(-psi) + dy * cos(-psi);
          }

          // Build coeffs
          Eigen::Map<Eigen::VectorXd> ptsxv(waypoints_x.data(), waypoints_x.size());
          Eigen::Map<Eigen::VectorXd> ptsyv(waypoints_y.data(), waypoints_y.size());
          Eigen::VectorXd coeffs = polyfit(ptsxv, ptsyv, 3);

          // Calculating errors
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);
          cout << epsi << endl; 

          // Build state
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          // Find optimal
          vector<double> actuators = mpc.Solve(state, coeffs);

          json msgJson;
          msgJson["steering_angle"] = actuators[0] /(deg2rad(25));
          msgJson["throttle"] = actuators[1];

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for(int i = 2; i < actuators.size(); ++i)
            if(i%2 == 0)
              mpc_x_vals.push_back(actuators[i]);
            else
              mpc_y_vals.push_back(actuators[i]);

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = waypoints_x;
          msgJson["next_y"] = waypoints_y;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
