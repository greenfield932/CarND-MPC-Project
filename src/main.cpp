#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "MPC.h"
#include "json.hpp"
#include "MPC_helpers.h"
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


constexpr double MilesPerHour2MetersPerSecondCoeff(){
    //meter = mile/0.00062137
    //1 h = 3600 s
    return 1./0.00062137/3600.;
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
    cout << sdata << endl;
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

          double delta= j[1]["steering_angle"];
          double a = j[1]["throttle"];
          //1. Preprocessing
          //1.1 Transform x,y,psi coordinates from map coordinate system to car's coordinate system,
          //so the first point is the origin and next points coordinates calculated from this point.
          //1.2 Rotate all points on 90 degree to aligh zero heading with X axis.

          const size_t pointsCount = ptsx.size();

          for (size_t i=0;i<pointsCount;++i)
          {
              double offset_x = ptsx[i] - px;
              double offset_y = ptsy[i] - py;
              ptsx[i] = offset_x * cos(0. - psi) - offset_y * sin(0. - psi);
              ptsy[i] = offset_x * sin(0. - psi) + offset_y * cos(0. - psi);
          }

          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptsx.data(), pointsCount), ptsy_transform(ptsy.data(), pointsCount);

          //Approximate reference trajectory by 3-rd order polynomial
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          //Latency
          //One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency.
          //The resulting state from the simulation is the new initial state for MPC.

          // Latency value
          const double latency = 0.1;//100 ms

          // Initial state.
          const double x0 = 0;
          const double y0 = 0;
          const double psi0 = 0;
          const double cte0 = coeffs[0];
          const double epsi0 = -atan(coeffs[1]);
          const double dt = latency;

          // State after delay.
          double x1 = x0 + v * cos(psi0) * dt;
          double y1 = y0 + v * sin(psi0) * dt;
          double psi1 = psi0 - v / MPCCoeffs::Lf * delta * dt;
          double v1 = v + a * dt;
          double psi_des = atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

          double cte1 = cte0 - y0 + v * sin(epsi0) * dt;
          double epsi1 = psi0 - psi_des + v / MPCCoeffs::Lf * delta * dt;

          Eigen::VectorXd state((size_t)vector_index::siCount);
          //Since we made transormation relative to the first point then state position and rotation components are 0
          state[vector_index::siX] = x1;
          state[vector_index::siY] = y1;
          state[vector_index::siPsi] = psi1;
          state[vector_index::siV] = v1;
          state[vector_index::siCTE] = cte1;
          state[vector_index::siEPsi] = epsi1;

          //2. Solve optimization problem
          auto result = mpc.Solve(state, coeffs);

          //3. Post processing
          //3.1 Normalize steering value
          double steer_value = result[0];
          double throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          constexpr double steer_norm = deg2rad(25);

          msgJson["steering_angle"] = steer_value/steer_norm;
          msgJson["throttle"] =  throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          for(size_t i = 2; i < result.size()-1; i+=2)
          {
              mpc_x_vals.push_back(result[i]);
              mpc_y_vals.push_back(result[i+1]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = ptsx;
          msgJson["next_y"] = ptsy;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
