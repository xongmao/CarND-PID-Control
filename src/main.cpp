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

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(0.2, 0.004, 3.0, 0.0, 0.0);
  int count = 0;
  int shift = 0;
  double dKp = 1.2;
  double dKi = 1.004;
  double dKd = 4.0;
  double best_err = 1000.0;
  bool flag = true;
  double err = 0.0;
  h.onMessage([&pid, &count, &dKp, &dKi, &dKd, &best_err, &flag, &shift, &err](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
		  pid.int_cte += cte;
		  if (count==0){
			  if (cte<best_err){
				  best_err = cte;
			  }
			  pid.prev_cte = cte;
			  pid.UpdateError(cte);
			  double tmp = pid.p_error + pid.i_error + pid.d_error;
			  if (tmp > 3.1415926/4.0){
				  tmp = 3.1415926/4.0;
			  }
			  if (tmp < -3.1415926/4.0){
				  tmp = -3.1415926/4.0;
			  }
              steer_value = tan(tmp);
			  //steer_value = fmod(angle + steer_value, 2.0 * 3.1415926);
			  
		  } else if (dKp+dKi+dKd>0.2 && count==1){
			  if (err < best_err){
				  best_err = err;
			  }
			  if (shift==0){
				pid.Kp += dKp;
			  }else if (shift==1){
				pid.Ki += dKi;
			  }else{
				pid.Kd += dKd;
			  }
			  pid.UpdateError(cte);
			  double tmp = pid.p_error + pid.i_error + pid.d_error;
			  if (tmp > 3.1415926/4.0){
				  tmp = 3.1415926/4.0;
			  }
			  if (tmp < -3.1415926/4.0){
				  tmp = -3.1415926/4.0;
			  }
              steer_value = tan(tmp);
			  //steer_value = fmod(angle + steer_value, 2.0 * 3.1415926);
			  pid.prev_cte = cte;
		  }else if (dKp+dKi+dKd>0.2 && count!=1 && flag){
			  if (count>1000){
				  err += (cte * cte);
			  }  
			  if (err < best_err){
				  best_err = err;
				  if (shift==0){
					dKp *= 1.1;
				  }else if (shift==1){
					dKi *= 1.1;
				  }else{
					dKd *= 1.1;
				  }
				  shift++;
				  if (shift>=3){
					  shift = 0;
				  }
				  if (shift==0){
					pid.Kp += dKp;
				  }else if (shift==1){
					pid.Ki += dKi;
				  }else{
					pid.Kd += dKd;
				  }
				  pid.UpdateError(cte);
				  double tmp = pid.p_error + pid.i_error + pid.d_error;
				  if (tmp > 3.1415926/4.0){
					  tmp = 3.1415926/4.0;
				  }
				  if (tmp < -3.1415926/4.0){
					  tmp = -3.1415926/4.0;
				  }
				  steer_value = tan(tmp);
				  //steer_value = fmod(angle + steer_value, 2.0 * 3.1415926);
				  pid.prev_cte = cte;
			  }else{
				  flag = false;
				  if (shift==0){
					pid.Kp -= dKp*2.0;
				  }else if (shift==1){
					pid.Ki -= dKi*2.0;
				  }else{
					pid.Kd -= dKd*2.0;
				  }
				  pid.UpdateError(cte);
				  double tmp = pid.p_error + pid.i_error + pid.d_error;
				  if (tmp > 3.1415926/4.0){
					  tmp = 3.1415926/4.0;
				  }
				  if (tmp < -3.1415926/4.0){
					  tmp = -3.1415926/4.0;
				  }
				  steer_value = tan(tmp);
				  //steer_value = fmod(angle + steer_value, 2.0 * 3.1415926);
				  pid.prev_cte = cte;
			  }
		  }else if (dKp+dKi+dKd>0.2 && count!=1 && !flag){
			  flag = true;
			  if (count>1000){
				  err += (cte * cte);
			  }
			  if (err < best_err){
				  best_err = err;
				  if (shift==0){
					dKp *= 1.1;
				  }else if (shift==1){
					dKi *= 1.1;
				  }else{
					dKd *= 1.1;
				  }
				  shift++;
				  if (shift>=3){
					  shift = 0;
				  }
				  if (shift==0){
					pid.Kp += dKp;
				  }else if (shift==1){
					pid.Ki += dKi;
				  }else{
					pid.Kd += dKd;
				  }
				  pid.UpdateError(cte);
				  double tmp = pid.p_error + pid.i_error + pid.d_error;
				  if (tmp > 3.1415926/4.0){
					  tmp = 3.1415926/4.0;
				  }
				  if (tmp < -3.1415926/4.0){
					  tmp = -3.1415926/4.0;
				  }
				  steer_value = tan(tmp);
				  //steer_value = fmod(angle + steer_value, 2.0 * 3.1415926);
				  pid.prev_cte = cte;
			  }else{
				  if (shift==0){
					pid.Kp += dKp;
					dKp *= 0.9;
				  }else if (shift==1){
					pid.Ki += dKi;
					dKi *= 0.9;
				  }else{
					pid.Kd += dKd;
					dKd *= 0.9;
				  }
				  shift++;
				  if (shift>=3){
					  shift = 0;
				  }
				  if (shift==0){
					pid.Kp += dKp;
				  }else if (shift==1){
					pid.Ki += dKi;
				  }else{
					pid.Kd += dKd;
				  }
				  pid.UpdateError(cte);
				  double tmp = pid.p_error + pid.i_error + pid.d_error;
				  if (tmp > 3.1415926/4.0){
					  tmp = 3.1415926/4.0;
				  }
				  if (tmp < -3.1415926/4.0){
					  tmp = -3.1415926/4.0;
				  }
				  steer_value = tan(tmp);
				  //steer_value = fmod(angle + steer_value, 2.0 * 3.1415926);
				  pid.prev_cte = cte;
			  }
		  }else if (dKp+dKi+dKd<=0.2){
			  pid.UpdateError(cte);
			  double tmp = pid.p_error + pid.i_error + pid.d_error;
			  if (tmp > 3.1415926/4.0){
				  tmp = 3.1415926/4.0;
			  }
			  if (tmp < -3.1415926/4.0){
				  tmp = -3.1415926/4.0;
			  }
			  steer_value = tan(tmp);
			  //steer_value = fmod(angle + steer_value, 2.0 * 3.1415926);
			  pid.prev_cte = cte;
		  }	  
		  count++; 
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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