#include <fstream>
#include <math.h> 
#include <uWS/uWS.h>
#include <chrono> 
#include <iostream>
#include <thread> 
#include <vector>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "PathTools.h" 
#include "helpers.h"
#include "Vehicle.h" 
#include "JMT.h" 
#include "BehaviorPlanner.h" 
#include "Trajectory.h" 
#include "helper_functions.h"

using namespace std;

// For convenience
using json = nlohmann::json;

int main() {

	uWS::Hub h;

	cout << "Program Started." << endl;
	bool just_starting = true;

	cout << "Loading map..." << endl;
	PathTools pathHighway("../data/highway_map.csv", TRACK_DISTANCE);
	cout << "Map loaded..." << endl;
	
  	Trajectory Trajectory;

	h.onMessage([&pathHighway,&just_starting,&Trajectory](

		uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {

			// "42" at the start of the message means there's a websocket message event.
			// The 4 signifies a websocket message
			// The 2 signifies a websocket event
			//auto sdata = string(data).substr(0, length);
			//cout << sdata << endl;
			
			if (length && length > 2 && data[0] == '4' && data[1] == '2') {

				auto s = hasData(data);

				if (s != "") {
					auto j = json::parse(s);

					string event = j[0].get<string>();
					
					if (event == "telemetry") {

						//*********************************
						//* Get relevant data from simulator
						//*********************************

						// j[1] is the data JSON object

						// Main car's localization Data
						double car_x = j[1]["x"];
						double car_y = j[1]["y"];

						double car_s = j[1]["s"];
						double car_d = j[1]["d"];
						double car_speed = j[1]["speed"];

						// Previous path data given to the Planner
						vector<double> previous_path_x = j[1]["previous_path_x"];
						vector<double> previous_path_y = j[1]["previous_path_y"];
						double end_path_s = j[1]["end_path_s"];
						double end_path_d = j[1]["end_path_d"];

						auto sensor_fusion = j[1]["sensor_fusion"];
						
						//*********************************
						//* Update car object
						//*********************************
                      	Vehicle egoCar; // but Mithi likes 666
                      	//cout << "---------------------------------" << endl;
						//cout << " Traj.start.s-declar = " << Trajectory.startState_s.p << " " << Trajectory.startState_s.v << endl;
                      	//cout << " Traj.start.d-declar = " << Trajectory.startState_d.p << " " << Trajectory.startState_d.v << endl;
                      	//cout << " egoCar position-declaration = " << egoCar.s << " " << egoCar.d << endl;
                      	//cout << " egoCar ID = " << egoCar.id << endl;
						
						egoCar.update_position(car_s, car_d);
						egoCar.update_speed(car_speed);
						egoCar.specify_adjacent_lanes();

						//*********************************
						//* Generate the XY_points which will be sent to the simulator
						//*********************************
						
						// Our default is to just give back our previous plan
						int n = previous_path_x.size();
                      	//cout << "---------------------------------" << endl;
                      	//cout << " number of points in previous path = " << n << endl;
                      	//cout << " start state of path: " << endl;
                      	//cout << " Traj.start.s = " << Trajectory.startState_s.p << " " << Trajectory.startState_s.v << endl;
                      	//cout << " Traj.start.d = " << Trajectory.startState_d.p << " " << Trajectory.startState_d.v << endl;
						XYPoints XY_points = { previous_path_x, previous_path_y, n };
						
						/*
						if (just_starting) {

							//Our car hasn't moved yet. Let's move it!
							cout << "Starting engine..." << endl;
							cout << "egoCar.s = " << egoCar.s << endl;
							cout << "egoCar.v = " << egoCar.v << endl;
							print_lane(egoCar.lane);
							egoCar.startState_s = { egoCar.s,MIN_SPEED,0 };
							egoCar.front_gap = BIG_DISTANCE;
							egoCar.front_v = SPEED_LIMIT;
							Trajectory.new_Trajectory(egoCar, BehaviorType::KEEPLANE);
							just_starting = false;
							cout << "Engine started..." << endl;

						}*/
				 		
						if (n <= PATH_SIZE_CUTOFF) {
							
							// the plannar updates path every UPDATE_RATE sec.
							// Make a list of all relevant information about other cars
							vector<Vehicle> otherCars;

							for (int i = 0; i < sensor_fusion.size(); i++) {

								int id = sensor_fusion[i][0];
								double s = sensor_fusion[i][5];
								double d = sensor_fusion[i][6];
								double vx = sensor_fusion[i][3];
								double vy = sensor_fusion[i][4];

								Vehicle otherCar(id);
								otherCar.update_position(s, d);
								otherCar.update_speed(sqrt(vx * vx + vy * vy));
								otherCars.emplace_back(otherCar);
							}
							
                          	/*
							// Print for debugging
							cout << "---------------------------------" << endl;
							cout << "STATE: s, d --- x, y --- v:" << endl;
							cout << car_s << " , "
								<< car_d << " --- "
								<< car_x << " , "
								<< car_y << " --- "
								<< car_speed << ":" << endl;

							cout << "---------------------------------" << endl;
							cout << "our lane: " << endl;
							print_lane(egoCar.lane);
							cout << endl;

							cout << "---------------------------------" << endl;
							*/
							// Decide whether to turn left, turn right or keeplane based on data at hand
							// NOTE: BehaviorPlanner updates our car's current leading/front vehicle's speed and gap

							BehaviorPlanner planner;
							BehaviorType behavior = planner.update_behavior(egoCar, otherCars, Trajectory);
							
							// This trajectory generates target states and then from this
							// generates a jerk minimized trajectory fucntion
							// given the impending state and suggested behavior
                          	if ( n==0 ) {
                              	Trajectory.startState_s = { egoCar.s,SPEED_LOW_LIMIT*0.3,0 };
                              	Trajectory.startState_d = { egoCar.d,0,0 };
								egoCar.front_gap = BIG_DISTANCE;
								egoCar.front_v = SPEED_HIGH_LIMIT;
                                BehaviorType behavior = BehaviorType::KEEPLANE;
                              	cout << " This is the first trajectory " << endl;
                            }
                          	print_behavior(behavior);
                          	cout << "---------------------------------" << endl;
							Trajectory.new_Trajectory(egoCar, behavior);

							// Update saved state of our car (THIS IS IMPORTANT) with the latest
							// generated target states, this is to be used as the starting state
							// when generating a trajectory next time
							Trajectory.update_start_states(Trajectory.targetState_s, Trajectory.targetState_d);
                            //cout << "---------------------------------" << endl;
                          	//cout << " 1.saved Traj.start.s = " << Trajectory.startState_s.p <<" "<< Trajectory.startState_s.v << endl;
   							//cout << " 1.saved Traj.start.d = " << Trajectory.startState_d.p <<" "<< Trajectory.startState_d.v << endl;
                          	
							// convert this trajectory in the s-d frame to to discrete XY points
							// the simulator can understand
                          	
							XYPoints NextXY_points = pathHighway.make_path(
								Trajectory.get_jmt_s(), Trajectory.get_jmt_d(), TIME_INCREMENT, NUMBER_OF_POINTS);

							NextXY_points.n = NUMBER_OF_POINTS;

							// Append these generated points to the old points
							XY_points.xs.insert(
								XY_points.xs.end(), NextXY_points.xs.begin(), NextXY_points.xs.end());

							XY_points.ys.insert(
								XY_points.ys.end(), NextXY_points.ys.begin(), NextXY_points.ys.end());

							XY_points.n = XY_points.xs.size();
                          //cout << "---------------------------------" << endl;
                          //cout << " 2.saved Traj.start.s = " << Trajectory.startState_s.p <<" "<< Trajectory.startState_s.v << endl;
   						  //cout << " 2.saved Traj.start.d = " << Trajectory.startState_d.p <<" "<< Trajectory.startState_d.v << endl;
						}
						//*********************************
						//* Send updated path plan to simulator
						//*********************************
                      	
						json msgJson;

						msgJson["next_x"] = XY_points.xs;
						msgJson["next_y"] = XY_points.ys;


						auto msg = "42[\"control\"," + msgJson.dump() + "]";

						//this_thread::sleep_for(chrono::milliseconds(1000));
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}

				}
				else {
					// Manual driving
					std::string msg = "42[\"manual\",{}]";
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
		});

	// We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t, size_t) {

		const std::string s = "<h1>Hello world!</h1>";

		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		}
		else {
			res->end(nullptr, 0); // i guess this should be done more gracefully?
		}
		});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
		});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
		});

	int port = 4567;

	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}
