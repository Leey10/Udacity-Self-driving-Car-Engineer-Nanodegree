#include <string>
#include <vector>
#include<iostream>
#include "helpers.h"

using namespace std;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

void print_lane(LaneType lane) {

	if (lane == LaneType::LEFT) {
		cout << "LEFT       " << endl;
	}
	else if (lane == LaneType::MID) {
		cout << "MID        "<< endl;
	}
	else if (lane == LaneType::RIGHT) {
		cout << "RIGHT      "<< endl;
	}
	else if (lane == LaneType::NONE) {
		cout << "NONE       "<< endl;
	}
	else if (lane == LaneType::UNSPECIFIED) {
		cout << "UNSPECIFIED"<< endl;
	}
}

void print_behavior(BehaviorType behavior) {

	if (behavior == BehaviorType::KEEPLANE) {
		cout << "KEEP-LANE       " << endl;
	}
	else if (behavior == BehaviorType::TURNRIGHT) {
		cout << "TURN-RIGHT       "<< endl;
	}
	else if (behavior == BehaviorType::TURNLEFT) {
		cout << "TURN-LEFT      "<< endl;
    }
  	else if (behavior == BehaviorType::BRAKE) {
		cout << "BRAKE      "<< endl;
    }
	else  {
		cout << "SLOWDOWN"<< endl;
	}
}
