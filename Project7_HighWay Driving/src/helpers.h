#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <math.h>
#include <vector>

using namespace std;

// define MACROS 
// Here's the duration period for each path plan we send to the controller
const double TIME_INCREMENT = 0.02;
const double TRAVERSE_TIME = 2.0;
const double PATH_PLAN_TIME = 2.0;
const int NUMBER_OF_POINTS = int(TRAVERSE_TIME / TIME_INCREMENT);

const double BIG_DISTANCE = 1000000.0;
const double SMALL_BENEFIT = -1000000.0;
const double BIG_BENEFIT = 1000000.0;

// the d value of each lane's center
const double LEFT_d = 2.2;
const double MID_d = 6.0;
const double RIGHT_d = 9.8;

// define path plannar update period in sec
// UPDATE_RATE <= PATH_PLAN_TIME
const double UPDATE_RATE = 1.0;
// how much points left for the controller to perform
// before we start planning again
const int  PATH_SIZE_CUTOFF = NUMBER_OF_POINTS - int(UPDATE_RATE / TIME_INCREMENT);


// used as parameter in BehaviorPlanner::get_gap()
const double FROM_FRONT = 1.0;
const double FROM_BACK = -1.0;

// Total road distance of the the highway loop
const double TRACK_DISTANCE = 6945.554;

// boundaries of vehicle dynamics
const double HARD_SPEED_LIMIT = 22.352; // 50mph in m/s
const double SPEED_HIGH_LIMIT = 21.2;
const double SPEED_LOW_LIMIT = 19.0;
const double ACC_LIMIT = 10; // acceleration limit 10m/s
const double JER_LIMIT = 10; // jerk limit 10m/s2

// no action distance from front car
const double NO_ACTION_DISTANCE = 80;
// slow down distance from front car
const double SLOWDOWN_DISTANCE = 50;
// brake distance from front car
const double BRAKE_DISTANCE = 30;

enum class LaneType {
	LEFT, MID, RIGHT, NONE, UNSPECIFIED
};

enum class BehaviorType {
	KEEPLANE, TURNRIGHT, TURNLEFT, SLOWDOWN, BRAKE
};

/* State - stores three doubles p, v, a
 * intended to store position, velocity, and acceleration components in the s, or d axis
 */
struct State {
	double p;
	double v;
	double a;
};

/* XYPoints stores two vectors x, y which are the map coordinates to be passed to
 * the simulator. Also holds an int n intended to store the number of (x, y) pairs
 */
struct XYPoints {
	std::vector<double> xs;
	std::vector<double> ys;
	int n;
};



#endif  // HELPERS_H
