#include "Trajectory.h"

using namespace std;

Trajectory::Trajectory() {}

void Trajectory::new_Trajectory(Vehicle& egoCar, const BehaviorType behavior) {
	
  	// in KEEPLANE: if car speed is high, slow it down, if is low, speed it up
	if (behavior == BehaviorType::KEEPLANE) {
		if (this->startState_s.v >= SPEED_HIGH_LIMIT) {
          this->startState_s.v *= 0.9;
        }
      	else if (this->startState_s.v <= SPEED_LOW_LIMIT) {
          this->startState_s.v *= 1.1;
        }
	}
  	// if SLOWDOWN: reduce the startState_s.v
  	if (behavior == BehaviorType::SLOWDOWN) {
      this->startState_s.v *= 0.9;   
    }
   // if BRAKE: use a speed slightly lower than the front car to enlarge the gap
  	if (behavior == BehaviorType::BRAKE) {
      this->startState_s.v = egoCar.front_v * 0.98;
    }
  
  	// if change lane, then the 2.0s safe time margin has been checked in BehaviorPlanner::update_behavior
	double target_s = this->startState_s.p + TRAVERSE_TIME * this->startState_s.v;
	double target_v = this->startState_s.v;

	// target acceleration along the load is zero
	this->targetState_s = { target_s, target_v, 0.0 };
	// get target d component state based on behavior
	// target speed and acceleration sideways of the road are both zero
	this->targetState_d = { egoCar.get_target_d(behavior), 0.0, 0.0 };
	
	// generate JMTs
	JMT jmt_s(this->startState_s, this->targetState_s, TRAVERSE_TIME);
	JMT jmt_d(this->startState_d, this->targetState_d, TRAVERSE_TIME);
  	
  	jmtPair.clear(); // be sure to clear the vector before using emplace_back()
	this->jmtPair.emplace_back(jmt_s);
  	this->jmtPair.emplace_back(jmt_d);
  	
}

void Trajectory::update_start_states(const State& state_s, const State& state_d) {
	this->startState_s = state_s;
	this->startState_d = state_d;
	this->startState_s.p = fmod(this->startState_s.p, TRACK_DISTANCE);
  	//cout << " saved Traj.start.s = " << this->startState_s.p <<" "<< this->startState_s.v << endl;
   	//cout << " saved Traj.start.d = " << this->startState_d.p <<" "<< this->startState_d.v << endl;
  	//cout << "---------------------------------" << endl;
}

JMT Trajectory::get_jmt_s() const {
	return jmtPair[0];
}

JMT Trajectory::get_jmt_d() const {
	return jmtPair[1];
}

