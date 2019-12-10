#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include <iostream>
#include "helpers.h"
#include "Vehicle.h"
#include "JMT.h"

class Trajectory {

public:
	Trajectory();
	void new_Trajectory(Vehicle& car, const BehaviorType behavior);
  	void update_start_states(const State& state_s, const State& state_d); 
	State startState_d;
	State startState_s;
  	State targetState_d;
	State targetState_s;
	JMT get_jmt_s() const;
	JMT get_jmt_d() const;

private:
	std::vector<JMT> jmtPair;

};

#endif //TRAJECTORY_H_


