#ifndef BEHAVIORPLANNER_H_
#define BEHAVIORPLANNER_H_

#include <vector>
#include <iostream>
#include <tuple>

#include "helpers.h"
#include "Vehicle.h"
#include "Trajectory.h"

using namespace std;


class BehaviorPlanner {

public:
	BehaviorPlanner();
	BehaviorType update_behavior(Vehicle& egoCar, std::vector<Vehicle>& otherCars, Trajectory& Traj);
	tuple<double,double> get_otherCar(const Vehicle& egoCar,
		const vector<Vehicle>& otherCars,
		const LaneType lane_type,
		const double direction);
private:

	double get_benefit(
		const Trajectory& Traj, const double frontGap, const double frontV,
		const double rearGap, const double rearV) const;

};

#endif //BEHAVIORPLANNER_H_
