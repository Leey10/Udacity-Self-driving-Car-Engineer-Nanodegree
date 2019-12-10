#include "BehaviorPlanner.h"


BehaviorPlanner::BehaviorPlanner() {}

BehaviorType BehaviorPlanner::update_behavior(Vehicle& egoCar, std::vector<Vehicle>& otherCars, Trajectory& Traj) {

	double turnLeft_benefit = SMALL_BENEFIT;
	double turnRight_benefit = SMALL_BENEFIT;
	double keepLane_benefit = SMALL_BENEFIT;

	// now looking at left-, current-, right- lane, find the benefit of driving on each lane
	// looking left......
	if ((egoCar.lane_at_left == LaneType::NONE) || (egoCar.lane_at_left == LaneType::UNSPECIFIED)) {
		turnLeft_benefit = SMALL_BENEFIT;
	}
	else {
		double FrontLeftGap, FrontLeftV;
		tie(FrontLeftGap,FrontLeftV) = this->get_otherCar(egoCar, otherCars, egoCar.lane_at_left, FROM_FRONT);
		double RearLeftGap, RearLeftV;
		tie(RearLeftGap, RearLeftV) = this->get_otherCar(egoCar, otherCars, egoCar.lane_at_left, FROM_BACK);
		turnLeft_benefit = this->get_benefit(Traj, FrontLeftGap, FrontLeftV, RearLeftGap, RearLeftV);
	}

	// looking ahead.....
	double FrontGap, FrontV;
	tie(FrontGap, FrontV) = this->get_otherCar(egoCar, otherCars, egoCar.lane, FROM_FRONT);
	egoCar.front_gap = FrontGap;
  	egoCar.front_v = FrontV;
    double RearGap, RearV;
	tie(RearGap, RearV) = this->get_otherCar(egoCar, otherCars, egoCar.lane, FROM_BACK);
	keepLane_benefit = this->get_benefit(Traj, FrontGap, FrontV, RearGap, RearV);
  	if (keepLane_benefit >= NO_ACTION_DISTANCE) { // if the front car is far, just keep lane,disregard if change lane has higher benefit
      return BehaviorType::KEEPLANE;
    }
  
  	// looking right ...
	if((egoCar.lane_at_right == LaneType::NONE) || (egoCar.lane_at_right == LaneType::UNSPECIFIED)) {
		turnRight_benefit = SMALL_BENEFIT;
	}
	else {
		double FrontRightGap, FrontRightV;
		tie(FrontRightGap, FrontRightV) = this->get_otherCar(egoCar, otherCars, egoCar.lane_at_right, FROM_FRONT);
		double RearRightGap, RearRightV;
		tie(RearRightGap, RearRightV) = this->get_otherCar(egoCar, otherCars, egoCar.lane_at_right, FROM_BACK);
		turnRight_benefit = this->get_benefit(Traj, FrontRightGap, FrontRightV, RearRightGap, RearRightV);
	}

  	cout << " Left, keep, Right = " << turnLeft_benefit << " " << keepLane_benefit << " " << turnRight_benefit << endl;
	// decide next move, whichever has the highest benefit
	if (turnLeft_benefit >= turnRight_benefit && turnLeft_benefit > keepLane_benefit) {
		return BehaviorType::TURNLEFT;
	}
	else if (turnRight_benefit >= turnLeft_benefit && turnRight_benefit > keepLane_benefit) {
      return BehaviorType::TURNRIGHT;
	}
  	else  {
      	if (FrontGap <= SLOWDOWN_DISTANCE) {
      		return BehaviorType::SLOWDOWN; 
    	}
      	else if (FrontGap <= BRAKE_DISTANCE) {
         	return BehaviorType::BRAKE; 
        }
      	else {
     		return BehaviorType::KEEPLANE; 
        }
    }
	
}

// get the benefit of egoCar goes to/stay on a lane
// after TRAVERSE_TIME, the reachable s of egocar is its benefit, the larger s the more benefit
// egoCar has to be >= 2.0*egoCar.v from the front car or considered unsafe
// egoCar can be <2.0*otherCar.v from the rear car, but the closer it's to the rear car, the lower the benefit
// possible bug: path plan ahead time = 2.0s, but path update time = 1.0s, there's oscission case when otherCar
// has high dynamic, then the 1.0s path left from previous plan does not work and need to be discarded
double BehaviorPlanner::get_benefit(
	const Trajectory& Traj, const double frontGap, const double frontV, 
	const double rearGap, const double rearV ) const {

	double benefit = SMALL_BENEFIT;
	double NewFrontGap = frontGap + TRAVERSE_TIME*frontV -  TRAVERSE_TIME * Traj.startState_s.v ;
  	//cout << " frontGap,frontV,startState_s.v " << frontGap << " " << frontV << " " << Traj.startState_s.v << endl;
	if (NewFrontGap < 2.0 * Traj.startState_s.v) {
		// this is unsafe lane change
		return benefit;
	}
	else {
		
		double NewRearGap = rearGap + TRAVERSE_TIME * Traj.startState_s.v - TRAVERSE_TIME * rearV;
        //cout << " rearGap,rearV,startState_s.v " << rearGap << " " << rearV << " " << Traj.startState_s.v << endl;
		// hit by rear car
		if (NewRearGap <= rearV * 2.0) {
			return benefit;
		}
		else {
			// benefit = how far ahead can egoCar go
			benefit =  NewFrontGap;
			// adjust the benefit: the further to rear car the better
			benefit += NewRearGap*0.5 ;
		}
		
	}

	return benefit;
}




// get the closest car on a particular lane, the method is only invoked when lane_type is not 'NONE' or 'UNSPECIFIED'
// return the gap and speed of the otherCar
tuple<double,double> BehaviorPlanner::get_otherCar(
	const Vehicle& egoCar, const vector<Vehicle>& otherCars, const LaneType lane_type, const double direction) {

	// if no car on lane_type in direction, imagine a car far-far away driving at the same speed as ego car (pretty safe)
	double smallest_gap = BIG_DISTANCE;
	double speed = egoCar.v;
	
	for (auto& otherCar : otherCars) {

		double gap = (otherCar.s - egoCar.s) * direction;

		if (otherCar.lane == lane_type && gap > 0.0 && gap < smallest_gap) {
			smallest_gap = gap;
			speed = otherCar.v;
		}
	}

	return make_tuple(smallest_gap,speed);
}
