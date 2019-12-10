# CarND-Path-Planning-Project
In this project, a path planner is developed in C++. The planner generates trajectories for the car to keep lane, speed up / slow down and change lane based on the behaviors of other cars. The planner also makes sure that driving with the trajectories, the car won't overspeed, accerlerate too much, jerk too much or collide with other cars.
   
### Overview of Code
The code is developed based on Mithi Sevilla's blog  [Reflections on Designing a Virtual Highway Path Planner](https://medium.com/@mithi/reflections-on-designing-a-virtual-highway-path-planner-part-1-3-937259164650) and [Mithi Sevilla's Github](https://github.com/mithi/highway-path-planning). I used the C++ infrastructure(Class definitions) that Mithi developed, the main differences are:  
* rewrite BehaviorPlanner.cpp and Trajectory.cpp. Instead of a "cost" based planner(cost of certain behavior) , a "benefit" based planner(benefit of certain lane) is developed. The "benefit" of a lane is defined as the furthest distance the car can reach driving on the lane. Therefore, the goal of the path planner is to allow the car reach as far distance as possible (without rule violations).  
* remove the "void start_engine()" which is used to generate the very first trajector. Instead, let the path planner take care of   trajectory generating in any cases as long as required variables as properly initialized. 

### Benefit Based Strategy
##### 1. tuple<double,double>BehaviorPlanner::get_otherCar(const Vehicle& egoCar, const vector<Vehicle>& otherCars, const LaneType lane_type, const double direction)  
  
   First, the planner needs to get the position and speed information on the cars around. The get_otherCar method finds the car on *lane_type* (left, current, right) that is closest to *egoCar* in *direction*(front, rear). The distance to the closest car and speed of that car are returned by the tuple.  
   
##### 2. double BehaviorPlanner::get_benefit(const Trajectory& Traj, const double frontGap, const double frontV, const double rearGap, const double rearV )

   Second, the planner evaluates the "benefit" of driving on each lane using the information of cars on that lane. *frontGap* is the gap from ego car to the font car, *frontV* is the speed of front car. The same for *rearGap* and *rearV*.  
   A *NewFrontGap* is predicted for the car in next trajectory: 
  ```
  NewFrontGap = frontGap + TRAVERSE_TIME * frontV -  TRAVERSE_TIME * Traj.startState_s.v; 
  ```
  Now, the *NewFrontGap* is evaluated againest the 2.0s rule to make sure after next trajectory, the distance between ego car and the    front car is enough to allow the ego car drive for another 2.0s. If this condition does not meet, it's considered unsafe drive, and the  benefit will be very small, as follows:
```
if (NewFrontGap < 2.0 * Traj.startState_s.v) {
  // this is unsafe drive
  return benefit;
}
```
  If it is safe from the front car, the evaluation is conducted again for the rear car, as follows:
  ```
  double NewRearGap = rearGap + TRAVERSE_TIME * Traj.startState_s.v - TRAVERSE_TIME * rearV;
		// might be hit by rear car
		if (NewRearGap <= rearV * 2.0) {
			return benefit;
	}
 ```
  If it is safe driving both direction after the evaluation, the benefit equals the *NewFrontGap*, a larger benefit means after the next behavior, ego car can go further. The *NewRearGap* is not as significant as *NewFrontGap* at this stage, but a larger *NewRearGap* makes the ego car safer. So the final benefit is calculated with weights.  
```
// benefit = how far ahead can egoCar go  
		benefit =  NewFrontGap;
// adjust the benefit: the further to rear car the better  
		benefit += NewRearGap*0.5 ;
```
##### 3. BehaviorType BehaviorPlanner::update_behavior(Vehicle& egoCar, std::vector<Vehicle>& otherCars, Trajectory& Traj)  
  
  Third, the *update_behavior* method implement the pipleline to first *get_otherCar* on left, current, and right lanes, then *get_benefit* on left_turn, keep_lane and right_turn. Finally, behavior among the three is returned based on the value of their respective benefit.  
  
One interesting result of this strategy is that when the Simulator just starts, the ego car does a quick lane change occasionally even when the middle lane does not have any car in front. The reason is that the planner detects the rear car and finds that if ego car stays on the middle lane, the rear car will hit the ego car. After this quick lane change, the rear car can pass and be observed. The default behavior of Simulator is to slow down the car behind the ego car to avoid collision.   

### Trajectory Generating
##### void Trajectory::new_Trajectory(Vehicle& egoCar, const BehaviorType behavior)
##### 1. Trajectory of lane change
If the planner wants to do lane change, then target_s and target_v are calculated, the targetState_s and targetState_d are packaged and input to the Jerk Minimum Trajectory (JMT) generator, which generates the coefficients of the spline polynomial.  
```
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
```

##### 2. Trajectory of keep lane
The planner needs to change ego car speed in the following cases when not changing lane:  
* define *SPEED_LOW_LIMIT* and *SPEED_HIGH_LIMIT*, when ego car speed is lower than *SPEED_LOW_LIMIT*, the ego car speeds up by multipling starState_s.v by 1.1, while ego car speed is higher than *SPEED_HIGH_LIMIT*, the ego car slows down by multipling by 0.9 
```
// in KEEPLANE: if car speed is high, slow it down, if is low, speed it up
	if (behavior == BehaviorType::KEEPLANE) {
		if (this->startState_s.v >= SPEED_HIGH_LIMIT) {
          this->startState_s.v *= 0.9;
        }
      	else if (this->startState_s.v <= SPEED_LOW_LIMIT) {
          this->startState_s.v *= 1.1;
        }
	}
 ```
 
* define *NO_ACTION_DISTANCE*, *SLOWDOWN_DISTANCE* and *BRAKE_DISTANCE*, if the distance between ego car and front car is larger than *NO_ACTION_DISTANCE*, the planner will keep lane regardless of benefits of lane change; if front gap is less than *SLOWDOWN_DISTANCE*,ego car speed becomes 0.9 of its current speed; if front gap is less than *BRAKE_DISTANCE* meaning ego car is too close to front car, then ego car speed becomes 0.98 of that of front car (in this way, the collision will not happen).  
```
// if SLOWDOWN: reduce the startState_s.v
  	if (behavior == BehaviorType::SLOWDOWN) {
      this->startState_s.v *= 0.9;   
    }
// if BRAKE: use a speed slightly lower than the front car to enlarge the gap
  	if (behavior == BehaviorType::BRAKE) {
      this->startState_s.v = egoCar.front_v * 0.98;
    }
```
### Result of Implementation
The planner can correctly generates path with highest "benefit" for the car. The driving is safe without over accerleration, over jerking, over speed and collision. 

### Improvements
1. The planner now looks at just one lane on the left/right, but sometimes, looking at two lanes can give better output. So a planner that looks at all the lanes will be more advanced. However, this also requries more complex trajectory generator,for example, when the car has to first slow down on the left most lane, right change to middle lane which has lower benefit than driving on left lane, and then right lane change to right most lane which has the highest benefit.  

2. To avoid rule violations, several constants are defined regarding speed and distance. However, the constants need to be defined conservatively. So the dynamic model can be added to the trajectory generator or some type of filter can be used after the JMT generator, so that speed, accerleration rule can be "theoretically" guaranteed.
  
