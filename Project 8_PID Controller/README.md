# PID Controller Porject
In this project, a simple PID controller is build with C++. The controller generates steering angle values between -1 and 1 to drive the simulator car around a circular track.

### Code Overview
*PID.cpp* includes methods to calculate controller output based on CrossTrack Error(cte) as follows:
```
u = - ( this->Kp * cte + this->Ki * int_cte + this->Kd * diff_cte );
```
A saturation mechanisim is also implemented which guarantees controller output can reach as high as 95% of full capability.  

### Effects of PID Controller
The proportional gain Kp determines response speed of the controller. With a higher Kp, controller reacts to system error faster. However, a Kp too high will cause oscillation on the output and may lead to system unstability. Integral gain Ki is used to eliminate the steady state error, because even the smallest error can be accumulated over time and be compensated by the controller. The negative effect of Ki is that overshoot can happen with high Ki, since integrator increases controller inertia. Differential part of controller is used to "predict" system behavior,which increases the adaptivity of the controller.

### PID Parameter Tunning
The PID parameters are tuned by first setting Ki and Kd equal to 0, and just use Kp to control the car. Increase Kp to the point where the car begins to oscillate. Increase Kd until the oscillation is dampened. Kp and Kd can be further increased to obtained faster response. Lastly, increase Ki so that steady state error of the car can reach zero. Ki can be increased to the point where there's small overshoot and oscillation but system response is fast enough.
