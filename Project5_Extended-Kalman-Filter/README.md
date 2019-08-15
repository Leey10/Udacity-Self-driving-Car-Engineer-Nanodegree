# Extended Kalman Filter Project 

In this project,a simple sensor fusion task is conducted using standard and extended Kalman filter. Data from Lidar and Radar are given, the task is to write code that can take the results from each sensor and estimate both the position and velocity of an object.


## Explaination on the files
* main.cpp : provides an interface between the .cpp files and the simulator that is an visulization tool to demonstrate the result. In the main() function, a loop takes in each line of data.txt, sends data to Kalman filter, and calculates the root mean square error.  

* FushionEKF.cpp : is where the sensor fusion process takes place, including Kalman filter initialization, package and unpackage data, calculate filter matrixs.

* kalman_filter.cpp: implements the methods for prediction and update steps of both the standard and extended kalman filter. 

* tools.cpp : calculates the RSME value in real time and the Jacobian matrix for extended kalman filter.

## Runing results
The code can correctly outputs the position and velocity estimation of the object based on very noisy measurement using lidar and radar. The RSME value for [px,py,vx,vy] are less than [0.11,0.11,0.52,0.52] compared with the ground truth value.

## Discussion
The code used for RSME calculating is as follows:
```
for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
	}
  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();
  
  return rmse;
```
The code is not very efficient, since as estimation list gets longer, the 'residual' calculation and the following become time consuming. A better way is to do this incrementally since every time the function is called, there will only be four new values. Keep a variable to store the summation result and get the new result by adding the increments can be a cheaper way.


