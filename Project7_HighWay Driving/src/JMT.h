#ifndef JMT_H_
#define JMT_H_

#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"

class JMT {

public:
	Eigen::VectorXd alpha; // the coefficients for JMT polynomial
	JMT(const State& start, const State& end, const double T);
	double get(const double t) const;
};

#endif // JMT_H_


