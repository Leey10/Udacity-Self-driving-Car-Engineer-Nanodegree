#include "JMT.h"



using Eigen::MatrixXd;
using Eigen::VectorXd;

JMT::JMT(const State& start, const State& end, const double T) {

  	//cout << " JMT::JMT" << endl;
  	//cout << " start = " << start.p << " " << start.v << endl;
  	//cout << " end = " << end.p << " " << end.v << endl;
  	
	MatrixXd A = MatrixXd(3, 3);
	VectorXd b = VectorXd(3);
	VectorXd x = VectorXd(3);
	this->alpha = VectorXd(6);

	const double t2 = T * T;
	const double t3 = T * t2;
	const double t4 = T * t3;
	const double t5 = T * t4;

	A << t3, t4, t5,
		3 * t2, 4 * t3, 5 * t4,
		6 * T, 12 * t2, 20 * t3;

	b << end.p - (start.p + start.v * T + 0.5 * start.a * t2),
		 end.v - (start.v + start.a * T),
		 end.a - start.a;

	x = A.inverse() * b;

	this->alpha << start.p, start.v, start.a / 2.0,
				   x[0]   , x[1]   , x[2]         ;
  	//cout << " alpha  = " << this->alpha << endl;
	//cout << "---------------------------------" << endl;
}

double JMT::get(const double t) const {

	const double t2 = t * t;
	const double t3 = t * t2;
	const double t4 = t * t3;
	const double t5 = t * t4;

	Eigen::VectorXd T = VectorXd(6);
	T << 1.0, t, t2, t3, t4, t5;

	return T.transpose() * this->alpha;
}
