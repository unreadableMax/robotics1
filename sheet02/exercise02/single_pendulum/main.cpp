#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

using namespace Eigen;

// Define a few types to make it easier
typedef VectorXd (*rhsFuncPtr) (const double, const VectorXd&);


VectorXd heun_integrator (const double t, const VectorXd &y, const double h, rhsFuncPtr rhs) {
	return 0.5 * (rhs (t, y) + rhs (t + h, y + h * rhs(t, y)));
}
VectorXd rk4_integrator (const double t, const VectorXd &y, const double h, rhsFuncPtr rhs) {
	VectorXd k1 = rhs (t, y);
	VectorXd k2 = rhs (t + (double) 0.5 * h, y + (double) 0.5 * h * k1);
	VectorXd k3 = rhs (t + (double) 0.5 * h, y + (double) 0.5 * h * k2);
	VectorXd k4 = rhs (t + h, y + h * k3);

	return (double) 1. / 6. * (k1 + (double) 2. * k2 + (double) 2. * k3 + k4);
}

VectorXd euler_integrator (const double t, const VectorXd &y, const double h, rhsFuncPtr rhs) {
	return rhs (t, y);
}


VectorXd rhs (double t, const VectorXd &y) {
	assert (y.size() == 2);

	double m = 0.2;
	double l = 0.4;
	double g = 9.81;

	VectorXd res = VectorXd::Zero(2);
	/* **********************************
	 * put your solution here           *
	 * **********************************/
	res[0] = 0;
	res[1] = 0;

	return res;
}


int main(int argc, char *argv[]) {

	//Initial values
	VectorXd y0  = VectorXd::Zero(2);
	y0[0] = 0.5*M_PI;
	y0[1] = 0.0;
	double t0 = 0.;

	double tf = 5.0;
	double h = 0.01;
		
	double t = t0;
	VectorXd y = y0;

	std::ofstream of("animation.csv");

	while (t <= tf) {
		of << t << ", " << y[0] << "\n";
		y = y + h * rk4_integrator (t, y, h, rhs);
		t = t + h;
	}

	of.close();
	return 0;
}
