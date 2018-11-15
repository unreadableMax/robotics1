#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

using namespace Eigen;

// Define a few types to make it easier
typedef VectorXd (*rhsFuncPtr) (const double, const VectorXd&);

// This should be the alternative Runge-Kutta-2 variant, better than euler, weaker than rk4
VectorXd heun_integrator (const double t, const VectorXd &y, const double h, rhsFuncPtr rhs) {
	return 0.5 * (rhs (t, y) + rhs (t + h, y + h * rhs(t, y)));
}

// We had that in Simulation with Rasenat, do you remember, Max? 4th order Runge-Kutta it should be far superior to euler and better than heun in case of fault tolerance
VectorXd rk4_integrator (const double t, const VectorXd &y, const double h, rhsFuncPtr rhs) {
	VectorXd k1 = rhs (t, y);
	VectorXd k2 = rhs (t + (double) 0.5 * h, y + (double) 0.5 * h * k1);
	VectorXd k3 = rhs (t + (double) 0.5 * h, y + (double) 0.5 * h * k2);
	VectorXd k4 = rhs (t + h, y + h * k3);

	return (double) 1. / 6. * (k1 + (double) 2. * k2 + (double) 2. * k3 + k4);
}

// Based on simulation, this is a simple and weak numeric caluclation method
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
	
	/* rhs = right hand side
	 * Like in the tutorial, the right hand side of a differential equation 
	 * y'(t) = f(t,y(t)) is the part on the right side of the =
	 */

	/* The function of this task is: m*l*q'' = -m*g*sin(q)
	 * This is no "ODE" (because of "q''" instead of 'q'
 	 * So we need to make it into an ODE using order reduction from the assignment:
	 */

	/* m*l*q'' = -m*g*sin(q)
	 * l*q'' = -g*sin(q)
	 * q'' = - g/l * sin(q) 
	 * with q'' = r' we get two differential equations:
	 * q' = r
	 * r' = - g/l * sin(q)
	 */

	//We get a reference to y, which is the value from the numeric calculation from the last running through this function: The vector (q r) (not (r q) because the tutorial did it that way too)
	res[0] = y[1];
	res[1] = - g/l * sin(y[0]);

	/************************************/

	return res;
}


int main(int argc, char *argv[]) {

	//Initial values
	// y is already a 2D-vector, but this is because we have a differential equation of the order 2
	VectorXd y0  = VectorXd::Zero(2);
	//These are our initial value (therefore this is an initial value problem)
	y0[0] = 0.5*M_PI; 
	y0[1] = 0.0; 
	double t0 = 0.;

	double tf = 20.0; //runtime of the calculation / anomation
	double h = 0.01; //numeric stepwidth (between y_i and y_i+1
		
	double t = t0;
	VectorXd y = y0; //applying initial values

	std::ofstream of("animation.csv");
	
	//This part is the actual numeric calculated simulation of the movement
	while (t <= tf) {
		//printing q'?
		of << t << ", " << y[0] << "\n";
		
		//using Runge Kutta 4
		//y = y + h * rk4_integrator (t, y, h, rhs);
		//using heun
		//y = y + h * heun_integrator (t, y, h, rhs);
		//using euler
		y = y + h * euler_integrator (t, y, h, rhs);

		t = t + h;
	}

	of.close();
	return 0;
}
