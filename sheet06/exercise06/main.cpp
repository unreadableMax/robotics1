#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace Eigen;
const double g = 9.81;

// Define a few types to make it easier
typedef VectorXd (*rhsFuncPtr) (const double, const VectorXd&);
typedef VectorXd (*integratorFuncPtr) (const double, const VectorXd&, const double, rhsFuncPtr);

Model* pendulum(NULL); //before: pendulum(nulptr) - caused compiler error

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
	assert (y.size() == 4);

	VectorNd res (VectorNd::Zero(4));
	VectorNd q (VectorNd::Zero(2));
	VectorNd qd (VectorNd::Zero(2));
	VectorNd qdd (VectorNd::Zero(2));
	VectorNd tau (VectorNd::Zero(2));

	res[0] = y[2];
	res[1] = y[3];

	q[0] = y[0];
	q[1] = y[1];

	qd[0] = y[2];
	qd[1] = y[3];

	assert(pendulum);
	ForwardDynamics(*pendulum, q,qd,tau,qdd);

	res[2] = qdd[0];
	res[3] = qdd[1];

	return res;
}

// 2 new lines just to switch between different modes:
enum initial_value_mode { i, ii, iii, iv};
enum integrator_mode {rk4,euler,heun};

int main(int argc, char *argv[]) {

	// lets see what todo by looking into argv...
	initial_value_mode init_vals = i;
	integrator_mode integ_mode = rk4;	
	if(argc != 3){
		std::cerr << "naaa, do it like this: ./double_pendulum [initial_value_set(1-4)] [integrator(1-3)]" << std::endl;
		abort();
	}
	else{
		// i catch possible errors later at switch(...)...
		init_vals  = static_cast<initial_value_mode>(argv[1][0]-'0'-int(1));
		integ_mode = static_cast<integrator_mode>(argv[2][0]-'0'-int(1));
	}

	//load lua model which also contains all constants for our simulation
	pendulum = new Model;

	if (! RigidBodyDynamics::Addons::LuaModelReadFromFile( "double_pendulum.lua" , pendulum, false)) {
		std::cerr << "Error loading model - aborting" << std::endl;
		abort();
	}

	// 2 degrees of freedom (2 angles q_1 and q_2)
	assert(pendulum->dof_count == 2);

	VectorNd y0 (VectorNd::Zero(4));

	double t0 = 0.0;	//start time
	double tf = 20.0;	// end time
	double h = .001;	// time step
	
	// ----------- b) use i, ii, iii and iv as initial values ----------b)
	switch(init_vals)
	{

		case i: // stays never still. All integrators doing there jobs well
			y0[0] = M_PI*.5; 	// q_1
			y0[1] = 0.0;		// q_2
			y0[2] = 0.0;		// q_1'
			y0[3] = 0.0;		// q_2'
			std::cout<<"using i) as initial values"<<std::endl;
			break;

		case ii: // stays still with all integrators
			y0[0] = M_PI; 		// q_1
			y0[1] = M_PI;		// q_2
			y0[2] = 0.0;		// q_1'
			y0[3] = 0.0;		// q_2'
			std::cout<<"using ii) as initial values"<<std::endl;
			break;

		case iii: // drops down after a fiew secconds (~8) with all integrators 
			y0[0] = M_PI; 		// q_1
			y0[1] = 0.0;		// q_2
			y0[2] = 0.0;		// q_1'
			y0[3] = 0.0;		// q_2'
			std::cout<<"using iii) as initial values"<<std::endl;
			break;

		case iv: // stays still with all integrators. Smallest posible potential energy
			y0[0] = 0.0; 		// q_1
			y0[1] = 0.0;		// q_2
			y0[2] = 0.0;		// q_1'
			y0[3] = 0.0;		// q_2'
			std::cout<<"using iv) as initial values"<<std::endl;
			break;

		default: // stays still off cause. Smallest posible potential energy
			y0[0] = 0.0; 		// q_1
			y0[1] = 0.0;		// q_2
			y0[2] = 0.0;		// q_1'
			y0[3] = 0.0;		// q_2'
			std::cout<<"wrong imput. Using iv) as initial values"<<std::endl;
	}

	std::ofstream of("animation.csv");

	double t=t0;

	// first simulation results are y0
	VectorNd y = y0;

	// ----c)------------------ using different integrators -------------------------c)
	VectorXd (*integratorPtr)(const double t, const VectorXd &y, const double h, rhsFuncPtr rhs);

	switch(integ_mode)
	{
		case rk4:
			integratorPtr = rk4_integrator;
			std::cout<<"using rk4_integrator"<<std::endl;
			break;

		case euler:
			integratorPtr = euler_integrator;
			std::cout<<"using euler_integrator"<<std::endl;
			break;

		case heun:
			integratorPtr = heun_integrator;
			std::cout<<"using heun_integrator"<<std::endl;
			break;

		default:
			integratorPtr = rk4_integrator;
			std::cout<<"Wrong imput. Using rk4_integrator"<<std::endl;
	}
	
	// simulation loop:
	while (t <= tf) {
		y = y + h * integratorPtr (t, y, h, rhs);
		of << t << ", " << y[0] << ", " << y[1] << "\n";
		t = t + h;
	}

	of.close();
	return 0;
}
