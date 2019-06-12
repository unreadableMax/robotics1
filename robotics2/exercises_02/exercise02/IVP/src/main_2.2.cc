#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <sstream>

#include "SimpleMath/SimpleMath.h"

using namespace std;

typedef VectorNd (*rhsFuncPtr) (const double, const VectorNd&);
typedef VectorNd (*integratorFuncPtr) (const double, const VectorNd&, const double, rhsFuncPtr);

/** \brief Delete all contents of an existing file, i.e. truncate it so
 * that it is empty. */
void reset_file (const char *filename) {
	ofstream output_file (filename, ios::trunc);

	if (!output_file) {
		cerr << "Error: could not reset file " << filename << "." << endl;
		abort();
	}
	output_file.close();
}

/** \brief Open and append data to the end of the file */
void append_data (const char *filename, double t, const VectorNd &values) {
	ofstream output_file (filename, ios::app);

	if (!output_file) {
		cerr << "Error: could not open file " << filename << "." << endl;
		abort();
	}

	output_file << scientific;
	output_file << t << ", ";
	for (unsigned int i = 0; i < values.size(); i++) {
		output_file << values[i];

		if (i != values.size() - 1)
			output_file << ", ";
	}
	output_file << endl;
	output_file.close();
}

VectorNd rhs_func (double t, const VectorNd &y) {
	assert (y.size() == 1);

	unsigned int dim = y.size();
	VectorNd res (VectorNd::Zero(dim));

    res[0] = -200.0*t*y[0]*y[0];
    //res[0] = y[0];

	return res;
}

VectorNd rk4_integrator (const double t, const VectorNd &y, const double h, rhsFuncPtr rhs) {
    //implement rk4_integrator----------
    VectorNd k1 = rhs (t, y);
    VectorNd k2 = rhs (t + (double) 0.5 * h, y + (double) 0.5 * h * k1);
    VectorNd k3 = rhs (t + (double) 0.5 * h, y + (double) 0.5 * h * k2);
    VectorNd k4 = rhs (t + h, y + h * k3);

    return (double) 1. / 6. * (k1 + (double) 2. * k2 + (double) 2. * k3 + k4);
}

VectorNd euler_integrator (const double t, const VectorNd &y, const double h, rhsFuncPtr rhs) {
    //implement euler_integrator--------
    return rhs (t, y);;
}
double exact (double t) {
	return 1. / (1. + 100 * t * t);
}

VectorNd simulate (const double t0, const VectorNd &y0, const double tf, const double h, rhsFuncPtr rhs, integratorFuncPtr integrator, const string foutput) {
    double t = t0;
    VectorNd y = y0;
    VectorNd y_out = VectorNd::Zero(2);
    reset_file (foutput.c_str());

    // do not write each single computation step
    unsigned int total_steps = (unsigned int)(tf-t)/h;
    cout<<"steps = "<<total_steps;
    int max_steps = 200;
    int writing_step = total_steps/max_steps;
    if(writing_step == 0)
        writing_step =1;
    int step = writing_step;
    cout <<"writing each "<<writing_step<<endl;

    while (t < tf) {

        // implement single step method
        y_out[0] = y;
        y_out[1] = exact(t);

        y = y+h*integrator(t,y,h,rhs);
        t = t + h;

        // save data
    if(step==writing_step){
        append_data (foutput.c_str(), t, y_out);
        step=0;
    }
    else
        step++;

    }

    return y;
}


int main(int argc, char* argv[]) {
	VectorNd y0 (VectorNd::Zero(1));
	VectorNd yf;

	double t0 = -3.;
	double tf = 1.0;
	y0[0] = 1. / 901.;
    cout<<"starting now..."<<endl;
	for (int i = 1; i < 6; i++) {
	  stringstream outputfileStr;
	  outputfileStr << "output-2-" << i << ".csv";
		double h = pow (10, -i);
		yf = simulate (t0, y0, tf, h, rhs_func, rk4_integrator, outputfileStr.str());
		cout << "h = " << h << "\teta(tf) = " << yf[0] << "\ty(tf) = " << exact(tf) << "\terror = " << yf[0] - exact(tf) << endl;
	}

	return 0;
}