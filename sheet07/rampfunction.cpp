#include <assert.h>
#include <iostream>
#include <cmath>

#include "rampfunction.h"

RampFunction::RampFunction(double s, double a, double vmax) :  t_end(0), t1(0), t2(0)  {
	this->s = s;
	this->a = a;
	this-> vmax = vmax;

	double scrit = (vmax*vmax/a);
	
	
	// Compute here, t1,t2 and t_end
	
	double sabs = std::abs(s);
	if ((vmax*vmax/a) > sabs) { //case b
		// s is too short, will not reach vmax

		t1 = sqrt(sabs/a);
		t2 = t1;
		t_end = 2*t1;
		
	} else { // (vmax*vmax/a) <= sabs -> case a
		//reaching vmax;
		// will flat 

		t1 = vmax/a;
		t2 = (sabs-scrit)/(vmax) + t1;
		t_end = t2+t1;
	}
}


double RampFunction::S(double t) {
	assert(t<=t_end);// restrict t to t_end?
	
	if(t>t2){
		return a*t1*t1 + (t2-t1)*a*t1 - .5*a*(t_end-t)*(t_end-t);
	} 
	else if(t>t1){
		return .5*a*t1*t1 + (t-t1)*a*t1;
	} 
	else if(t>=0){
		return .5*a*t*t;
	}	

	return 0; // <<--
}


RampFunction::~RampFunction() {
}


double RampFunction::getEndTime() {
	return t_end;
}
double RampFunction::getT1() {
	return t1;
}
double RampFunction::getT2() {
	return t2;
}
