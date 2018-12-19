#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <fstream>
#include <vector>
#include <string>

#include "rampfunction.h"
#include "simpletests.h"

using namespace Eigen;


void runtests() {
	std::cout << "===================================================\n";
	{
		RampFunction r(1,10,1);
		test_close(r.getT1(),0.1);
		test_close(r.getT2(),1.);
		test_close(r.getEndTime(),1.1);
		test_close(r.S(0),0);
		test_close(r.S(0.25*r.getEndTime()),0.225);
		test_close(r.S(0.75*r.getEndTime()),0.775);
		test_close(r.S(0.5*r.getEndTime()),0.5);
		test_close(r.S(r.getEndTime()),1);
	}
	{
		RampFunction r(100,2,1);
		test_close(r.getT1(),0.5);
		test_close(r.getT2(),100.);
		test_close(r.getEndTime(),100.5);
		test_close(r.S(0),0);
		test_close(r.S(0.25*r.getEndTime()),24.875);
		test_close(r.S(0.75*r.getEndTime()),75.125);
		test_close(r.S(0.5*r.getEndTime()),50);
		test_close(r.S(r.getEndTime()),100);
	}
	{
		
		RampFunction r(1,2,1);
		test_close(r.getT1(),0.5);
		test_close(r.getT2(),1.);
		test_close(r.getEndTime(),1.5);
		test_close(r.S(0),0);
		test_close(r.S(0.5*r.getEndTime()),0.5);
		test_close(r.S(r.getEndTime()),1);
	}
	
	std::cout << "===================================================\n";
}

// Define a few types to make it easier
typedef Matrix<double, 6, 1>  Vector6d;

// Read why this has to be done here:
//  http://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
typedef std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > Vector6dVector ;

struct datarow_t {
	double t;
	Vector6d q;
};

typedef std::vector<datarow_t> Data ;



Vector6dVector PTP(Vector6d q_start, Vector6d q_end, double dt = 0.01, double a = 1, double vmax = 2.) {
	//Vector6dVectorl; // here i found a little syntax error
	Vector6dVector l;

	Vector6d dq = q_end-q_start;
	
	double longestDistance =0; // <<---
	
	// compute longestDistance
	for(int i=0;i<6;i++){
		if(dq(i)>longestDistance)
			longestDistance = dq(i);
	}
	
	RampFunction rf(longestDistance,a,vmax);
	double t_end = rf.getEndTime();
	
	//Interpolate here
	for (double t=0; t < t_end; t+=dt) {
		
		Vector6d q = q_start; // <<---
		//for(int i=0;i<6;i++){
		//	q(i)+=dq(i)*(rf.S(t)/longestDistance);
		//}
		q+=dq*(rf.S(t)/longestDistance);

		l.push_back(q);
	}
	return l;
}


void writeFile(std::string filename, Data& data) {
	std::ofstream outfile(filename.c_str());
	if (!outfile.good()) {
		std::cerr << "Could not open '" << filename << "'" << std::endl;
		
	}
	for (size_t i =0 ; i < data.size(); i++) {
		outfile << data[i].t ;
		for(int j=0; j < 6; j++) {
			outfile << ", " << data[i].q[j];
		}
		outfile << "\n";
	}
	
	outfile.close();
}




int main(int argc, char *argv[]) {

	runtests();

	//predefine poses to motion
	Vector6dVector poses;
	Vector6d q;
	q << 0,0,0,0,0,0;			//moving from this
	poses.push_back(q);
	q << -M_PI/4,-M_PI/2,M_PI/2,0,0,0;	// to this
	poses.push_back(q);
	q <<  M_PI/4,-M_PI/2,M_PI/2,0,0,0;	// to this
	poses.push_back(q);
	q <<  .2,-1,1.1,-.5,1,2.5;		// to this
	poses.push_back(q);
	
	
	Data data;
	double t = 0.0;
	double dt = 0.01;
	//Go through the poses
	for (size_t i=0; i < poses.size() ; i++) {
		//Interpolate poses
		Vector6dVector motion = PTP(poses[i],poses[(i+1) % poses.size() ],dt);
		for (std::size_t j = 0; j < motion.size(); ++j) {
			datarow_t dr;
			dr.t = t;
			t+=dt;
			dr.q = motion[j];
			data.push_back(dr);
		}
	}
	
	
	writeFile("animation.txt", data);
	
	return 0;
}
