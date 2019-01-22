#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <fstream>
#include <vector>
#include <string>

#include "rampfunction.h"
#include "simpletests.h"

#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_LUAMODEL
	#error "Error: RBDL addon LuaModel not enabled."
#endif
#include <rbdl/addons/luamodel/luamodel.h>
using namespace RigidBodyDynamics;
using namespace Eigen;


// Define a few types to make it easier
typedef Matrix<double, 6, 1>  Vector6d;

// Read why this has to be done here:
//  http://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
typedef std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > Vector6dVector ;
typedef std::vector<Vector3d, Eigen::aligned_allocator<Vector3d> > Vector3dVector ;

struct datarow_t {
	double t;
	Vector6d q;
};

typedef std::vector<datarow_t> Data ;


/* Put your 2d coordinates to the drawing in here*/

Vector3dVector endPoints(
	{
		Vector3d(0,0,.5),
	
		Vector3d(0,0,0),
		Vector3d(0,1,0),
		Vector3d(.5,2,0),
		Vector3d(1,1,0),
		Vector3d(0,1,0),
		Vector3d(1,0,0),
		Vector3d(0,0,0),
		Vector3d(1,1,0),
		Vector3d(1,0,0),

		Vector3d(1,0,.5),
	});

/* Transformation from the local to the global system */

class Transformation {
	private:
	double scale;
	Vector3d translate;
	Eigen::Matrix3d M;
	
	public:
	Transformation() {
		//Some default values
		scale =0.1;
		translate = Vector3d(0.5,0,0.0);
		//~ M = Eigen::AngleAxisd(0.5*M_PI, Vector3d::UnitY());
		M = Eigen::Matrix3d::Identity();
	}

	bool calibrate(Vector3d A,Vector3d B,Vector3d C) {
		Vector3d AB = B - A;
		Vector3d AC = C - A;
		Vector3d n = AB.cross(AC); //normal
		//Test, if the given values make sense
		if (AB.norm() <1E-10 || AC.norm() < 1E-10|| n.norm() < 1E-10)
			return false;
		//Put your code here and replace the fixed values
		/*
		scale = .1;
		translate = Vector3d(.5,0,0);;
		Vector3d x = Vector3d(1,0,0);
		Vector3d z = Vector3d(0,0,1);
		Vector3d y = Vector3d(0,1,0);
		//*/

		// my code:
		///*
		scale = AB.norm();
		translate = A;
		Vector3d x = AB;
		x.normalize();
		Vector3d z = x.cross(AC);
		z.normalize();
		Vector3d y = z.cross(x);
		y.normalize();
		//*/
		std::cout<<x<<std::endl<<std::endl<<y<<std::endl<<std::endl<<z<<std::endl<<std::endl;
		
		M <<	x[0], y[0], z[0], 
			x[1], y[1], z[1], 
			x[2], y[2], z[2];
		return true;
	}


	Vector3d transform(Vector3d in) {
		return M * in*scale + translate;
	}

	Matrix3d pointingMatrix() {
		Matrix3d Mz = Matrix3d::Identity() * Eigen::AngleAxisd(0.5*M_PI, Vector3d::UnitY());
		return   (M *  Mz).transpose();
	}
	
};

/* Make calling the inverse kinematics a little easier */
bool invKin(Model& model, Vector3d pos, Matrix3d ori, Vector6d& q) {
	int tcp_id = model.GetBodyId("TCP");
	InverseKinematicsConstraintSet cs;
	cs.AddPointConstraint (tcp_id,Vector3d(0,0,0), pos);
	cs.AddOrientationConstraint  (tcp_id, ori);

	// run Inverse Kinematic
	Eigen::VectorXd q_start = q;
	Eigen::VectorXd q_new;
	bool ret = InverseKinematics(model, q_start, cs, q_new);
	if (ret)
		q=q_new;
	return ret;
}

/* PTP Motion */
Vector6dVector PTP(Vector6d q_start, Vector6d q_end, double dt = 0.01, double a = 1, double vmax = 2.) {
	Vector6dVector l;
	
	Vector6d dq = q_end - q_start;
	double longest =0;
	for (int i =0; i < 6; i++) {
		longest = std::max(longest, std::abs(dq[i]));
	}
	if (longest <= 0){
		l.push_back(q_start);
		return l;
	}
	
	RampFunction rf(longest,a,vmax);
	double t_end = rf.getEndTime();
	for (double t=0; t < t_end; t+=dt) {
		Vector6d q = q_start + dq*(rf.S(t)/std::abs(longest));
		l.push_back(q);
		
	}
	
	return l;
}

/*LIN motion */
// Put your code in this function
Vector6dVector LIN(Model& model, Vector3d p_start, Vector3d p_end, Matrix3d ori,Vector6d q_start, double dt = 0.01, double a = .15, double vmax = .1) {

	//------------------------------
	Vector3d dp = p_end - p_start;
	double longest =0;
	for (int i =0; i < 3; i++) {
		longest = std::max(longest, std::abs(dp[i]));
	}
	RampFunction rf(longest,a,vmax);
	//--------------------------------
	
	Vector6dVector l;
	
	//double t_end = 1.5;
	//------------------------------
	double t_end = rf.getEndTime();
	//---------------------------
	Vector6d q = q_start;
	for (double t=0; t < t_end; t+=dt) {
		//Linear interpolation without a ramp
		// Put your code in this function
		//Vector3d pos = t/t_end*(p_end - p_start)+p_start;
		//------------------------
		Vector3d pos = p_start + dp*(rf.S(t)/std::abs(longest));
		//-------------------------------
			
		bool ret = invKin(model, pos, ori,q);
		if (!ret) {
			std::cout << "InverseKinematics did not find a solution" << std::endl;
		} else {
			l.push_back(q);
		}
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

void appendMotion(const Vector6dVector& motion, Data& data, double& t, double dt) {
	for (std::size_t j = 0; j < motion.size(); ++j) {
		datarow_t dr;
		dr.t = t;
		t+=dt;
		dr.q = motion[j];
		data.push_back(dr);
	}
}


int main(int argc, char *argv[]) {
	
	// lets see what todo by looking into argv...
	int calibration_set=0;
	if(argc != 2){
		//std::cerr << "please select calibration set" << std::endl;
		//abort();
		calibration_set=-1;
	}
	else{
		// i catch possible errors later at switch(...)...
		calibration_set  = static_cast<int>(argv[1][0]-'0');
		std::cout<<"calibration_set="<<calibration_set<<std::endl;
	}

	
	Model model;
	if (!Addons::LuaModelReadFromFile ("kuka.lua", &model, false)) {
		std::cerr << "Error loading lua file" << std::endl;
		abort();
	}
	
	Data data;
	double t = 0.0;
	double dt = 0.01;
	Vector6d q_home;
	q_home << 0,-M_PI/2.,M_PI/2.,0,0,0;
	Vector6d q_start = q_home;

	Transformation tr;

	// Different calibrations to play with
	//----------------------------------------------
	switch(calibration_set)
	{
		case 0:
			assert(tr.calibrate(Vector3d(0.5,0,0),Vector3d(0.6,0.0,0),Vector3d(0.5,0.1,0)));
			break;

		case 1:
			assert(tr.calibrate(Vector3d(0.7,0,.2),Vector3d(0.7,-0.1,.2),Vector3d(0.7,0.,0.3)));
			break;

		case 2:
			assert(tr.calibrate(Vector3d(0.5,0,0.2),Vector3d(0.6,0.0,0.2),Vector3d(0.5,0.,0.3)));
			break;
		case 3:
			assert(tr.calibrate(Vector3d(0.5,0,.2),Vector3d(0.6,0.0,.2),Vector3d(0.5,0.1,0.3)));
			break;

		default:
			std::cout<<"no calibration"<<std::endl;
	}
	//-------------------------------------------------------

	Matrix3d ori = tr.pointingMatrix();

	assert(endPoints.size() > 0);

	//-----------------------------------------------------------------
	// first: lets create a PTP motion from q_home to endPoints(0)
	Vector3d p_first_draw = tr.transform(endPoints[0]);
	Vector6d q_first_draw = q_start;
	bool ret = invKin(model, p_first_draw, ori,q_first_draw);
	if (!ret) 
		std::cout << "InverseKinematics did not find a solution" << std::endl;
	
	Vector6dVector start_motion = PTP(q_start, q_first_draw, dt);
	appendMotion(start_motion,data,t,dt);
	//-----------------------------------------------------------

	//LIN Motion(draw a picture)	
	for (size_t i=0; i < endPoints.size() - 1; i++) {
		Vector3d start = tr.transform(endPoints[i]);
		Vector3d end = tr.transform(endPoints[(i+1)]);
		Vector6dVector motion = LIN(model, start, end,ori, q_start, dt);
		appendMotion(motion,data,t,dt);
	}

	//-------------------------------------------------------------
	//move from the finished picture to home pose.
	Vector3d p_last_draw = tr.transform(endPoints[endPoints.size() - 1]);
	Vector6d q_last_draw=q_start;
	ret = invKin(model, p_last_draw, ori,q_last_draw);
	if (!ret) 
		std::cout << "InverseKinematics did not find a solution" << std::endl;
	
	Vector6dVector end_motion = PTP(q_last_draw, q_start, dt);
	appendMotion(end_motion,data,t,dt);
	//------------------------------------------------------------------

	writeFile("animation.csv", data);
	
	return 0;
}
