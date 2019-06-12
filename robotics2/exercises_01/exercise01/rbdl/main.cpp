

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

MatrixNd CalcOrientationEulerXYZ( double x, double y, double z ) {
    return rotx(x) * roty(y) * rotz(z);
}


int main(int argc, char *argv[]) {

  	rbdl_check_api_version (RBDL_API_VERSION);
  
	//Initial values
	Model* humanoid = NULL;

    	//Load your model 
	humanoid = new Model();
	if (!Addons::LuaModelReadFromFile ("./humanoid_model.lua", humanoid, false)) {
    		std::cerr << "Error loading model " << std::endl;
    		abort();
  	}
	//prepare data structures and compute CoM
	double mass = 0;
    	Vector3d com;
    	int dof = humanoid->dof_count;

  	VectorNd q = VectorNd::Zero(dof);
  	VectorNd qdot = VectorNd::Zero(dof);
  	VectorNd Tau = VectorNd::Zero(dof);
  	VectorNd qddot = VectorNd::Zero(dof);

	Utils::CalcCenterOfMass(*humanoid,q,qdot,&qddot,mass,com);

	Vector3d l_foot;
	Vector3d r_foot;
	Vector3d pelv;

	pelv=CalcBodyToBaseCoordinates(*humanoid,q,humanoid->GetBodyId("pelvis"),Vector3d(0,0,0));

	r_foot=CalcBodyToBaseCoordinates(*humanoid,q,humanoid->GetBodyId("foot_right"),Vector3d(0,0,0));

	l_foot=CalcBodyToBaseCoordinates(*humanoid,q,humanoid->GetBodyId("foot_left"),Vector3d(0,0,0));

	Vector3d base=CalcBodyToBaseCoordinates(*humanoid,q,humanoid->GetBodyId("base_link"),Vector3d(0,0,0));

    	//compute kinematics 
	std::cout << "The humanoid robot has " << dof << " degrees of freedom and ways " << mass << " kilograms" << std::endl;
    	std::cout << "Center of mass position: " << com.transpose() << std::endl;

	double f_d = (l_foot - r_foot)[1];
	double p_h = (pelv - r_foot)[2]+0.04;
    	std::cout << "Feet distance: " << f_d << std::endl;
    	std::cout << "Pelvis height: " << p_h << std::endl;
	std::cout << "Pelvis: " << pelv << std::endl;

    
    	// Compute inverse kinematic
	InverseKinematicsConstraintSet CS;
    
    	// Change this parameter for infeasible configurations and see what happens
    	CS.lambda = 0.00001;

    	VectorNd q_init (VectorNd::Zero(humanoid->dof_count));
	q_init[12]=.5;
	VectorNd q_res = VectorNd::Zero(dof);
    
    	// Prepare data structures for Inverse kinematics

    	// Hint: unsigned int pelvis = CS.AddFullConstraint(....)
	unsigned int pelvis_id; 
	pelvis_id= CS.AddFullConstraint( humanoid->GetBodyId("pelvis"), Vector3d(0,0,0),Vector3d(0,0,0),CalcOrientationEulerXYZ(.0,.0,.0));
	
	CS.AddFullConstraint(humanoid->GetBodyId("foot_right"),Vector3d(.0,.0,-0.04),Vector3d(.0,-f_d/2.0,.0) ,CalcOrientationEulerXYZ( 0.0, 0.0, 0.0));

	CS.AddFullConstraint(humanoid->GetBodyId("foot_left"),Vector3d(.0,.0,-0.04),Vector3d(.0,f_d/2.0,.0), CalcOrientationEulerXYZ( 0.0, 0.0, 0.0));

	CS.AddOrientationConstraint(humanoid->GetBodyId("base_link"), CalcOrientationEulerXYZ( 0.0, 0.0, 0.0));



    	//Prepare animation output
    	std::ofstream of("animation.csv");
	std::ofstream ff("arrows.ff");

    // Run inverse kinematics and save animation
    for (unsigned int i = 0; i < 100; i ++){
        
	double A = 0.2;
        double Pelvis_height =  A * sin( M_PI * i / 10. ) -A*1.001;//prevent fully streched legs
        double Pelvis_angle = sin( M_PI * i / 10. );
        
        //override target position and orientation
        CS.target_positions[pelvis_id] = Vector3d(.0,.0,p_h) + Vector3d(0,0,Pelvis_height);
        CS.target_orientations[pelvis_id] = CalcOrientationEulerXYZ( 0.0, 0.0, Pelvis_angle);
        
        //calculate IK6
        bool ret = InverseKinematics(*humanoid, q_init, CS, q_res);
	
        if (!ret) {
            std::cout << "InverseKinematics did not find a solution at " << static_cast<double>(i)/10.0<< " s"<< std::endl;
        }
        
        //compute CoM for animation
	qddot = (q_res - q_init) - qdot;
	qddot/= .1*.1;
	qdot = (q_res - q_init)/.1;
	Utils::CalcCenterOfMass(*humanoid,q_res,qdot,&qddot,mass,com);
	q_init = q_res;
        of << i / 10. << ", ";
        ff << i / 10. << ", ";
        for (unsigned int j = 0; j < dof; j++){
            of << q_res[j] << ", ";
        }
        of << i << ",\n";
        ff << com[0] << ", " << com[1] << ", " << com[2] << ", 1000, 0, 0, 0, 0, 0\n";
        
    	}
    
	of.close();

	return 0;
}
