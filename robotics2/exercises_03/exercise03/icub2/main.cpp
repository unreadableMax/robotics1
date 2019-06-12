#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// Define a few types to make it easier
typedef VectorNd (*rhsFuncPtr) (const VectorNd&, const VectorNd&);
typedef VectorNd (*integratorFuncPtr) (const VectorNd&, const VectorNd&, const double, rhsFuncPtr);

MatrixNd CalcOrientationEulerXYZ( double x, double y, double z ) {
    return rotx(x) * roty(y) * rotz(z);
}

Model humanoid;

VectorNd rk4_integrator (const VectorNd &x, const VectorNd &u, const double h, rhsFuncPtr rhs) {

    VectorNd one_vec = VectorNd::Ones(x.size());

    // ===>>> AKS: how does it work without time?
    VectorNd k1 = rhs (x, u);
    VectorNd k2 = rhs (x +  0.5 * h * k1, u);
    VectorNd k3 = rhs (x +  0.5 * h * k2, u);
    VectorNd k4 = rhs (x + h * k3, u);

    return (double) 1. / 6. * (k1 + (double) 2. * k2 + (double) 2. * k3 + k4);
    //return k1;//like euler

}

//How many points do you need to constrain for a 2D model? ---> 2 damit fuß auch wagerecht bleibt???
VectorNd rhs_constraint_set_feet (const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2*humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());
    
    //update robot states 
    VectorNd q = VectorNd::Zero(humanoid.dof_count);
    VectorNd qdot = VectorNd::Zero(humanoid.dof_count);
    VectorNd qddot = VectorNd::Zero(humanoid.dof_count);
    VectorNd tau = VectorNd::Zero(humanoid.dof_count);
    
    for (unsigned int i = 0; i < humanoid.dof_count; i++) {
        q[i] = x[i];
        qdot[i] = x[i + humanoid.dof_count];
    }
    //6 of 9 dof can be actuated...
    for (unsigned int i = 0; i < humanoid.dof_count -3; i++) {
        tau[i+3] = u[i];
    }
            
    
    int left_foot = humanoid.GetBodyId("l_sole");
    int right_foot = humanoid.GetBodyId("r_sole");

    ConstraintSet fix_feet;

    //--- Add your constraints here-------------------
    Vector3d front,back;
    front = Vector3d(.1,.0,.0);
    back = Vector3d(-.1,.0,.0);

    fix_feet.AddContactConstraint(right_foot,front,Vector3d(0,0,1));
    fix_feet.AddContactConstraint(right_foot,back,Vector3d(0,0,1));
    fix_feet.AddContactConstraint(right_foot,front,Vector3d(1,0,0));
    fix_feet.AddContactConstraint(right_foot,back,Vector3d(1,0,0));

    fix_feet.AddContactConstraint(left_foot,front,Vector3d(0,0,1));
    fix_feet.AddContactConstraint(left_foot,back,Vector3d(0,0,1));
    fix_feet.AddContactConstraint(left_foot,front,Vector3d(1,0,0));
    fix_feet.AddContactConstraint(left_foot,back,Vector3d(1,0,0));
    fix_feet.Bind(humanoid);

    //compute ForwardDynamics
    ForwardDynamicsConstraintsDirect(humanoid,q,qdot,tau,fix_feet,qddot);
    
    for (unsigned int i = 0; i < humanoid.dof_count; i++) {
        res[i] = qdot[i];
        res[i + humanoid.dof_count] = qddot[i];
    }
    
    return res;
}


int main(int argc, char *argv[]) {
    //load model
	if (! RigidBodyDynamics::Addons::LuaModelReadFromFile( "../models/iCubHeidelberg01.lua" , &humanoid, false)) {
		std::cerr << "Error loading model - aborting" << std::endl;
		abort();
	}
	//get DoF
    int dof = humanoid.dof_count;
    int actuated_dof = dof - 3;

    // by me:
    double mass = 0;
    Vector3d com;
    //Utils::CalcCenterOfMass(humanoid,VectorNd::Zero(dof),VectorNd::Zero(dof),VectorNd::Zero(dof),mass,com);
	
    //compute feet distance and pelvis hight
    VectorNd q (VectorNd::Zero(dof));
    Vector3d r_foot =   CalcBodyToBaseCoordinates(humanoid,q,humanoid.GetBodyId("r_sole"),Vector3d(0,0,0));
    Vector3d l_foot =   CalcBodyToBaseCoordinates(humanoid,q,humanoid.GetBodyId("l_sole"),Vector3d(0,0,0));
    Vector3d pelv   =   CalcBodyToBaseCoordinates(humanoid,q,humanoid.GetBodyId("chest"),Vector3d(0,0,0));

    
    double feet_dist = (r_foot - l_foot)[1];
    double pelvis_hight = (pelv - r_foot)[2];

	InverseKinematicsConstraintSet CS;
    
    // Change this parameter for infeasible configurations
    CS.lambda = .001;
    VectorNd q_init (VectorNd::Zero(dof));
    VectorNd q_res (VectorNd::Zero(dof));
    
    // Set Bent knees in the same direction
    q_init[3] = 0.45;
    q_init[4] = -0.7;
    q_init[5] = 0.4;
     
    q_init[6] = -0.45; //-0.48
    q_init[7] = -0.7;
    q_init[8] = -0.4;
    
    // Add target positions
    Vector3d R_foot_pos = Vector3d(r_foot[0],feet_dist/2.0,.0); // Vector3d(.0,feet_dist/2.0,.0);
    Vector3d L_foot_pos = Vector3d(l_foot[0],-feet_dist/2.0,.0);
    Vector3d Pelvis_pos = Vector3d(pelv[0],pelv[1],pelvis_hight - .02);
    Vector3d Pelvis_pos_init = Vector3d(pelv[0],pelv[1],0.4); //"40cm above ground"
    
    //model feet are turned
    Matrix3d R_foot_ort = CalcOrientationEulerXYZ(0,0,M_PI);
    Matrix3d L_foot_ort = CalcOrientationEulerXYZ(0,0,M_PI);
    
    //add constrains
    CS.AddFullConstraint(humanoid.GetBodyId("r_sole"),Vector3d(.0,.0,0.0), R_foot_pos,R_foot_ort);
    CS.AddFullConstraint(humanoid.GetBodyId("l_sole"),Vector3d(.0,.0,0.0), L_foot_pos,L_foot_ort);
    unsigned int pelv_cs_id = CS.AddPointConstraint(humanoid.GetBodyId("chest"), Vector3dZero, Pelvis_pos_init);
    //unsigned int pelv_cs_id = CS.AddFullConstraint(humanoid.GetBodyId("chest"),Vector3d(.0,.0,0.0), Pelvis_pos_init,CalcOrientationEulerXYZ(.0,.0,M_PI));
    //CS.AddPointConstraint(humanoid.GetBodyId("r_sole"), Vector3dZero, R_foot_pos);
    //CS.AddPointConstraint(humanoid.GetBodyId("l_sole"), Vector3dZero, L_foot_pos);

    // aus exercise01
    //compute IK for beginning and end pose
    bool ret = InverseKinematics(humanoid, q_init, CS, q_res);
    if (!ret) {
        std::cout << "first InverseKinematics did not find a solution" << std::endl;
    }

    q_init = q_res;

    //Update target position and run IK again
    CS.target_positions[pelv_cs_id] = Pelvis_pos;
    
    ret = InverseKinematics(humanoid, q_init, CS, q_res);
    if (!ret) {
        std::cout << "second InverseKinematics did not find a solution" << std::endl;
    }
    
    //save animation
    std::ofstream an("animation.csv");
    an << 0. << ", ";
    for (int j = 0; j < dof; j++){
        an << q_init[j] << ", ";
    }
    an << "\n";
    an << .4 << ", ";
    for (int j = 0; j < dof; j++){
        an << q_res[j] << ", ";
    }
    an << "\n";
    cout<<"animation.csv generated with inverse kinematics"<<endl;

    //compute ForwardDynamics with constraints
    // define start and end position
    VectorNd q_start = q_init;
    VectorNd q_end = q_res;
    
	// States: x = [ q q_dot ]
    VectorNd x (VectorNd::Zero(2 * dof));
    
    // Control vector u = [ tau ]
    VectorNd u = VectorNd::Zero(actuated_dof);
    
	double t = 0.0;
	double tf = 0.4;//original value: .4
	double h = 0.00001;
    
    //close enough initial guess
    //double tau_hip = 2.8,tau_leg = 19,tau_ankle = 3.5, s=1.1;
    double tau_hip = 2.6,tau_leg = 20.7,tau_ankle = 4.2, s=1.0;
    u[0] = -tau_hip*s;     //l_hip_1           3
    u[1] = tau_leg*s;     //l_lower_leg       4
    u[2] = -tau_ankle*s;  //l_ankle_1         5
    u[3] = tau_hip*s;     //r_hip_1           6
    u[4] = tau_leg*s;     //r_lower_leg       7
    u[5] = tau_ankle*s;   //r_ankle_1         8

	std::ofstream of("computed_motion.csv");
    
    //iterations for single shooting, set to 0 for only one forward simulation
    int n = 5;
    for (unsigned int i = 0; i <= n; i++){
        //reset data:
        t = 0;
        x = VectorNd::Zero(2 * dof);
        for (unsigned int j = 0; j < dof; j++){
            x[j] = q_start[j];
            x[j + dof] = 0;     // speed =0
        }
        
        //perform single shot
        while (t <= tf) { 
            x = x + h * rk4_integrator (x, u, h, rhs_constraint_set_feet);
            t = t + h;
            
            //save result after last iteration
            if (i == n){
                of << t << ", ";
                for (unsigned int j = 0; j < dof; j++){
                    of << x[j] << ", ";
                }
                of << "\n";

            }
        }
        
        //compute difference of resulting position to target
        VectorNd q = VectorNd::Zero(dof);
        for (unsigned int j = 0; j < dof; j++){
            q[j] = x[j];
        }
        VectorNd dif = q-q_end;

        //--- implement update for u

        //print difference
        cout << "Difference to target: "<<dif.norm()<<endl;

        double lambda = .1;
        //double lambda = dif.norm();
        for (unsigned int j = 0; j < actuated_dof; j++){
            u[j] = u[j]-lambda*dif[j+3];
        }
        //cout << "Torques: " << u.transpose() << endl;
    }
    cout << "saved torque-based animation"<<endl;
    an.close();
	of.close();
	return 0;
}