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

Model humanoid;

void get_q_qd(const VectorNd& x, VectorNd& q,  VectorNd& qd){
    for(int i=0;i<humanoid.dof_count;i++){
        q[i] = x[i];
        qd[i] = x[i+humanoid.dof_count];
    }
}

void get_res(VectorNd& res, const VectorNd& qd, const VectorNd& qdd){
    for(int i=0;i<humanoid.dof_count;i++){
        res[i] = qd[i];
        res[i+humanoid.dof_count] = qdd[i];
    }
}

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

//Implement a RHS function that returns the derivative of the state vector ẋ = [ q̇, q̈]
VectorNd rhs (const VectorNd &x, const VectorNd &u) {

    assert (x.size() == 2*humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());    // ẋ = [ q̇, q̈]

    //allocate robot states update kinematics
    VectorNd q = VectorNd::Zero(humanoid.dof_count);
    VectorNd qd = q;
    VectorNd qdd = q;

    get_q_qd(x,q,qd);

    //compute ForwardDynamics
    ForwardDynamics(humanoid,q,qd,VectorNd::Zero(humanoid.dof_count),qdd);

    //copy to res vector
    get_res(res,qd,qdd);

    return res;
}

VectorNd rhs_contact_force (const VectorNd &x, const VectorNd &u) {

    assert (x.size() == 2*humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());    // ẋ = [ q̇, q̈]

    //allocate robot states update kinematics
    VectorNd q = VectorNd::Zero(humanoid.dof_count);
    VectorNd qd = q;
    VectorNd qdd = q;

    get_q_qd(x,q,qd);

    //set up external forces use RBDL API
    std::vector< SpatialVector > fext(humanoid.mBodies.size());
    fext.resize(humanoid.dof_count);


    int body_Id = humanoid.GetBodyId("root_link");

    //compute forces and torque
    double g = humanoid.gravity[2];
    double m;
    Vector3d com;
    Utils::CalcCenterOfMass(humanoid,q,qd,&qdd,m,com);
    double actio_force = -m*g;
    Vector3d force(0, 0,actio_force) ;
    Vector3d pos = CalcBodyToBaseCoordinates(humanoid,q,body_Id,Vector3d::Zero());
    Vector3d torque = VectorCrossMatrix(pos)*force;

    for (int i = 0; i < fext.size(); i++) {
        fext[i].setZero();
    }

    fext[body_Id][0]=torque[0];
    fext[body_Id][1]=torque[1];
    fext[body_Id][2]=torque[2];
    fext[body_Id][3]=force[0];
    fext[body_Id][4]=force[1];
    fext[body_Id][5]=force[2];


    //compute ForwardDynamics
    ForwardDynamics(humanoid,q,qd,VectorNd::Zero(humanoid.dof_count),qdd, &fext);

    get_res(res,qd,qdd);

    return res;
}


//• Add joint friction and observe the effect. Hint: The friction force is proportional to the
//        joint velocity.
//• Add symmetric torques to the hip joints and observe the motions. Does this look as if
//iCub was fixed in space?
//• How do the motions look if we instead of using an external force set the gravity to zero?
 VectorNd rhs_contact_force_damping (const VectorNd &x, const VectorNd &u) {
     assert (x.size() == 2*humanoid.dof_count);
     VectorNd res = VectorNd::Zero(x.size());    // ẋ = [ q̇, q̈]

     //allocate robot states update kinematics
     VectorNd q = VectorNd::Zero(humanoid.dof_count);
     VectorNd qd = q;
     VectorNd qdd = q;
     get_q_qd(x,q,qd);

     //set up external forces use RBDL API
     int body_amount = humanoid.mBodies.size();
     std::vector< SpatialVector > fext(body_amount);
     //fext.resize(humanoid.dof_count);

     int body_Id = humanoid.GetBodyId("root_link");

     //compute forces and torque
     double g = humanoid.gravity[2];
     double m;
     Vector3d com;
     Utils::CalcCenterOfMass(humanoid,q,qd,&qdd,m,com);
     double actio_force = -m*g;
     Vector3d force(0, 0,actio_force) ;
     Vector3d com_root_link = Vector3d(-0.0529681, -4e-05, -0.0005388);
     Vector3d pos = CalcBodyToBaseCoordinates(humanoid,q,body_Id,Vector3d::Zero())+com_root_link;
     Vector3d torque = VectorCrossMatrix(pos)*force;

     for (int i = 0; i < fext.size(); i++) {
         fext[i].setZero();
     }

     //apply force to root:
     fext[body_Id][3]=force[0];
     fext[body_Id][4]=force[1];
     fext[body_Id][5]=force[2];

     //apply torque to hip-bodys... hip_3 returns a weird id...
     double sym_torque = .0;
     fext[humanoid.GetBodyId("l_hip_1")][2]=sym_torque;
     fext[humanoid.GetBodyId("l_hip_2")][2]=sym_torque;
     //fext[humanoid.GetBodyId("l_hip_3")][2]=sym_torque;
     fext[humanoid.GetBodyId("r_hip_1")][2]=-sym_torque;
     fext[humanoid.GetBodyId("r_hip_2")][2]=-sym_torque;
     //fext[humanoid.GetBodyId("r_hip_3")][2]=sym_torque;

     //friction torque:
     VectorNd q_fric_torque = -qd*0.001; //to high or low -> meshup crashes

     //compute ForwardDynamics      VectorNd::Zero(humanoid.dof_count)
     ForwardDynamics(humanoid,q,qd,q_fric_torque,qdd, &fext);

     get_res(res,qd,qdd);

     return res;
}

VectorNd rhs_constraint_set (const VectorNd &x, const VectorNd &u) {
    assert (x.size() == 2*humanoid.dof_count);
    VectorNd res = VectorNd::Zero(x.size());

    //allocate robot states update kinematics
    VectorNd q = VectorNd::Zero(humanoid.dof_count);
    VectorNd qd = q;
    VectorNd qdd = q;
    get_q_qd(x,q,qd);
    
    //create constraints here
    int body_Id = humanoid.GetBodyId("root_link");
    ConstraintSet fixed_root_link;
    // -----------> how can i fix the root in all directions?
    fixed_root_link.AddContactConstraint(body_Id,Vector3d::Zero(),Vector3d(0,0,1));
    fixed_root_link.AddContactConstraint(body_Id,Vector3d::Zero(),Vector3d(0,1,0));
    fixed_root_link.AddContactConstraint(body_Id,Vector3d::Zero(),Vector3d(1,0,0));

    fixed_root_link.Bind(humanoid);

    //friction-torque:
    VectorNd q_fric_torque = -qd*0.5;

    //compute ForwardDynamics
    ForwardDynamicsConstraintsDirect(humanoid,q,qd,q_fric_torque,fixed_root_link,qdd);

    get_res(res,qd,qdd);

    //copy to res vector
    return res;
}


int main(int argc, char *argv[]) {

	if (! RigidBodyDynamics::Addons::LuaModelReadFromFile( "../models/iCubHeidelberg01.lua" , &humanoid, false)) {
		std::cerr << "Error loading model - aborting" << std::endl;
		abort();
	}

    //humanoid.gravity = Vector3dZero;
    cout<<"g = "<<humanoid.gravity<<endl;

	cout<<"humanoid.mBodies.size() = "<<humanoid.mBodies.size()<<endl;

	// States: x = [ q q_dot ]
	VectorNd x = VectorNd::Zero(humanoid.dof_count*2);
	// a little push:
    //for(int i=0;i<humanoid.dof_count;i++){   x[i+humanoid.dof_count] =.1;   }
    
    // State derivatives xd = [ qdot qddot ]
    
    // Control vector u = [ tau ]
    VectorNd u = VectorNd::Zero(humanoid.dof_count);
    
	double t = 0.;
	double tf = 3;//usualy 10,
	double h = 0.001;

	std::ofstream of("animation.csv");

	// fileformat: time , q1,q2,q3,...
	while (t <= tf) {
        //add joint torques in u if needed
		of << t << ", ";
        for (unsigned int i = 0; i < humanoid.dof_count; i++){
            of << x[i] << ", ";
        }
        of << "\n";
		//x = x + h * rk4_integrator (x, u, h, rhs);
        x = x + h * rk4_integrator (x, u, h, rhs_contact_force);
        //x = x + h * rk4_integrator (x, u, h, rhs_contact_force_damping);
        //x = x + h * rk4_integrator (x, u, h, rhs_constraint_set);
		t = t + h;
	}
	of.close();

	return 0;
}
