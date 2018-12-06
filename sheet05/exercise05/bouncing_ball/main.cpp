#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

using namespace Eigen;
const double g = 9.81;

// Define a few types to make it easier
typedef VectorXd (*rhsFuncPtr) (const double, const VectorXd&);
typedef VectorXd (*integratorFuncPtr) (const double, const VectorXd&, const double, rhsFuncPtr);

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

/* Choose the requested task here */

//#define TEILAUFGABE_1 //Free fall
//#define TEILAUFGABE_2 //Reflection
//#define TEILAUFGABE_3 //Air resistance
//#define TEILAUFGABE_4 //3D
#define TEILAUFGABE_5_UND_6 //Wind

/****************************************
 * Aufgabe 5.1 Teilaufgabe 1: Free Fall *
 ****************************************/
#ifdef TEILAUFGABE_1

const int dim=2;

VectorXd rhs (double t, const VectorXd &y) {
	assert (y.size() == dim);
	VectorXd res = VectorXd::Zero(dim);

	/* 1. Free fall:
	 * Insert equation z'' = -g
	 * Substitution: Z = z0, z' = z1;
	 * --> zo' = z' = z-Geschwindigkeit = z1
	 * --> z1' = z'' = z-Beschleunigung = -g (Beschleunigung richtet sich entgegen das Koordinatensystem)
	 * Strecke entlang X --> 0
	 * Geschwindigkeit entlang X --> 1
	 * Strecke entlang y --> 2
	 * Geschwindigkeit entlang y --> 3
	 * Strecke entlang Z --> 4
	 * Geschwindigkeit entlang Z --> 5
 	 */

	//Result ist unser z' = [z0', z1']-Vektor
	//[z0', z1'] = [z1, -g]
	res[0] = y[1]; 
	res[1] = -g; 

	return res;
}


int main(int argc, char *argv[]) {

	//Initial values
	VectorXd y0  = VectorXd::Zero(dim);
	y0[0] = 10.0; //Initial heigth
	y0[1] = 0.0; //Initial velocity along z/heigth

	double t0 = 0.0;

	double tf = 40.0;//Gesamtzeit
	double h = 0.01; //Zeitliche Schrittweite
		
	double t = t0;
	VectorXd y = y0;
	
	std::ofstream of("animation.csv");
	std::ofstream ff("arrows.ff");

	while (t <= tf) {

		//We want write the results, so:
		Eigen::Vector3d pos(0,0,y[0]);

		of << t << ", " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		
		y = y + h * rk4_integrator (t, y, h, rhs);
		t = t + h;

		ff << t << ", " << 0 << ", " << 0 << ", " << y[0] << ", " << 0 << ", " << 0 << ", " << y[1]*500 << ", 0, 0, 0, 0, 0, 0\n";
	}

	of.close();
	ff.close();
	return 0;
}

#endif

/*****************************************
 * Aufgabe 5.1 Teilaufgabe 2: Reflection *
 *****************************************/
#ifdef TEILAUFGABE_2

const int dim=2;

VectorXd rhs (double t, const VectorXd &y) {
	assert (y.size() == dim);
	VectorXd res = VectorXd::Zero(dim);

	//Result ist unser z' = [z0', z1']-Vektor
	res[0] = y[1];
	res[1] = -g;

	return res;
}


int main(int argc, char *argv[]) {

	//Initial values
	VectorXd y0  = VectorXd::Zero(dim);
	y0[0] = 10.0; //Initial heigth
	y0[1] = 0.0; //Initial velocity along z/heigth

	double t0 = 0.0;

	double tf = 40.0;//Gesamtzeit
	double h = 0.01; //Zeitliche Schrittweite
		
	double t = t0;
	VectorXd y = y0;
	
	std::ofstream of("animation.csv");
	std::ofstream ff("arrows.ff");

	while (t <= tf) {

		//We want write the results, so:
		Eigen::Vector3d pos(0,0,y[0]);

		of << t << ", " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		

		y = y + h * rk4_integrator (t, y, h, rhs);
		t = t + h;

		/* 2. Reflection
		 * Elastic collision at z=0;
		 * As soon as z = y[0] <= 0 we changce the velocity direction, by changing y[1]:
		 */
		if(y[0] <= 0.0){ //If z reaches zero
			y[0] = 0.0; //It should not be possible to be below ground. Numerical it can be that way of course.
			y[1] = -y[1]; 
		}
		/* "Why is the energy conserved in this case of flipping the sign?"
		 * Physically, there is no negative velocity. The sign only indicates the direction of the velocity. Since we are not changing
		 * its magnitude, the value stays the same.
		 *
		 * "How can we see in the simulation that the energy is conserved"?
		 * We can see that in the fact that the ball reaches the same heigth over and over again. If energy got lost,
    		 * It would start to bounce less high.
		 */

		ff << t << ", " << 0 << ", " << 0 << ", " << y[0] << ", " << 0 << ", " << 0 << ", " << y[1]*500 << ", 0, 0, 0, 0, 0, 0\n";
	}

	of.close();
	ff.close();
	return 0;
}

#endif


/*********************************************
 * Aufgabe 5.1 Teilaufgabe 3: Air resistance *
 *********************************************/
#ifdef TEILAUFGABE_3

const int dim=2;

VectorXd rhs (double t, const VectorXd &y) {
	assert (y.size() == dim);
	VectorXd res = VectorXd::Zero(dim);
	
	/* 3. Air resistance:
	 * Insert equation z'' += -k * z' * |z'| //Wir behalten das Vorzeichen der Geschwindigkeit bei
	 * Substitution: Z = z0, z' = z1;
	 * --> zo' = z1
	 * --> z1' = -g - k*z1*|z1| (Der Luftwiderstand wirkt gegen die Geschwindigkeitsrichtung (Siehe Vorzeichen v))
	 * We are supposed to guess k...
	 */
	double k = 0.05;//It doesnt look very realistic at all...it keeps bouncing and bouncing, maybe we need to remove energy if it hits the ground (see main loop)
	//Since abs() somehow does not work, lets get the sign ouerselves.
	short sign;	
	if(y[1] < 0){
		sign = -1;
	}else{
		sign = 1;
	}

	res[0] = y[1];
	res[1] = -g - k*y[1]*y[1]*sign;

	return res;
}


int main(int argc, char *argv[]) {

	//Initial values
	VectorXd y0  = VectorXd::Zero(dim);
	y0[0] = 10.0; 
	y0[1] = 0.0;

	double t0 = 0.0;

	double tf = 40.0;
	double h = 0.01; 
		
	double t = t0;
	VectorXd y = y0;

	//For collisions
	double part_of_energy_after_collision = 0.9;
	
	std::ofstream of("animation.csv");
	std::ofstream ff("arrows.ff");

	while (t <= tf) {

		//We want write the results, so:
		Eigen::Vector3d pos(0,0,y[0]);

		of << t << ", " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		

		y = y + h * rk4_integrator (t, y, h, rhs);
		t = t + h;

		//Restriction along z
		if(y[0] <= 0.0){ //I made the hit only partly elastic since it should realistic in 3. and full elastic collisions are strange
			y[0] = 0.0; 
			y[1] = -part_of_energy_after_collision*y[1]; 
		}

		ff << t << ", " << 0 << ", " << 0 << ", " << y[0] << ", " << 0 << ", " << 0 << ", " << y[1]*500 << ", 0, 0, 0, 0, 0, 0\n";
	}

	of.close();
	ff.close();
	return 0;
}


#endif

/*********************************
 * Aufgabe 5.1 Teilaufgabe 4: 3D *
 *********************************/
#ifdef TEILAUFGABE_4

//The dimensions have changed since we are moving now in 3 spaces.
const int dim=6;

VectorXd rhs (double t, const VectorXd &y) {
	/* res as well as y have now 6 dimensions, which are:
	 * index 0: Position in X
	 * index 1: Velocity in X
	 * index 2: Position in Y
	 * index 3: Velocity in Y
	 * index 4: Position in Z
	 * index 5: Velocity in Z
	 */
	assert (y.size() == dim);
	VectorXd res = VectorXd::Zero(dim);

	double k = 0.05;
	short sign;	

	//Z:
	if(y[5] < 0){
		sign = -1;
	}else{
		sign = 1;
	}

	res[4] = y[5];
	res[5] = -g - k*y[1]*y[1]*sign;

	//Air dynamics are also counting for movement in x and y

	//X:
	if(y[1] < 0){
		sign = -1;
	}else{
		sign = 1;
	}
	res[0] = y[1];
	res[1] = - k*y[1]*y[1]*sign;

	//Y:
	if(y[3] < 0){
		sign = -1;
	}else{
		sign = 1;
	}
	res[2] = y[3];
	res[3] = - k*y[3]*y[3]*sign;

	return res;
}


int main(int argc, char *argv[]) {

	//Initial values
	VectorXd y0  = VectorXd::Zero(dim);

	//Change initial values here:
	y0[0] = 0.0; //Initial Position in X
	y0[1] = -30.0; //Initial Velocity in X
	y0[2] = 0.0; //Initial Position in Y
	y0[3] = -10.0; //Initial Velocity in Y
	y0[4] = 10.0; //Initial heigth
	y0[5] = 0.0; //Initial velocity along z/heigth

	double t0 = 0.0;

	double tf = 40.0;
	double h = 0.01; 
		
	double t = t0;
	VectorXd y = y0;

	//For collisions
	double part_of_energy_after_collision = 0.9;
	
	std::ofstream of("animation.csv");
	std::ofstream ff("arrows.ff");

	while (t <= tf) {

		//We want write the results, so:
		Eigen::Vector3d pos(y[0],y[2],y[4]);

		of << t << ", " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		

		y = y + h * rk4_integrator (t, y, h, rhs);
		t = t + h;

		/* 4. 3D
		 * The ball is supposed to stay within -15 <= x, y <= 15
		 * We are supposed to have initial velocities for x and y with x, y != 0
		 */
		//Restriction along x 
		if(y[0] <= -15.0){ //Change velocity if it hit
			y[0] = -15.0; 
			y[1] = -part_of_energy_after_collision*y[1]; 
		}else if(y[0] >= 15.0){
			y[0] = 15.0;
			y[1] = -part_of_energy_after_collision*y[1];
		}
		
		//Restriction along y 
		if(y[2] <= -15.0){ //Change velocity if it hit
			y[2] = -15.0; 
			y[3] = -part_of_energy_after_collision*y[3]; 
		}else if(y[2] >= 15.0){
			y[2] = 15.0;
			y[3] = -part_of_energy_after_collision*y[3];
		}

		//Restriction along z
		if(y[4] <= 0.0){ 
			y[4] = 0.0; 
			y[5] = -part_of_energy_after_collision*y[5]; 
		}

		ff << t << ", " << y[0] << ", " << y[2] << ", " << y[4] << ", " << y[1]*500 << ", " << y[3]*500 << ", " << y[5]*500 << ", 0, 0, 0, 0, 0, 0\n";
	}

	of.close();
	ff.close();
	return 0;
}


#endif

/****************************************************
 * Aufgabe 5.1 Teilaufgabe 5 und 6: Wind and Arrows *
 ****************************************************/
#ifdef TEILAUFGABE_5_UND_6

const int dim=6;

VectorXd rhs (double t, const VectorXd &y) {
	assert (y.size() == dim);
	VectorXd res = VectorXd::Zero(dim);

	double k = 0.05;
	short sign;	
	if(y[5] < 0){
		sign = -1;
	}else{
		sign = 1;
	}

	res[4] = y[5];
	res[5] = -g - k*y[5]*( sign*y[5] );

	/* 5. Wind
     	 * "Adding wind is simple, as v_rel is the velocity between air and the object. Add wind to the simulation."
	 * Wind is added as an additional term 
 	 */
	double wind_speed_x = 15.0;
	double wind_speed_y = 7.5;

	//X:
	if(y[1] < 0){
		sign = -1;
	}else{
		sign = 1;
	}
	res[0] = y[1];
	res[1] = - k*y[1]*y[1]*sign + k*(wind_speed_x*wind_speed_x); //Air resistance and wind

	//Y:
	if(y[3] < 0){
		sign = -1;
	}else{
		sign = 1;
	}
	res[2] = y[3];
	res[3] = - k*y[3]*y[3]*sign + k*(wind_speed_y*wind_speed_y);

	return res;
}


int main(int argc, char *argv[]) {

	//Initial values
	VectorXd y0  = VectorXd::Zero(dim);
	y0[0] = 0.0; //Initial Position in X
	y0[1] = -10.0; //Initial Velocity in X
	y0[2] = 0.0; //Initial Position in Y
	y0[3] = -5.0; //Initial Velocity in Y
	y0[4] = 10.0; //Initial heigth
	y0[5] = 0.0; //Initial velocity along z/heigth

	double t0 = 0.0;

	double tf = 40.0;
	double h = 0.01; 
		
	double t = t0;
	VectorXd y = y0;

	//For collisions
	double part_of_energy_after_collision = 0.9;
	
	std::ofstream of("animation.csv");
	std::ofstream ff("arrows.ff");

	while (t <= tf) {

		//We want write the results, so:
		Eigen::Vector3d pos(y[0],y[2],y[4]);

		of << t << ", " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		

		y = y + h * rk4_integrator (t, y, h, rhs);
		t = t + h;

		//Restriction along x 
		if(y[0] <= -15.0){ 
			y[0] = -15.0; 
			y[1] = -part_of_energy_after_collision*y[1]; 
		}else if(y[0] >= 15.0){
			y[0] = 15.0;
			y[1] = -part_of_energy_after_collision*y[1];
		}
		
		//Restriction along y 
		if(y[2] <= -15.0){ 
			y[2] = -15.0; 
			y[3] = -part_of_energy_after_collision*y[3]; 
		}else if(y[2] >= 15.0){
			y[2] = 15.0;
			y[3] = -part_of_energy_after_collision*y[3];
		}

		//Restriction along z
		if(y[4] <= 0.0){ 
			y[4] = 0.0; 
			y[5] = -part_of_energy_after_collision*y[5]; 
		}

		/* 6. Arrows
		 * We should write af file with the following values_
		 * -) The first three values mark the linear position in x,y,z
		 * -) The second three values mark the "magnitude of length" of the vector
		 * -) The next three value mark the position of the rotational vector
		 * -) The last three values mark the magnitude of ration
		 */
		ff << t << ", " << y[0] << ", " << y[2] << ", " << y[4] << ", " << y[1]*500 << ", " << y[3]*500 << ", " << y[5]*500 << ", 0, 0, 0, 0, 0, 0\n";
	}

	of.close();
	ff.close();
	return 0;
}

#endif
