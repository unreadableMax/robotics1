#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// Example for the use of eigen3 lib

int main(int argc, char *argv[]) {
	
	// Define and print a vector
	
	Vector3d a(1,2,0);
	std::cout <<"a= \n" <<a<< std::endl;
	std::cout << std::endl;
	
	
	// Define and print a matrix
	
	Matrix3d R;
	R << 	0,-1,0,
		1,0,0,
		0,0,1;

	Vector3d b = R*a;

	cout << "b=R*a=\n"<<b<<endl;

	cout <<"a*b=\n"<<a.dot(b)<<endl;
	

	Vector3d v;

	v = a.cross(b);
	v=v.normalized();

	cout<<"normalized cross product of a and b = \n" <<v<<endl;
	
	

	
	
	return 0;
}
