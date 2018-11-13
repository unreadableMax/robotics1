#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

// Example for the use of eigen3 lib

int main(int argc, char *argv[]) {
	
	// Define and print a vector
	
	Vector3d v(1,2,3);
	std::cout << v << std::endl;
	std::cout << std::endl;
	
	
	// Define and print a matrix
	
	Matrix3d M;
	M << 1, 0, 0,
	     2, 5, 3,
	     0, 2 ,8;
	
	std::cout << M << std::endl;
	std::cout << std::endl;
	
	// Print the product
	std::cout << M * v<< std::endl;
	
	
	return 0;
}
