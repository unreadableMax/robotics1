#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

using namespace Eigen;



// Define a few types to make it easier
typedef Matrix<double, 6, 1>  Vector6d;

struct datarow_t {
	double t;
	Vector6d q;
	Eigen::Vector3d tcp;
};

typedef std::vector<datarow_t> Data ;

Data readFile(std::string filename) {
	Data data;
	
	std::ifstream infile(filename.c_str());
	if (!infile.good()) {
		std::cerr << "Could not open '" << filename << "'" << std::endl;
		
	}
	std::string line;
	//Ließt die Information zeilenweise ein und schmeißt sie als Zeile auf den Data-Vektor
	while (std::getline(infile, line))
	{
		for(size_t i=0; i < line.size(); i++) 
			if (line[i]==',') line[i]=' ';
		std::istringstream iss(line);
		datarow_t dr;
		if ((iss >> dr.t >> dr.q[0]>> dr.q[1]>> dr.q[2]>> dr.q[3]>> dr.q[4]>> dr.q[5] )) { 
			data.push_back(dr);
		} else {
			std::cerr << "could not parse the following line:\n" << line << std::endl;
		}
	}
	infile.close();
	return data;
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
		for(int j=0; j < 3; j++) {
			outfile << ", " << data[i].tcp[j];
		}
		outfile << "\n";
	}
	
	outfile.close();
}


Vector3d forwardKinematic(Vector6d q) {

	Vector3d t0(0.0,0.0,0.203);
	Vector3d t1(0.075,0.0,0.132);
	Vector3d t2(0.365,0.0,0.0);
	Vector3d t3(0.208,0.0,0.09);	
	Vector3d t4(0.197,0.0,0.0);
	Vector3d t5(0.08,0.0,0.0);
	Vector3d t6(0.13,0.0,0.0);

	//Rotationsachsen
	Vector3d usedXAxis(1.0,0.0,0.0);
	Vector3d usedYAxis(0.0,-1.0,0.0); //-1 -> lfh
	Vector3d usedZAxis(0.0,0.0,1.0);
		
	//calculating the TCP with some rot-matrices
	Vector3d TCP = t6;
	TCP = t5 + AngleAxisd(-q[5], usedXAxis) * TCP;
	TCP = t4 + AngleAxisd(-q[4], usedYAxis) * TCP;
	TCP = t3 + AngleAxisd(-q[3], usedXAxis) * TCP;
	TCP = t2 + AngleAxisd(-q[2], usedYAxis) * TCP;
	TCP = t1 + AngleAxisd(-q[1], usedYAxis) * TCP;
	TCP = t0 + AngleAxisd(-q[0], usedZAxis) * TCP;
	return TCP;

	//return Vector3d(0.600824, -0.600346, 0.313005);
}



int main(int argc, char *argv[]) {
	//Enthält die zeitabhängigen Verläufe der Winkel q0 bis q5 vom Roboterarm
	Data data = readFile("animation.txt");
	//Hier wird für den zeitlichen Verlauf des Roboterarms die Endeffektorposition berechnet
	for (size_t i =0 ; i < data.size(); i++) {
		data[i].tcp = forwardKinematic(data[i].q);
	}
	//Die ergänzten Informationen werden in eine neue Datei "mit TCP" geschrieben
	writeFile("animation_with_tcp.txt", data);
	
	return 0;
}
