#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

// global variables (can be used in all scopes, e.g. inside functions)
double l0 = 1.0;
double l1 = 1.0;
double l2 = 0.15;

/**
 * @brief Inverse Kinematic
 * This function evaluates the inverse kinematic of the planar gripper
 * analytically and returns the kinematic configuration of the gripper
 * @param TCP vector containing TCP states [x, y, phi]
 * @return 3 dimensional vector containing [q0(x, y, phi), q1(x, y, phi), q2(x, y, phi)]
 */
Vector3d inverseKinematic(Vector3d TCP) {
    Vector3d q;
    q(0) = 0.0;
    q(1) = 0.0;
    q(2) = 0.0;
    return q;
}

/**
 * @brief Forward Kinematic
 * This function computes the end effector position for given control angles q
 * @param q vector containing angle configurations [q0, q1, q2]
 * @return 3 dimensional vector containing TCP states [x(q0,q1,q2), y(q0,q1,q2), phi(q0,q1,q2)]
 */
Vector3d forwardKinematic(Vector3d q) {
    Vector3d zAxis(0.0, 0.0, 1.0);
    Vector3d posJoint3(0.0, 0.0, 0.0);
    posJoint3 = AngleAxisd(q(2), zAxis) * (Vector3d(0.0, l2, 0.0) + posJoint3);
    posJoint3 = AngleAxisd(q(1), zAxis) * (Vector3d(0.0, l1, 0.0) + posJoint3);
    posJoint3 = AngleAxisd(q(0), zAxis) * (Vector3d(0.0, l0, 0.0) + posJoint3);

    return Vector3d(posJoint3.x(), posJoint3.y(), q(0) + q(1) + q(2) + M_PI/2.0);
}

/**
 * @brief Cube Trajectory
 * @param t time (between 0 and 1)
 * @return 3 dimensional vector containing cube trajectory and orientation [x(t), y(t), phi(t)]
 */
Vector3d cubeTrajectory(double t) {

    double r = l0 + l1 + l2;

    double offs_y = r*2.0/3.0;
    double offs_x = 0.0;
    double radian = r/3.0;

    double angle = M_PI*2.0*t;
    double traj_x = offs_x + radian * cos(angle) * sin(angle) * sin(angle);
    double traj_y = offs_y + radian * sin(angle) * sin(angle) * sin(angle);

    double traj_dY = radian * 3.0 * cos(angle) * sin(angle) * sin(angle);
    double traj_dX = radian * (-1.0 * sin(angle) * sin(angle) * sin(angle) + 2.0 * sin(angle) * cos(angle) * cos(angle));

    double traj_phi;
    if (t < 0.5) {
        traj_phi = atan2(traj_dY, traj_dX) - M_PI/2.0;
    } else {
        traj_phi = atan2(traj_dY, traj_dX) + M_PI/2.0;
    }

    return Vector3d(traj_x, traj_y, traj_phi);
}

/**
 * @brief Test if angles (rad) are almost equal
 * @return true if angles are almost equal, else otherwise
 */
bool angleEqual(double angle1, double angle2) {

    angle1 = std::fmod(angle1, 2.0*M_PI);
    if (angle1 < 0) {
        angle1 += 2.0*M_PI;
    }

    angle2 = std::fmod(angle2, 2.0*M_PI);
    if (angle2 < 0) {
        angle2 += 2.0*M_PI;
    }

    if ( abs(angle1 - angle2) > 1e-12) {
        return false;
    } else {
        return true;
    }
}

int main(int argc, char *argv[]) {

    // open output file
    ofstream file;
    file.open("animation.csv", ios::out);
    file.setf(ios::fixed);

    size_t n_samples = 101;
    bool bFailed = false;
    for (size_t i = 0; i <= n_samples; ++i) {
        double t = static_cast<double>(i)/static_cast<double>(n_samples);

        // compute trajectory
        Vector3d TCP = cubeTrajectory(t);

        // compute analytic inverse kinematic
        Vector3d q = inverseKinematic(TCP);

        // test inverseKinematic solution
        Vector3d TCP_ref = forwardKinematic(q);
        if ( ((TCP.x() - TCP_ref.x()) > 1e-12) ||
             ((TCP.y() - TCP_ref.y()) > 1e-12) ||
             !angleEqual(TCP(2), TCP_ref(2)) )
        {
            cerr << "ERROR at t = " << t << ": TCP is (" << TCP(0) << ", " << TCP(1) << ", " << TCP(2) << ")"
                 << " but should be (" << TCP_ref(0) << ", " << TCP_ref(1) << ", " << TCP_ref(2) << ")"
                 << endl;
            bFailed = true;
        }

        // export data
        file << t << ", " << TCP(0) << ", " << TCP(1) << ", " << TCP(2) << ", " << q(0) << ", " << q(1) << ", " << q(2) << "\n";
    }

    if (bFailed) {
        cout << "There were errors! Check your implementation." << endl;
    } else {
        cout << "Your code seems to work properly!" << endl;
    }

    return 0;
}
