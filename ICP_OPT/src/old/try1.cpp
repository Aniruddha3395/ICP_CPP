#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include "stdlib.h"
#include "utilities.hpp"
#include "file_rw.hpp"
#include "robotics_utilities.hpp"

// Eigen::Matrix3d eul2rot(Eigen::MatrixXd);
// Eigen::MatrixXd rot2eul(Eigen::Matrix3d);
// Eigen::Matrix3d qt2rot(Eigen::MatrixXd);
// Eigen::MatrixXd rot2qt(Eigen::Matrix3d);
// Eigen::MatrixXd eul2qt(Eigen::MatrixXd);
// Eigen::MatrixXd qt2eul(Eigen::MatrixXd);


int main()
{
	
	Eigen::MatrixXd eul(1,3);
	eul << 0.5,0,-0.5;
	Eigen::Matrix3d M = rtb::eul2rot(eul);

	std::cout << rtb::eul2qt(eul) << std::endl << std::endl;

	std::cout << M << std::endl;

	std::cout << rtb::rot2eul(M) << std::endl << std::endl;

	Eigen::MatrixXd q(1,4);
	q << 0.5,0.5,0.75,0.75;
	
	Eigen::MatrixXd N(1,3);
	
	N = rtb::qt2eul(q);

	std::cout << N << std::endl << std::endl;

	// std::cout << rot2qt(M) << std::endl << std::endl;


	return 0;
}

// Eigen::Matrix3d eul2rot(Eigen::MatrixXd eul_angles)
// {
// // Euler Angles to Rotation Matrix Conversion 
// // NOTE: Euler Angles are: (alpha,beta,gamma) in ZYX-form 

// 	Eigen::Matrix3d rot_mat;

// 	rot_mat = Eigen::AngleAxisd(eul_angles(0,0), Eigen::Vector3d::UnitZ())
//        *Eigen::AngleAxisd(eul_angles(0,1), Eigen::Vector3d::UnitY())
//        *Eigen::AngleAxisd(eul_angles(0,2), Eigen::Vector3d::UnitX());

//     return rot_mat; 
// }

// Eigen::MatrixXd rot2eul(Eigen::Matrix3d rot_mat)
// {
// // Rotation Matrix to Euler Angles Conversion 
// // NOTE: Euler Angles are: (alpha,beta,gamma) in ZYX-form 

// 	Eigen::MatrixXd eul_angles(1,3);

// 	Eigen::Vector3d eul_angles_vec = rot_mat.eulerAngles(2, 1, 0);
// 	eul_angles(0,0) = eul_angles_vec[0];
// 	eul_angles(0,1) = eul_angles_vec[1];
// 	eul_angles(0,2) = eul_angles_vec[2];

// 	return eul_angles;
// }

// Eigen::Matrix3d qt2rot(Eigen::MatrixXd quat)
// {
// // Quaternion to Rotation Matrix Conversion
// // NOTE: Input Quaternion is in [w,x,y,z] form  

// 	Eigen::Quaterniond q;

// 	q.w() = quat(0,0);
// 	q.x() = quat(0,1);
// 	q.y() = quat(0,2);
// 	q.z() = quat(0,3);
	
// 	return q.normalized().toRotationMatrix();
// }

// Eigen::MatrixXd rot2qt(Eigen::Matrix3d rot_mat)
// {
// // Rotation Matrix to Quaternion Conversion
// // NOTE: Input Quaternion is in [w,x,y,z] form 
// 	Eigen::MatrixXd quat(1,4);
// 	Eigen::Quaterniond q(rot_mat);

// 	quat(0,0) = q.w();
// 	quat(0,1) = q.x();
// 	quat(0,2) = q.y();
// 	quat(0,3) = q.z();

// 	return quat;
// }

// Eigen::MatrixXd eul2qt(Eigen::MatrixXd eul_angles)
// {
// // Euler Angles to Quaternion Matrix Conversion 
// // NOTE: Euler Angles are: (alpha,beta,gamma) in ZYX-form 
// // NOTE: Input Quaternion is in [w,x,y,z] form 

// 	Eigen::MatrixXd quat(1,4);
// 	Eigen::Quaterniond q;

// 	q = Eigen::AngleAxisd(eul_angles(0,0), Eigen::Vector3d::UnitZ())
//        *Eigen::AngleAxisd(eul_angles(0,1), Eigen::Vector3d::UnitY())
//        *Eigen::AngleAxisd(eul_angles(0,2), Eigen::Vector3d::UnitX());
// 	quat(0,0) = q.w();
// 	quat(0,1) = q.x();
// 	quat(0,2) = q.y();
// 	quat(0,3) = q.z();

//     return quat; 
// }

// Eigen::MatrixXd qt2eul(Eigen::MatrixXd quat)
// {
// // Quaternion to Euler Angles Matrix Conversion 
// // NOTE: Euler Angles are: (alpha,beta,gamma) in ZYX-form 
// // NOTE: Input Quaternion is in [w,x,y,z] form 

// 	Eigen::Matrix3d rot_mat;
// 	Eigen::Quaterniond q;
// 	Eigen::MatrixXd eul_angles(1,3);

// 	q.w() = quat(0,0);
// 	q.x() = quat(0,1);
// 	q.y() = quat(0,2);
// 	q.z() = quat(0,3);
	
// 	rot_mat = q.normalized().toRotationMatrix();

// 	Eigen::Vector3d eul_angles_vec = rot_mat.eulerAngles(2, 1, 0);
// 	eul_angles(0,0) = eul_angles_vec[0];
// 	eul_angles(0,1) = eul_angles_vec[1];
// 	eul_angles(0,2) = eul_angles_vec[2];

// 	return eul_angles;
// }
