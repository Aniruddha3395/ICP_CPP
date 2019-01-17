#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include "stdlib.h"
#include "utilities.hpp"
#include "file_rw.hpp"

// Eigen::MatrixXd mean(Eigen::MatrixXd);
// Eigen::MatrixXd get_rob_T_part(Eigen::MatrixXd, Eigen::MatrixXd);

int main()
{
	Eigen::MatrixXd a(4,3);
	a << 0, 0.82, 5.92,
    	0, 249.68, 47.52,
    	752.48, 249.68, 47.52,
    	752.48, 0.82, 5.92;
	Eigen::MatrixXd b(4,3);
	b << 385.62, 372.98, 11.33,
    	631.25, 375.52, 57.89,
    	633.94, -378.44, 53.31,
    	384.81, -376.38, 7.69;
	std::cout << ut::get_rob_T_part(a,b) << std::endl;
	return 0;
}

// Eigen::MatrixXd get_rob_T_part(Eigen::MatrixXd part_pts, Eigen::MatrixXd rob_pts)
// {
// // part_pts: co-ordinates with respect to CAD part frame
// // rob_pts: points with respect to robot base frame (or world frame if it matches with robot base frame)
// // input: part_pts = [x1, y1, z1;
// //                             x2, y2, z2;
// //                                :
// //                                :
// //                             xn, yn, zn]
// // input: rob_pts = [x1, y1, z1;
// //                            x2, y2, z2;
// //                                :
// //                                :
// //                            xn, yn, zn]
// 	Eigen::MatrixXd centroid_part_pts(1,part_pts.cols());
// 	Eigen::MatrixXd centroid_rob_pts(1,rob_pts.cols());
// 	Eigen::MatrixXd shifted_part_pts(part_pts.rows(),part_pts.cols());
// 	Eigen::MatrixXd shifted_rob_pts(rob_pts.rows(),rob_pts.cols());
// 	Eigen::MatrixXd cros_cov_mat(part_pts.cols(),rob_pts.cols());
// 	Eigen::Matrix3d R;
// 	Eigen::Matrix3d U_T;
// 	Eigen::Matrix3d V;
// 	Eigen::Vector3d T;
// 	Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
// 	Eigen::Matrix4d transform_mat = Eigen::Matrix4d::Constant(0);
// 	if (part_pts.rows()==rob_pts.rows())
// 	{ 
// 		centroid_part_pts = mean(part_pts);
// 		centroid_rob_pts = mean(rob_pts);
// 		for (int i=0;i<part_pts.rows();++i)
// 		{
// 			shifted_part_pts.block(i,0,1,shifted_part_pts.cols()) = part_pts.block(i,0,1,part_pts.cols()) - centroid_part_pts;
// 			shifted_rob_pts.block(i,0,1,shifted_rob_pts.cols()) = rob_pts.block(i,0,1,rob_pts.cols()) - centroid_rob_pts;	
// 		}
// 		cros_cov_mat = shifted_part_pts.transpose()*shifted_rob_pts;
// 	// Singular Value Decomposition
// 		Eigen::JacobiSVD<Eigen::MatrixXd> svd(cros_cov_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
// 	// Take care of reflection case due to negative eigen vectors
// 		U_T = svd.matrixU().transpose();	V = svd.matrixV();
// 		M(2,2) = (V*U_T).determinant();
// 		R = V*M*U_T;
// 		if (R.determinant()>0)
// 		{
// 			T = -R*centroid_part_pts.transpose() + centroid_rob_pts.transpose();
// 			transform_mat.block(0,0,3,3) = R;
// 			transform_mat.block(0,3,3,1) = T;
// 			transform_mat(3,3) = 1; 
// 		}
// 		else
// 		{
// 			std::cout << "Determinant of rotation matrix is negative..." << std::endl;
// 		}	
// 	}
// 	else
// 	{
// 		std::cout << "FUNCTION ERROR: For correspondance, number of rows of both matrices should be same..." << std::endl;
// 	}
// 	return transform_mat;
// }

// Eigen::MatrixXd mean(Eigen::MatrixXd mat)
// {
// 	Eigen::VectorXd vec(mat.cols());
// 	for (int i=0;i<mat.cols();++i)
// 	{
// 		vec(i) =  mat.block(0,i,mat.rows(),1).sum()/mat.rows();
// 	}
// 	return vec.transpose();
// }