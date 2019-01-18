#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <ctime>
#include "nabo/nabo.h"
#include "nlopt.h"
#include "stdlib.h"
#include "utilities.hpp"
#include "file_rw.hpp"
#include "transformation_utilities.hpp"
#include "opt_obj.hpp"

// template <typename t>
// 	std::vector<std::vector<t> > GetUniqueRows(std::vector<std::vector<t> > input)
// 	{
//     	// std::sort(input.begin(), input.end());
//     	input.erase(std::unique(input.begin(), input.end()), input.end());
//     	return input;	
// 	}

int main()
{
	Eigen::Vector3d tool_t;
    Eigen::MatrixXd tool_r(1,3);
    Eigen::Matrix4d tool_F_T_tcp;

    tool_t << -0.2,-0.01,48.6;
    tool_r << 0,0,0;
    tool_F_T_tcp = ut::hom_T(tool_t,rtf::eul2rot(tool_r,"ZYX"));

	// get appropriate scan data for ICP
	std::string traj_from_kuka_scanning_file = "/home/rflin/Desktop/ANIRUDDHA_WS/CPP/data_files/data_for_ICP.csv";
	
    // get appropriate scan data for ICP
    std::vector<std::vector<double> > traj_from_kuka_scanning_vec;
    
    traj_from_kuka_scanning_vec = file_rw::file_read(traj_from_kuka_scanning_file);
	// std::vector<std::vector<double> > traj_from_kuka_scanning_vec;
    
 //    traj_from_kuka_scanning_vec = file_rw::file_read(traj_from_kuka_scanning_file);
	// traj_from_kuka_scanning_vec = GetUniqueRows(file_rw::file_read(traj_from_kuka_scanning_file));
	
	// Eigen::MatrixXd pts_from_tcp_publisher(traj_from_kuka_scanning_vec.size(),traj_from_kuka_scanning_vec[0].size());
	// pts_from_tcp_publisher = ut::vec_to_mat(traj_from_kuka_scanning_vec);
	// Eigen::Vector3d b_t_Flange;
	// Eigen::MatrixXd b_eul_Flange(1,3);
	// Eigen::Matrix3d b_r_Flange;
	// Eigen::Matrix4d b_T_Flange;
	// Eigen::Matrix4d b_T_tcp;
	// // Eigen::Matrix3d new_r;
		
	// // angles must be radians
	// Eigen::MatrixXd transformed_pt = Eigen::MatrixXd::Constant(pts_from_tcp_publisher.rows(),pts_from_tcp_publisher.cols(),0);
	// for (int i=0;i<pts_from_tcp_publisher.rows();++i)
	// {
	// 	b_t_Flange << pts_from_tcp_publisher(i,0), pts_from_tcp_publisher(i,1), pts_from_tcp_publisher(i,2);
	// 	b_eul_Flange << pts_from_tcp_publisher(i,3), pts_from_tcp_publisher(i,4), pts_from_tcp_publisher(i,5);
	// 	b_r_Flange = rtf::eul2rot(b_eul_Flange);
	// 	b_T_Flange = ut::hom_T(b_t_Flange,b_r_Flange);
	// 	b_T_tcp = b_T_Flange*tool_F_T_tcp;
	// 	// new_r = b_T_tcp.block(0,0,3,3);
	// 	transformed_pt(i,0) = b_T_tcp(0,3);
	// 	transformed_pt(i,1) = b_T_tcp(1,3);
	// 	transformed_pt(i,2) = b_T_tcp(2,3);
	// }

	// Eigen::MatrixXd scan_traj_wrt_tcp(transformed_pt.rows(),transformed_pt.cols());

	// // saves only xyz and not euler angles
	Eigen::MatrixXd scan_traj_wrt_tcp(traj_from_kuka_scanning_vec.size(),traj_from_kuka_scanning_vec[0].size());
	scan_traj_wrt_tcp = ut::get_traj_wrt_tcp(tool_F_T_tcp, traj_from_kuka_scanning_vec);

	std::cout << scan_traj_wrt_tcp << std::endl;    
}

