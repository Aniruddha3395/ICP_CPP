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

int main()
{

////////////////////////////GLOBAL CONFIG////////////////////////////////////
    std::string Data_dir = "/home/aniruddha/Desktop/ICP/ICP_OPT/data/";    
    // Tool Data
    Eigen::Vector3d tool_t;
    Eigen::MatrixXd tool_r(1,3);
    Eigen::Matrix4d tool_F_T_tcp;
    // tool_t << -0.2,-0.01,48.6;
    // tool_r << 0,0,0;
    tool_t << -0.0002,-0.00001,0.0486;
    tool_r << 0,0,0;
    
    tool_F_T_tcp = rtf::hom_T(tool_t,rtf::eul2rot(tool_r,"ZYX"));

    // Input Transformation Matrix for ICP
    Eigen::Vector3d input_w_t_p;
    Eigen::MatrixXd input_w_r_p(1,3);
    Eigen::Matrix4d input_w_T_p;
    // input_w_t_p << 478.4168,-160.7013,290.4602;
    // input_w_r_p << 0.0873,0.2618,0.0873;
    input_w_t_p << 0.4784,-0.1607,0.2904;
    input_w_r_p << 0.0873,0.2618,0.0873;
    input_w_T_p = rtf::hom_T(input_w_t_p,rtf::eul2rot(input_w_r_p,"ZYX"));

    // Part pointcloud from STL file 
    std::string part_ptcloud_file = Data_dir+"part_ptcloud.csv";
    std::vector<std::vector<double> > part_ptcloud_vec;
    part_ptcloud_vec = file_rw::file_read_vec(part_ptcloud_file);
    Eigen::MatrixXd part_ptcloud(part_ptcloud_vec.size(),part_ptcloud_vec[0].size());
    part_ptcloud = ut::vec_to_mat(part_ptcloud_vec);

    // Part poinrcoud transformed using Input Transformation Matrix for ICP
    Eigen::MatrixXd input_part_ptcloud_icp(part_ptcloud.rows(),part_ptcloud.cols());
    input_part_ptcloud_icp = rtf::apply_transformation(part_ptcloud, input_w_T_p);

    
    // Data for ICP from KUKA Scanning
    std::string traj_from_kuka_scanning_file = Data_dir+"data_for_ICP.csv";
    std::vector<std::vector<double> > traj_from_kuka_scanning_vec;
    traj_from_kuka_scanning_vec = file_rw::file_read_vec(traj_from_kuka_scanning_file);
    Eigen::MatrixXd scan_traj_wrt_tcp(traj_from_kuka_scanning_vec.size(),traj_from_kuka_scanning_vec[0].size());
    scan_traj_wrt_tcp = ut::get_traj_wrt_tcp(tool_F_T_tcp, traj_from_kuka_scanning_vec);
 
    // std::vector<std::vector<double> > part_ptcloud_normals_vec;
    // std::string part_ptcloud_normals_file = "/home/rflin/Desktop/ANIRUDDHA_WS/CPP/data_files/part_ptcloud_normals.csv";
    // std::string scanned_traj_file = "/home/rflin/Desktop/ANIRUDDHA_WS/CPP/data_files/scanned_traj.csv";
    // std::vector<std::vector<double> > scanned_traj_vec;
    // part_ptcloud_normals_vec = file_rw::file_read(part_ptcloud_normals_file);
    // scanned_traj_vec = file_rw::file_read(scanned_traj_file);
    // Eigen::MatrixXd part_ptcloud_normals(part_ptcloud_normals_vec.size(),part_ptcloud_normals_vec[0].size());
    // Eigen::MatrixXd scanned_traj(scanned_traj_vec.size(),scanned_traj_vec[0].size());
    // part_ptcloud_normals = ut::vec_to_mat(part_ptcloud_normals_vec);
    // scanned_traj = ut::vec_to_mat(scanned_traj_vec);
    // Eigen::MatrixXd input_part_ptcloud_icp_true(input_part_ptcloud_icp.rows(),input_part_ptcloud_icp.cols());
    // input_part_ptcloud_icp_true = input_part_ptcloud_icp;

    // Optimization Params 
    const int K = 5;            // K is number of nearest neighbours to be calculated for the lsf plane
    double Error_threshold = 0.003;
    double perturb_val = 0.005;
    std::vector<double> x0 = {0,0,0,0,0,0,1};       //Seed for ICP
    double W1 = 1.0;       // Weight for Avg(dist)
    double W2 = 0.0;       // Weight for Max(dist)
    double optH = 1e-13;
    double OptXtolRel = 1e-6;

    // Optimization Routine
    Eigen::Matrix4d icp_T;
    Eigen::Vector3d icp_t;
    Eigen::MatrixXd icp_qt(1,4);
    Eigen::MatrixXd icp_eul(1,3);
    Eigen::Matrix4d icp_T_final = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d icp_T_final_save;
    Eigen::Matrix4d Final_w_T_p;
    Eigen::MatrixXd input_part_ptcloud_icp_save(input_part_ptcloud_icp.rows(),input_part_ptcloud_icp.cols());
    opt_obj::opt_obj OptObj(x0, K, input_part_ptcloud_icp, scan_traj_wrt_tcp, 
        Error_threshold, perturb_val, W1, W2, optH, OptXtolRel);
    icp_T_final_save = OptObj.solveOPTCust();        

    // Final Transformation Matrix
    Final_w_T_p = icp_T_final_save*input_w_T_p;
    std::cout << Final_w_T_p << std::endl;
    return 0;
}

