#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "file_rw.hpp"
#include "transformation_utilities.hpp"
#include "utilities.hpp"

template <typename t>
    std::vector<std::vector<t> > GetUniqueRows(std::vector<std::vector<t> > input)
    {
        // std::sort(input.begin(), input.end());
        input.erase(std::unique(input.begin(), input.end()), input.end());
        return input;   
    }

///////////////////////////////////////////////////////////

Eigen::MatrixXd ut::vec_to_mat(std::vector<std::vector<double> > vec)
{
    Eigen::MatrixXd mat(vec.size(), vec[0].size());
    for (int i = 0; i < vec.size(); ++i)
        mat.row(i) = Eigen::VectorXd::Map(&vec[i][0], vec[0].size());
    return mat;
}

////////////////////////////////////////////////////////////

void ut::compute_TCP(Eigen::MatrixXd data_points, Eigen::MatrixXd normals)
{
    // tcp computation...bx,by,bz
    Eigen::Vector3d tool_x;
    Eigen::Vector3d tool_y;
    Eigen::Vector3d tool_z;
    Eigen::Vector3d dir_vec;
        
    Eigen::MatrixXd bx(data_points.rows(),3);
    Eigen::MatrixXd by(data_points.rows(),3);
    Eigen::MatrixXd bz(data_points.rows(),3);

    for (int i=0; i<data_points.rows(); ++i)
    {
        if (i!=data_points.rows()-1)
        {
            Eigen::Vector3d direction;
            
        // calculating direction vector from sequence of points
            // direction = data_points.block(i+1,0,1,data_points.cols()) - data_points.block(i,0,1,data_points.cols()); 
        // OR
        // applying constant direction vector
            direction << 0, 1, 0;
            
            dir_vec = direction/direction.norm();    
        }
        tool_z = -normals.block(i,0,1,normals.cols());
        tool_x = dir_vec.cross(tool_z);
        tool_x = tool_x/tool_x.norm();
        tool_y = tool_z.cross(tool_x);
        tool_y = tool_y/tool_y.norm();

        bx.block(i,0,1,3) = tool_x;
        by.block(i,0,1,3) = tool_y;
        bz.block(i,0,1,3) = tool_z;
    }
}

////////////////////////////////////////////////////////////

double ut::get_pt_to_lsf_plane_dist(Eigen::MatrixXd pt, Eigen::MatrixXd pts_for_plane)
{
    Eigen::MatrixXd x_vec(pts_for_plane.rows(),1);
    Eigen::MatrixXd y_vec(pts_for_plane.rows(),1);
    Eigen::MatrixXd z_vec(pts_for_plane.rows(),1);
    double x_avg;   double y_avg;   double z_avg;
    double L00; double L11; double L01; double R0;  double R1;
    double A;   double B;   double C;   double D;
    
    x_vec = pts_for_plane.block(0,0,pts_for_plane.rows(),1);
    y_vec = pts_for_plane.block(0,1,pts_for_plane.rows(),1);
    z_vec = pts_for_plane.block(0,2,pts_for_plane.rows(),1);
    
    x_avg = x_vec.sum()/pts_for_plane.rows();
    y_avg = y_vec.sum()/pts_for_plane.rows();
    z_avg = z_vec.sum()/pts_for_plane.rows();

    L00 = ((x_vec.array() - x_avg).array().pow(2)).sum();
    L01 = ((x_vec.array() - x_avg).array()*(y_vec.array() - y_avg).array()).sum(); 
    L11 = ((y_vec.array() - y_avg).array().pow(2)).sum();
    R0 = ((z_vec.array() - z_avg).array()*(x_vec.array() - x_avg).array()).sum(); 
    R1 = ((z_vec.array() - z_avg).array()*(y_vec.array() - y_avg).array()).sum(); 

    A = -((L11*R0-L01*R1)/(L00*L11-L01*L01));
    B = -((L00*R1-L01*R0)/(L00*L11-L01*L01));
    C = 1;
    D = -(z_avg+A*x_avg+B*y_avg);

    return fabs(A*pt(0,0)+B*pt(0,1)+C*pt(0,2)+D)/(sqrt(A*A+B*B+C*C));
}

////////////////////////////////////////////////////////////

Eigen::MatrixXd ut::get_traj_wrt_tcp(Eigen::Matrix4d tool_F_T_tcp, std::vector<std::vector<double> > vec)
{
    std::vector<std::vector<double> > traj_from_kuka_scanning_vec;
    traj_from_kuka_scanning_vec = GetUniqueRows(vec);
    
    Eigen::MatrixXd pts_from_tcp_publisher(traj_from_kuka_scanning_vec.size(),traj_from_kuka_scanning_vec[0].size());
    pts_from_tcp_publisher = ut::vec_to_mat(traj_from_kuka_scanning_vec);
    Eigen::Vector3d b_t_Flange;
    Eigen::MatrixXd b_eul_Flange(1,3);
    Eigen::Matrix3d b_r_Flange;
    Eigen::Matrix4d b_T_Flange;
    Eigen::Matrix4d b_T_tcp;
    // Eigen::Matrix3d new_r;
        
    // angles must be radians
    Eigen::MatrixXd transformed_pt = Eigen::MatrixXd::Constant(pts_from_tcp_publisher.rows(),pts_from_tcp_publisher.cols(),0);
    for (int i=0;i<pts_from_tcp_publisher.rows();++i)
    {
        b_t_Flange << pts_from_tcp_publisher(i,0), pts_from_tcp_publisher(i,1), pts_from_tcp_publisher(i,2);
        b_eul_Flange << pts_from_tcp_publisher(i,3), pts_from_tcp_publisher(i,4), pts_from_tcp_publisher(i,5);
        b_r_Flange = rtf::eul2rot(b_eul_Flange);
        b_T_Flange = rtf::hom_T(b_t_Flange,b_r_Flange);
        b_T_tcp = b_T_Flange*tool_F_T_tcp;
        // new_r = b_T_tcp.block(0,0,3,3);
        transformed_pt(i,0) = b_T_tcp(0,3);
        transformed_pt(i,1) = b_T_tcp(1,3);
        transformed_pt(i,2) = b_T_tcp(2,3);
    }

    Eigen::MatrixXd scan_traj_wrt_tcp(transformed_pt.rows(),transformed_pt.cols());

    // saves only xyz and not euler angles
    scan_traj_wrt_tcp = transformed_pt.block(0,0,transformed_pt.rows(),3);
    return scan_traj_wrt_tcp;
}

////////////////////////////////////////////////////////////
