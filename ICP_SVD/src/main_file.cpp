#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <ctime>
#include "nabo/nabo.h"
#include "stdlib.h"
#include "utilities.hpp"
#include "file_rw.hpp"
#include "transformation_utilities.hpp"
#include <chrono>

Eigen::Matrix4d ICP_SVD_with_AnchorPoints(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, int);

int main()
{
    // perfromance measure

    int max_iter = 300;
    
    Eigen::MatrixXd part_pts = file_rw::file_read_mat("/home/aniruddha/Desktop/ICP/ICP_SVD/data/part_ptsDome1.csv");
    Eigen::MatrixXd scan_pts = file_rw::file_read_mat("/home/aniruddha/Desktop/ICP/ICP_SVD/data/scan_ptsDome1.csv");
    Eigen::MatrixXd model_ptcloud = file_rw::file_read_mat("/home/aniruddha/Desktop/ICP/ICP_SVD/data/Dome_ptcloud.csv");
    Eigen::MatrixXd scan_ptcloud = file_rw::file_read_mat("/home/aniruddha/Desktop/ICP/ICP_SVD/data/DomeScan1.csv");            
    
    Eigen::Matrix4d Transform_mat_new = ICP_SVD_with_AnchorPoints(model_ptcloud, scan_ptcloud, part_pts, scan_pts, max_iter);
    std::cout << Transform_mat_new << std::endl;
    return 0;
}

Eigen::Matrix4d ICP_SVD_with_AnchorPoints(Eigen::MatrixXd model_ptcloud, Eigen::MatrixXd scan_ptcloud, Eigen::MatrixXd part_pts, Eigen::MatrixXd scan_pts, int max_iter)
{
    // NOTE: gives transformation matrix scan_T_model (part w.r.t. robot frame assuming that scan_ptcloud is with respect to robot frame) 

    Eigen::Matrix4d T_init = Eigen::Matrix4d::Identity();
    if (part_pts.rows()>0 && scan_pts.rows()>0)
    {
        T_init = rtf::get_rob_T_part(part_pts,scan_pts);
    }
    Eigen::MatrixXd scan_ptcloud_transformed = rtf::apply_transformation(scan_ptcloud,T_init.inverse());
    scan_ptcloud = scan_ptcloud_transformed;
    // create a kd-tree for M, note that M must stay valid during the lifetime of the kd-tree
    Eigen::MatrixXd model_ptcloud_t = model_ptcloud.transpose();
    Nabo::NNSearchD * nd = Nabo::NNSearchD::createKDTreeLinearHeap(model_ptcloud_t);
    Eigen::Matrix4d Transform_mat_new = T_init.inverse();
    Eigen::Matrix4d Transform_mat = Eigen::Matrix4d::Constant(0);
    int k = 1;
    double tol = 1e-6;
    int count = 0;
    Eigen::MatrixXi idx(k,scan_ptcloud.rows());
    Eigen::MatrixXd dist(k,scan_ptcloud.rows());
    Eigen::MatrixXd corresponding_val_from_model_ptcloud(scan_ptcloud.rows(), scan_ptcloud.cols());    
    for (unsigned int iter=0;iter<max_iter;++iter)
    {
        nd->knn(scan_ptcloud.transpose(), idx, dist, k);   
        for (unsigned int i=0;i<idx.cols();++i)
        {
            corresponding_val_from_model_ptcloud.row(i) = model_ptcloud.row(idx(0,i));    
        }
        // get transformation matrix
        Transform_mat = rtf::get_rob_T_part(corresponding_val_from_model_ptcloud,scan_ptcloud);
        Transform_mat_new = Transform_mat.inverse()*Transform_mat_new;        
        if(Transform_mat(0,3)<tol && Transform_mat(1,3)<tol && Transform_mat(2,3)<tol) 
        {
            count++;
            if(count>5)
            {
                break;
            }
        }
        scan_ptcloud_transformed = rtf::apply_transformation(scan_ptcloud,Transform_mat.inverse());
        scan_ptcloud = scan_ptcloud_transformed;
    }
    delete(nd);
    return Transform_mat_new.inverse();
}
