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

int main()
{

    Eigen::MatrixXd model_ptcloud = file_rw::file_read_mat("/home/aniruddha/Desktop/ICP/ICP_SVD/data/Dome_ptcloud.csv");
    Eigen::MatrixXd scan_ptcloud = file_rw::file_read_mat("/home/aniruddha/Desktop/ICP/ICP_SVD/data/DomeScan1.csv");

    Eigen::MatrixXd model_ptcloud_t = model_ptcloud.transpose();
    Eigen::MatrixXd scan_ptcloud_t = scan_ptcloud.transpose();
    
    Nabo::NNSearchD * nd = Nabo::NNSearchD::createKDTreeLinearHeap(model_ptcloud_t);

    int k = 1;
    Eigen::MatrixXi idx(k,scan_ptcloud.rows());
    Eigen::MatrixXd dist(k,scan_ptcloud.rows());   
    nd->knn(scan_ptcloud_t, idx, dist, k);
 
    std::cout << idx.transpose() << std::endl;

	return 0;
}
