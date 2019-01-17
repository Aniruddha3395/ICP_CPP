#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "stdlib.h"
#include "file_rw.hpp"

int main()
{
    std::string part_ptcloud = "/home/rflin/Desktop/ANIRUDDHA_WS/CPP/data_files/part_ptcloud.csv";
    std::string part_ptcloud_normals = "/home/rflin/Desktop/ANIRUDDHA_WS/CPP/data_files/part_ptcloud.csv";
    std::string scanned_traj = "/home/rflin/Desktop/ANIRUDDHA_WS/CPP/data_files/part_ptcloud.csv";
    
    std::vector<std::vector<double> > part_ptcloud_vec;
    part_ptcloud_vec = file_rw::file_read(part_ptcloud);
    std::vector<std::vector<double> > part_ptcloud_normals_vec;
    part_ptcloud_normals_vec = file_rw::file_read(part_ptcloud_normals);
    std::vector<std::vector<double> > scanned_traj_vec;
    scanned_traj_vec = file_rw::file_read(scanned_traj);
        
    return 0;
}
