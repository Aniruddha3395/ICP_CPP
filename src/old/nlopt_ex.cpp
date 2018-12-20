#include <iostream>
#include <Eigen/Eigen>
#include <string>
#include "opt_obj.hpp"

int main(){


    const int K = 5;
    
    double abc[3] = {1,2,3};
    double xyz[3] = {4,5,6};

    opt_obj::opt_obj OptObj(K, xyz);
    OptObj.solveOPT();

    std::cout << OptObj.solminf << std::endl;
    for (int i =0; i < OptObj.OptVarDim; ++i)
        std::cout << OptObj.solx[i] << ", ";
    std::cout << std::endl;

    return 0;

}