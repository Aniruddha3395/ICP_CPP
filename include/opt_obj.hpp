#include "nabo/nabo.h"
#include "nlopt.hpp"
#include <Eigen/Eigen>

namespace opt_obj
{

    class opt_obj
    {
        private:

        public:
            // constructor and distructor
            opt_obj(std::vector<double> x_start, int K, Eigen::MatrixXd Input_Part_Ptcloud_Icp, 
                Eigen::MatrixXd Scanned_Traj, const double error_threshold, double Perturb_Val, 
                double w1, double w2, double OptH, double optXtolRel);
            ~opt_obj();

            // other variables
            double W1;
            double W2;
            Nabo::NNSearchD * nns;
            int k;
            double Error_threshold;
            double perturb_val;
            Eigen::MatrixXd input_part_ptcloud_icp;
            Eigen::MatrixXd scanned_traj;
            Eigen::MatrixXd M;
            std::vector<double> x0;

            // optim solver
            int OptVarDim;
            double OptXtolRel;
            double optH;
            std::vector<double> optx;
            std::vector<double> OptVarlb;
            std::vector<double> OptVarub;        
            std::vector<double> solx;
            double solminf;
            nlopt::algorithm optalg;
            nlopt::opt opt;
            double ErrFun(const std::vector<double> &x);
            double ObjFun(const std::vector<double> &x, std::vector<double> &grad);
            bool solveOPT();
            Eigen::Matrix4d solveOPTCust();

    };
}