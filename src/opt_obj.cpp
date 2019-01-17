#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include "nabo/nabo.h"
#include "nlopt.hpp"
#include "stdlib.h"
#include "utilities.hpp"
#include "file_rw.hpp"
#include "transformation_utilities.hpp"
#include "opt_obj.hpp"

namespace opt_obj
{
    // defualt nlopt fuction
    double customminfunc(const std::vector<double>& x, std::vector<double>& grad, void* data) {
        // Auxilory function to minimize (Sum of Squared distance between correspondance
        // from both pointclouds). Because we wanted a Class
        // without static members, but NLOpt library does not support
        // passing methods of Classes, we use these auxilary functions.
        opt_obj *c = (opt_obj *) data;
        return c->ObjFun(x,grad);
    }

    // non-linear equality constraints
    double nonlcon(const std::vector<double> &x, std::vector<double> &grad, void* data)
    {
        if (!grad.empty()) 
        {
            grad[0] = 0.0;
            grad[1] = 0.0;
            grad[2] = 0.0;
            grad[3] = 2*x[3];
            grad[4] = 2*x[4];
            grad[5] = 2*x[5];
            grad[6] = 2*x[6];
        }
        return (x[3]*x[3] + x[4]*x[4] + x[5]*x[5] + x[6]*x[6] -1);
    }

    // class constructor
    opt_obj::opt_obj(std::vector<double> x_start, int K, Eigen::MatrixXd Input_Part_Ptcloud_Icp,
        Eigen::MatrixXd Scanned_Traj, const double error_threshold, double Perturb_Val,
        double w1, double w2, double OptH, double optXtolRel) 
    {
        //choose optimizer
        // optalg = nlopt::LN_NEWUOA;
        // optalg = nlopt::LN_NEWUOA_BOUND;
        // optalg = nlopt::LN_BOBYQA;
        // optalg = nlopt::LN_COBYLA;
        optalg = nlopt::LD_SLSQP;
        
        // optalg = nlopt::GN_ISRES;
        
        
        k = K;
        W1 = w1; 
        W2 = w2;
        optH = OptH;
        OptXtolRel = optXtolRel;
        input_part_ptcloud_icp = Input_Part_Ptcloud_Icp;
        scanned_traj = Scanned_Traj;
        x0 = x_start;
        Error_threshold = error_threshold;
        perturb_val = Perturb_Val;
        M = Eigen::MatrixXd(input_part_ptcloud_icp.cols(),input_part_ptcloud_icp.rows());
        M << input_part_ptcloud_icp.transpose();
        
        // create a kd-tree for M, note that M must stay valid during the lifetime of the kd-tree
        Nabo::NNSearchD * nns_temp = Nabo::NNSearchD::createKDTreeLinearHeap(M);
        nns = nns_temp;

        // optimization params
        OptVarDim = x_start.size();
        OptVarlb.resize(OptVarDim);
        OptVarub.resize(OptVarDim);

        // OptVarlb[0] = -20; OptVarub[0] = 20;
        // OptVarlb[1] = -20; OptVarub[1] = 20;
        // OptVarlb[2] = -20; OptVarub[2] = 20;
        OptVarlb[0] = -0.020; OptVarub[0] = 0.020;
        OptVarlb[1] = -0.020; OptVarub[1] = 0.020;
        OptVarlb[2] = -0.020; OptVarub[2] = 0.020;
        OptVarlb[3] = -1;  OptVarub[3] = 1;
        OptVarlb[4] = -1;  OptVarub[4] = 1;
        OptVarlb[5] = -1;  OptVarub[5] = 1;
        OptVarlb[6] = -1;  OptVarub[6] = 1;
        
        opt = nlopt::opt(optalg, OptVarDim);
        opt.set_lower_bounds(OptVarlb);
        opt.set_upper_bounds(OptVarub);
        opt.set_xtol_rel(OptXtolRel);
        optx.resize(OptVarDim);
        opt.set_min_objective(customminfunc, this);
        opt.add_equality_constraint(nonlcon, NULL, 1e-8);
    }

    // class destructor
    opt_obj::~opt_obj()
    {
        delete nns;
    }

    double opt_obj::ErrFun(const std::vector<double> &x)
    {
        double E = 0;
        Eigen::Vector3d t;
        Eigen::MatrixXd r(1,4);
        Eigen::Matrix4d transformation_matrix;
        
        t << x[0],x[1],x[2];
        r << x[3],x[4],x[5],x[6];
        transformation_matrix = rtf::hom_T(t, rtf::qt2rot(r));

        Eigen::MatrixXd transformed_data(input_part_ptcloud_icp.rows(),input_part_ptcloud_icp.cols());
        transformed_data = rtf::apply_transformation(input_part_ptcloud_icp,transformation_matrix);

        // Eigen::MatrixXd q(3,1);
        // Eigen::VectorXi indices(k);
        // Eigen::VectorXd dists(k);

        double d[scanned_traj.rows()];
        long double sum_d = 0.0;
        double max_d = -10000000;
        
        Eigen::MatrixXd q = scanned_traj.transpose();
        Eigen::MatrixXi indices(k, q.cols());
        Eigen::MatrixXd dists(k, q.cols());
        
        nns->knn(q, indices, dists, k);

        Eigen::MatrixXd corresponding_val_from_part_ptcloud(k,3);
        for (long int i=0;i<scanned_traj.rows();++i)
        {
            for (int j=0;j<k;++j)
            {
                corresponding_val_from_part_ptcloud.row(j) = transformed_data.row(indices(j,i));
            }

            d[i] = ut::get_pt_to_lsf_plane_dist(q.col(i).transpose(),corresponding_val_from_part_ptcloud);
            sum_d += d[i];
            if (d[i] > max_d)
            {
                max_d=d[i];
            }
        }

        E = ((W1*sum_d)/scanned_traj.rows()) + (W2*max_d);
               
        // std::cout << E << std::endl;
        return E;
    };

    bool opt_obj::solveOPT()
    {
        solx = x0;
        bool successFlag = false;
        try{
            nlopt::result result = opt.optimize(solx, solminf);
            successFlag = true;
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        return successFlag;
    };

    Eigen::Matrix4d opt_obj::solveOPTCust()
    {
        Eigen::Matrix4d icp_T;
        Eigen::Vector3d icp_t;
        Eigen::MatrixXd icp_qt(1,4);
        Eigen::MatrixXd icp_eul(1,3);
        Eigen::Matrix4d icp_T_final = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d icp_T_final_save;
        Eigen::Matrix4d Final_w_T_p;
        Eigen::MatrixXd input_part_ptcloud_icp_save(input_part_ptcloud_icp.rows(),input_part_ptcloud_icp.cols());


        double fval_curr = 1e6;
        
        while(fval_curr>Error_threshold)
        {
            // optimization routine
            solveOPT();

            icp_t << solx[0], solx[1], solx[2];
            icp_qt << solx[3], solx[4], solx[5], solx[6];
            icp_T = rtf::hom_T(icp_t,rtf::qt2rot(icp_qt));
            if (solminf<fval_curr)
            {
                fval_curr=solminf;
                for (int j=0;j<x0.size();++j)
                {
                    x0[j] = solx[j];    
                }
                icp_T_final = icp_T*icp_T_final;
                input_part_ptcloud_icp = rtf::apply_transformation(input_part_ptcloud_icp,icp_T);
                icp_T_final_save = icp_T_final;
                input_part_ptcloud_icp_save = input_part_ptcloud_icp;
                // std::cout << fval_curr << "," << solminf << std::endl;
            }
            else
            {
                x0 = {0,0,0,0,0,0,1};
                icp_t(0) = solx[0] - perturb_val + 2*perturb_val*((double) rand() / (double)(RAND_MAX));
                icp_t(1) = solx[1] - perturb_val + 2*perturb_val*((double) rand() / (double)(RAND_MAX));
                icp_t(2) = solx[2] - perturb_val + 2*perturb_val*((double) rand() / (double)(RAND_MAX));
                icp_eul = rtf::qt2eul(icp_qt);
                // icp_eul(0,0) = (icp_eul(0,0) - 0.5*perturb_val + 1*perturb_val*((double) rand() / (double)(RAND_MAX)))*(3.14159/180);
                // icp_eul(0,1) = (icp_eul(0,1) - 0.5*perturb_val + 1*perturb_val*((double) rand() / (double)(RAND_MAX)))*(3.14159/180);
                // icp_eul(0,2) = (icp_eul(0,2) - 0.5*perturb_val + 1*perturb_val*((double) rand() / (double)(RAND_MAX)))*(3.14159/180);
                
                icp_eul(0,0) = icp_eul(0,0) + (- 0.5*perturb_val + 1*perturb_val*((double) rand() / (double)(RAND_MAX)))*(3.14159/180);
                icp_eul(0,1) = icp_eul(0,1) + (- 0.5*perturb_val + 1*perturb_val*((double) rand() / (double)(RAND_MAX)))*(3.14159/180);
                icp_eul(0,2) = icp_eul(0,2) + (- 0.5*perturb_val + 1*perturb_val*((double) rand() / (double)(RAND_MAX)))*(3.14159/180);
                
                icp_T = rtf::hom_T(icp_t,rtf::eul2rot(icp_eul));
                icp_T_final = icp_T*icp_T_final;
                input_part_ptcloud_icp = rtf::apply_transformation(input_part_ptcloud_icp, icp_T);
            }
            std::cout << fval_curr << "," << solminf << std::endl;
        }    
        return icp_T_final_save;
    };

    // gradient computation:
    // Central Difference Method
    double opt_obj::ObjFun(const std::vector<double> &x, std::vector<double> &grad)
    {
        double err = ErrFun(x);
        if (!grad.empty()) {
            std::vector<double> xphp = x;
            std::vector<double> xphn = x;
            for (uint i=0; i < x.size(); ++i)
            {
                xphp[i] += optH;
                xphn[i] -= optH;
                grad[i] = (ErrFun(xphp)-ErrFun(xphn))/(2*optH);
                xphp[i] -= optH;
                xphn[i] += optH;
            }
        }    
        return err;

    };

    // Forward Difference Method
    // double opt_obj::ObjFun(const std::vector<double> &x, std::vector<double> &grad)
    // {
    //     double err = ErrFun(x);
    //     if (!grad.empty()) {
    //         std::vector<double> xph = x;
    //         for (uint i=0; i < x.size(); ++i)
    //         {
    //             xph[i] += optH;
    //             grad[i] = (ErrFun(xph)-err)/optH;
    //             xph[i] -= optH;
    //         }
    //     }    
    //     return err;
    // };

    // Backward Difference Method
    // double opt_obj::ObjFun(const std::vector<double> &x, std::vector<double> &grad)
    // {
    //     double err = ErrFun(x);
    //     if (!grad.empty()) {
    //         std::vector<double> xph = x;
    //         for (uint i=0; i < x.size(); ++i)
    //         {
    //             xph[i] -= optH;
    //             grad[i] = (err - ErrFun(xph))/optH;
    //             xph[i] += optH;
    //         }
    //     }    
    //     return err;
    // };

}