// This example is in the public domain
#include <iostream>
#include <Eigen/Eigen>
#include "nabo/nabo.h"


	Nabo::NNSearchD * dummy(Eigen::MatrixXd Mt){
		std::cout << "In Main Func!!" << std::endl;
		
		// Eigen::MatrixXd M(Mt.cols(),Mt.rows());
		// M << Mt.transpose();
		// std::cout << M.rows() << std::endl;
		// std::cout << M.cols() << std::endl;
		
		// create a kd-tree for M, note that M must stay valid during the lifetime of the kd-tree
		Nabo::NNSearchD* nns = Nabo::NNSearchD::createKDTreeLinearHeap(Mt);
		int dim = nns->dim;
		std::cout << nns << std::endl;
		return nns;
	}



int main()
{

	Eigen::MatrixXd Mt = Eigen::MatrixXd::Random(3,33333);

	Nabo::NNSearchD * nns_temp;
	nns_temp = dummy(Mt);
		int dim = nns_temp->dim;
		std::cout << nns_temp << std::endl;

	std::cout << "tree made" << std::endl;

	// for (int i=0;i<10000;++i)
	// {
	Eigen::MatrixXd q = Eigen::MatrixXd::Random(3,1);

	const int K = 5;
	Eigen::VectorXi indices(K);
	Eigen::VectorXd dists2(K);
	dummy(Mt)->knn(q, indices, dists2, K);
	// }

	// // cleanup kd-tree
	// delete nns_temp;

	// std::cout << "Ending Main Func!!" << std::endl;
	
	return 0;
}



	