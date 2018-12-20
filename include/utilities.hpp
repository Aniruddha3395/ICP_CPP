#include <Eigen/Dense>
#include <vector>
#include <string>

class ut
{
public:
	static Eigen::MatrixXd vec_to_mat(std::vector<std::vector<double> >);
	static void compute_TCP(Eigen::MatrixXd, Eigen::MatrixXd);
	static double get_pt_to_lsf_plane_dist(Eigen::MatrixXd, Eigen::MatrixXd);
	static Eigen::MatrixXd get_traj_wrt_tcp(Eigen::Matrix4d, std::vector<std::vector<double> >);
};
