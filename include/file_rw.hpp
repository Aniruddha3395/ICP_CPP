#include <string>
#include <vector>
#include <Eigen/Eigen>

class file_rw
{
public:
    // read data from file and store in vector
    static std::vector< std::vector<double> > file_read_vec(std::string);

	// read data from file and store in matrix
    static Eigen::MatrixXd file_read_mat(std::string);

    // write data to file from ve vector
    static void file_write_vec(std::string, std::vector< std::vector<double> >);
	
	// write data to file from matrix
    static void file_write_mat(std::string, Eigen::MatrixXd);
    
};