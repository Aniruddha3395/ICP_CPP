#include <iostream>
#include <C:/Users/aniru/OneDrive/Desktop/CPP/eigen/Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>

using namespace Eigen;


std::vector <std::vector <double>> file_read(std::string file_name);

Matrix3d ret_mat()
{
    Matrix3d m = Matrix3d::Constant(3,3,4);
    return m;
}

int main()
{
    Matrix3d aa;
    aa = ret_mat();
    std::cout << aa << std::endl;

    std::string file_name = "data_files/abc.txt";
    std::vector<std::vector<double>> v = file_read(file_name);
    
    MatrixXd a(v.size(), v[0].size());
    for (int i = 0; i < v.size(); ++i)
        a.row(i) = VectorXd::Map(&v[i][0], v[0].size());
    
    std::cout << a << std::endl;
}


std::vector <std::vector <double>> file_read(std::string file_name)
{
    int j;
    char arr[255];
    char *tok;
    std::vector <std::vector <double>> vec;
    std::string line;
    std::ifstream input_file;
    input_file.open(file_name);
   while(!input_file.eof())
   {
        getline(input_file,line);
        if(!line.empty())
        {
            line.copy(arr,line.size());
            tok = std::strtok(arr,",");
            std::vector<double>row_vec; 
            while(tok!=NULL)
            {
                row_vec.push_back(atof(tok));
                tok = strtok(NULL,",");
            }
            vec.push_back(row_vec);
        }  
    }
    input_file.close();
    return vec;
} 

