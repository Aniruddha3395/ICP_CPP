#include <iostream>
#include <C:/Users/aniru/OneDrive/Desktop/CPP/eigen/Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>

using namespace Eigen;


std::vector <std::vector <double>> read_file(std::string);
MatrixXd vec_to_mat(std::vector<std::vector<double>>);
MatrixXd apply_transformation(MatrixXd, Matrix4d);
void compute_TCP(MatrixXd, MatrixXd);

int main()
{
    std::string file_name = "data_files/abc.txt";
    std::vector<std::vector<double>> v = read_file(file_name);
    MatrixXd M = vec_to_mat(v);
    // std::cout << M << std::endl;

    Matrix4d T = Matrix4d::Random();
    std::cout << T << std::endl;
     
    MatrixXd N = apply_transformation(M,T);
    std::cout << N << std::endl;
        
}


std::vector <std::vector <double>> read_file(std::string file_name)
{
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


MatrixXd vec_to_mat(std::vector<std::vector<double>> vec)
{
    MatrixXd mat(vec.size(), vec[0].size());
    for (int i = 0; i < vec.size(); ++i)
        mat.row(i) = VectorXd::Map(&vec[i][0], vec[0].size());
    return mat;
}

MatrixXd apply_transformation(MatrixXd data, Matrix4d T_mat)
{
    //NOTE: Homogeneous Tranformation Matrix (4x4)

    // putting data in [x, y, z, 1]' format
    MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
    VectorXd ones_vec = VectorXd::Constant(data.rows(),1);
    data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
    data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
    MatrixXd transformed_data = T_mat*data_with_fourth_row;
    MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
    
    transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());

    return transformed_data_mat.transpose();
}

void compute_TCP(MatrixXd data_points, MatrixXd normals)
{
    // tcp computation...bx,by,bz
    Vector3d tool_x;
    Vector3d tool_y;
    Vector3d tool_z;
    Vector3d dir_vec;
        
    MatrixXd bx(data_points.rows(),3);
    MatrixXd by(data_points.rows(),3);
    MatrixXd bz(data_points.rows(),3);

    for (int i=0; i<data_points.rows(); ++i)
    {
        if (i!=data_points.rows()-1)
        {
            Vector3d direction;
            
            // calculating direction vector from sequence of points
            // direction = data_points.block(i+1,0,1,data_points.cols()) - data_points.block(i,0,1,data_points.cols()); 
            // OR
            // applying constant direction vector
            direction << 0, 1, 0;
            
            dir_vec = direction/direction.norm();    
        }
        tool_z = -normals.block(i,0,1,normals.cols());
        tool_x = dir_vec.cross(tool_z);
        tool_x = tool_x/tool_x.norm();
        tool_y = tool_z.cross(tool_x);
        tool_y = tool_y/tool_y.norm();

        bx.block(i,0,1,3) = tool_x;
        by.block(i,0,1,3) = tool_y;
        bz.block(i,0,1,3) = tool_z;
    }
}