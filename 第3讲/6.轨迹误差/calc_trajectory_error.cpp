#include <cmath>
#include "sophus/so3.h"
#include "sophus/se3.h"
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
 
int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e;

    ifstream estimated_file;
    estimated_file.open("/home/ubuntu/Documents/SL_pengrui/3/6.trajectory_error/estimated.txt");
    if(!estimated_file) cout << "estimated_file open error" << endl;

    double data_e;
    double Te[800][8];
    double *e = &Te[0][0];

    int num_e = 0;
    while(estimated_file >> data_e)
    {
        *e = data_e;
         e++;
         num_e++;
    }
    estimated_file.close();

    for(int i = 0 ; i < num_e / 8 ; i++)
    {
        Eigen::Quaterniond q_e(Te[i][7] , Te[i][4] , Te[i][5] , Te[i][6]);
        Eigen::Vector3d t_e(Te[i][1] , Te[i][2] , Te[i][3]) ;

        Sophus::SE3 SE3_qt_e(q_e , t_e);
        poses_e.push_back(SE3_qt_e);      
    }   
 
/*************************************************************************************************************/

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g;
 
    ifstream groundtruth_file;
    groundtruth_file.open("/home/ubuntu/Documents/SL_pengrui/3/6.trajectory_error/groundtruth.txt");
    if(!groundtruth_file) cout << "groundtruth_file open error" << endl;
     
    double data_g;
    double Tg[800][8];
    double *g = &Tg[0][0];

    int num_g = 0;
    while(groundtruth_file >> data_g)
    {
        *g = data_g;
         g++;
         num_g++;
    }
    groundtruth_file.close();

    for(int i = 0 ; i < num_g / 8 ; i++)
    {
        Eigen::Quaterniond q_g(Tg[i][7] , Tg[i][4] , Tg[i][5] , Tg[i][6]);
        Eigen::Vector3d t_g(Tg[i][1] , Tg[i][2] , Tg[i][3]) ;

        Sophus::SE3 SE3_qt_g(q_g , t_g);
        poses_g.push_back(SE3_qt_g);
    }   

/*************************************************************************************************************/

    double err  = 0 , Err = 0;
    double RMSE = 0;

    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d se3_Tg_Te;
 
    for(int i = 0 ; i < poses_g.size() ; i++)
    {
        se3_Tg_Te = (poses_g[i].inverse() * poses_e[i]).log();
        err      += se3_Tg_Te.transpose() * se3_Tg_Te;          
    }
 
    RMSE = sqrt(err / poses_g.size());
    cout << RMSE << endl;
 
    return 0;
}


 















 
