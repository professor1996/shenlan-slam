#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

 

int main ( int argc, char** argv )
{
     //define variables
     //q1 , q2
     Eigen::Quaterniond q1( 0.55 ,   0.3 ,  0.2 , 0.2); //expression of quaternion(w ,x,y,z)
     Eigen::Quaterniond q2(-0.1  ,   0.3 , -0.7 , 0.2); //expression of quaternion(w ,x,y,z)
 
     //t1 , t2
     Eigen::Vector3d t1( 0.7 , 1.1 , 0.2);
     Eigen::Vector3d t2(-0.1 , 0.4 , 0.8);

     //p1 , p2
     Eigen::Vector3d p1(0.5 , -0.1 , 0.2);   //coordinate in luobo-1
     Eigen::Vector3d p2;                     //coordinate in luobo-2
 
     //pw
     Eigen::Vector3d pw ;                    //world coordinate

     //world->q1 , q2   transform Matrix
     Eigen::Isometry3d T_q1w = Eigen::Isometry3d::Identity();
     Eigen::Isometry3d T_q2w = Eigen::Isometry3d::Identity();
     
     //normalizition of quaternion
     q1.normalize();
     q2.normalize();
 
     /*设置变换矩阵的参数*/
     T_q1w.rotate(q1);
     T_q1w.pretranslate(t1);
     T_q2w.rotate(q2);
     T_q2w.pretranslate(t2);
 
     /* p1 = T_1w * pw  solve pw*/
     pw = T_q1w.inverse() * p1;
 
     /* p2 = T_2w * pw  solve p2*/
     p2 = T_q2w * pw;
 
     //output coordinate under xiao_luobo-2
     cout<<"该点在小萝卜二号下的坐标为: "<< p2.transpose() <<endl;
 
     return 0;

}







