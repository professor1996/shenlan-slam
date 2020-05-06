//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

//string p3d_file = "../p3d.txt";
//string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream p3d_file;
    p3d_file.open("../p3d.txt");
    if(!p3d_file)cout << "open error" << endl;

    double Data_p3d;
    double P3d[76][3];
    double *p_3d = &P3d[0][0];

    while(p3d_file >> Data_p3d)
    {
        *p_3d = Data_p3d;
         p_3d++;
    }

    for(int i = 0 ; i < 76 ; i++)
    {
        Vector3d  P_3d(P3d[i][0] , P3d[i][1] , P3d[i][2]);
        p3d.push_back(P_3d);
    }
    ////////
    ifstream p2d_file;
    p2d_file.open("../p2d.txt");
    if(!p2d_file)cout << "open error" << endl;

    double Data_p2d;
    double P2d[76][2];
    double *p_2d = &P2d[0][0];

    while(p2d_file >> Data_p2d)
    {
        *p_2d = Data_p2d;
         p_2d++;
    }

    for(int i = 0 ; i < 76 ; i++)
    {
        Vector2d  P_2d(P2d[i][0] , P2d[i][1]);
        p2d.push_back(P_2d);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

//////////////////////////////////////////////////////////////

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();//p3d.size();

    Sophus::SE3 T_esti; // estimated pose
    Vector6d se3  ;

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        // compute cost
        cost = 0;
        
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector4d P3D_ex(p3d[i][0] , p3d[i][1] , p3d[i][2] , 1);
            Vector4d world_camera_XYZ = T_esti.matrix() * P3D_ex;
            Vector3d world_camera_XYZ_OFF1(world_camera_XYZ[0] ,  world_camera_XYZ[1] , world_camera_XYZ[2]);

            Vector3d U_camera(p2d[i][0] , p2d[i][1] , 1);
            Vector3d vector_error;
            Vector2d vector_error_OFF1;

            vector_error = U_camera - K * world_camera_XYZ_OFF1 / world_camera_XYZ[2];//!!!!!!!!!!!!!! /world_camera_XYZ[2]
            vector_error_OFF1 << vector_error[0] , vector_error[1];

            cost += vector_error_OFF1[0] * vector_error_OFF1[0] + vector_error_OFF1[1] * vector_error_OFF1[1];
     
	        // END YOUR CODE HERE

	        // compute jacobian
            Matrix<double, 2, 6> J;
            double X_1 , Y_1 , Z_1;
            // START YOUR CODE HERE 
            
            X_1 = world_camera_XYZ[0];
            Y_1 = world_camera_XYZ[1];
            Z_1 = world_camera_XYZ[2];

            J << fx / Z_1 , 0        , -fx * X_1 / (Z_1 * Z_1) , -fx * X_1 * Y_1 / (Z_1 * Z_1)       , fx * (1 + X_1 * X_1 / (Z_1 * Z_1)) , -fx * Y_1 / Z_1 ,
                        0 , fy / Z_1 , -fy * Y_1 / (Z_1 * Z_1) , -fy * (1 + Y_1 * Y_1 / (Z_1 * Z_1)) , fy * X_1 * Y_1 / (Z_1 * Z_1)       , fy * X_1 / Z_1;
	        // END YOUR CODE HERE

            H += J.transpose() * J;
            b += J.transpose() * vector_error_OFF1;

        }

	    // solve dx 
        Vector6d dx;
        
        // START YOUR CODE HERE 

        dx = H.ldlt().solve(b);
        
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }
 
        // update your estimation
        // START YOUR CODE HERE 

        //se3 += dx;
        T_esti = Sophus::SE3::exp(dx) * T_esti;

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost= " << cout.precision(12) <<  cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}


