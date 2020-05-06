//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

	// 给定Essential矩阵
	Matrix3d E;
	E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
	    0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
	    -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

	// 待计算的R,t
	Matrix3d R;
	Vector3d t;

	// SVD and fix sigular values
	// START YOUR CODE HERE
	vector<vector<float>> vec{{-0.0203618550523477   , -0.4007110038118445  , -0.03324074249824097 },
			      { 0.3939270778216369   , -0.03506401846698079 ,  0.5857110303721015},
			      {-0.006788487241438284 , -0.5815434272915686  , -0.01438258684486258} };


	const int rows{ 3 }, cols{ 3 };

	vector<float> vec_;
	for (int i = 0; i < rows; ++i) {
		vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());
	}
	Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec_.data() , rows , cols);

	JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeFullV | Eigen::ComputeFullU); // ComputeThinU | ComputeThinV
	MatrixXf singular_values = svd.singularValues();
	MatrixXf left_singular_vectors = svd.matrixU();
	MatrixXf right_singular_vectors = svd.matrixV();
	// END YOUR CODE HERE


	// set t1, t2, R1, R2 
	// START YOUR CODE HERE
	Matrix3d t_wedge1;
	Matrix3d t_wedge2;

	Matrix3d R1;
	Matrix3d R2;

	// 沿Z轴转90度的旋转矩阵
	Matrix3d R_po;
	Matrix3d R_mi;

	R_po = AngleAxisd( M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
	R_mi = AngleAxisd(-M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();

	// set sigma:
	Matrix3d sigma ;
	sigma << singular_values(0,0)  , 0                     , 0,
	        0                      , singular_values(1,0)  , 0,
	        0                      , 0                     , singular_values(2,0);

	// set left_singular_vectors
	Matrix3d Left_singular_vectors;
	Left_singular_vectors << left_singular_vectors(0,0) , left_singular_vectors(0,1) , left_singular_vectors(0,2),
		                 left_singular_vectors(1,0) , left_singular_vectors(1,1) , left_singular_vectors(1,2),
		                 left_singular_vectors(2,0) , left_singular_vectors(2,1) , left_singular_vectors(2,2);

	// set right_singular_vectors
	Matrix3d Right_singular_vectors;
	Right_singular_vectors << right_singular_vectors(0,0) , right_singular_vectors(0,1) , right_singular_vectors(0,2),
		                  right_singular_vectors(1,0) , right_singular_vectors(1,1) , right_singular_vectors(1,2),
		             	  right_singular_vectors(2,0) , right_singular_vectors(2,1) , right_singular_vectors(2,2);

	// calculation:
	t_wedge1 = Left_singular_vectors * R_po * sigma * Left_singular_vectors.transpose();
	t_wedge2 = Left_singular_vectors * R_mi * sigma * Left_singular_vectors.transpose();

	R1       = Left_singular_vectors * R_po.transpose() * Right_singular_vectors.transpose();
	R2       = Left_singular_vectors * R_mi.transpose() * Right_singular_vectors.transpose();


	// END YOUR CODE HERE

	cout << "R1 = " << R1 << endl;
	cout << "R2 = " << R2 << endl;
	cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
	cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

	// check t^R=E up to scale
	Matrix3d tR = t_wedge1 * R1;
	cout << "t^R = " << tR << endl;

	// t^R == E!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	return 0;
}



