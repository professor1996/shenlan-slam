#include <iostream>

#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>


/****************************
* 本程序use QR and Cholesky solve equation of x
****************************/

using namespace std;
int main( int argc, char** argv )
{
   //define 100 * 100 Matrix
   int matrix_limit = 100;

   //define coefficient Matrix A
   Eigen::Matrix <double , Eigen::Dynamic, Eigen::Dynamic > matrix_A;
   matrix_A = Eigen::MatrixXd::Random( matrix_limit, matrix_limit );

   //define solution Matrix b
   Eigen::Matrix <double , Eigen::Dynamic, 1              > matrix_b;
   matrix_b = Eigen::MatrixXd::Random( matrix_limit, 1 );
      
   //Ax = b
   Eigen::Matrix <double , Eigen::Dynamic, 1              > matrix_x;
   matrix_x.resize(matrix_limit , 1);

   //direct inverse:
   //matrix_x = matrix_A.inverse() * matrix_b;

   //QR decomposition
   matrix_x = matrix_A.colPivHouseholderQr().solve(matrix_b);
   cout << "result of QR decomposition:" << endl;  
   cout << matrix_x << endl << endl;

   //Cholesky decomposition (ldlt decomposition is improvement for cholesky)
   matrix_x = matrix_A.ldlt().solve(matrix_b);
   cout << "result of Cholesky decomposition:" << endl;
   cout << matrix_x << endl << endl;
 
 

   return 0;
}





