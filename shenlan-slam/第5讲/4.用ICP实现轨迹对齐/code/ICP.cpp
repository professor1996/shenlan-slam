//
// Created by xiang on 12/21/17.
//
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sophus/se3.h"
#include <sophus/so3.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> ori_poses1 , ori_poses2;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> now_poses1 , now_poses2;

    VecVector3d p1 , p2;
 
    ifstream infile;
    infile.open("../compare.txt");
    if(!infile) cout<<"error"<<endl;

    double data = 0;
    double a[612][16];
    double *p=&a[0][0];

    double as = 0;
    while(infile >> data)             //遇到空白符结束
    {
        *p=data;
         p++;
         as++;
    }
    infile.close();

    for(int i=0;i<612;i++)   //分别对每一行数据生成一个变换矩阵，然后存入动态数组poses中
    {    
        // p1
        Eigen::Quaterniond q1 = Eigen::Quaterniond(a[i][7],a[i][4],a[i][5],a[i][6]);
        Eigen::Vector3d t1;
        t1 << a[i][1],a[i][2],a[i][3];
        p1.push_back(t1);
        Sophus::SE3 SE3_qt1(q1,t1);
        ori_poses1.push_back(SE3_qt1);
        
        // p2
        Eigen::Quaterniond q2 = Eigen::Quaterniond(a[i][15],a[i][12],a[i][13],a[i][14]);
        Eigen::Vector3d t2;
        t2 << a[i][9],a[i][10],a[i][11];
        p2.push_back(t2);
        Sophus::SE3 SE3_qt2(q2,t2);
        ori_poses2.push_back(SE3_qt2);
    }
    // cout << ori_poses2[1] << endl;
    // double iterations = 100;    // 迭代次数
    // double cost = 0, lastCost = 0;  // 本次迭代的cost和上一次迭代的cost 
    // Sophus::SE3 T_esti; // estimated pose   

    // for (double iter = 0; iter < iterations; iter++)
    // {
    //     Matrix3d H = Matrix3d::Zero();             // Hessian = J^T J in Gauss-Newton
    //     Vector3d b = Vector3d::Zero();             // bias
    //     cost = 0;

    //     for (int i = 0; i < 612; i++) {
    //         Sophus::SE3 error_T; 
    //         double error = 0;   // 第i个数据点的计算误差
    //         error_T = (ori_poses1[i].inverse() * T_esti * ori_poses2[i]).log(); 
    //         error += error_T.transpose() * error_T;
    //     }
    // }
    
    // solve heavy_point :
    Eigen::Vector3d P1_h , P2_h;
    for(int i = 0 ; i < p1.size() ; i++)
    {
        P1_h += p1[i];
    }
    P1_h = P1_h / p1.size();

    for(int i = 0 ; i < p2.size() ; i++)
    {
        P2_h += p2[i];
    }
    P2_h = P2_h / p2.size();
 
    // solve point off-heavy_point :
    VecVector3d q1 , q2;
    for(int i = 0 ; i < p1.size() ; i++)
    {
        q1.push_back(p1[i] - P1_h);
    }  

    for(int i = 0 ; i < p2.size() ; i++)
    {
        q2.push_back(p2[i] - P2_h);
    }  

    // define Matrix W = sigma(q1 * q2.transpose())
    Matrix3d W;
    for(int i = 0 ; i < 612 ; i++)
    {
        W += q1[i] * q2[i].transpose();
    }  
   
    // solve SVD deposition
    vector<vector<double>> vec;
    
    double a1[3] = {W(0 , 0) , W(0 , 1) , W(0 , 2)};
    double a2[3] = {W(1 , 0) , W(1 , 1) , W(1 , 2)};
    double a3[3] = {W(2 , 0) , W(2 , 1) , W(2 , 2)};

    vector<double> b1(a1 , a1+3);
    vec.push_back(b1);

    vector<double> b2(a2 , a2+3);
    vec.push_back(b2);

    vector<double> b3(a3 , a3+3);
    vec.push_back(b3);

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

    // r(W) = 3 , R = U * V^T  
    Matrix3d R;
    R = Left_singular_vectors * Right_singular_vectors.transpose();
    // t = p - R * p'
    Vector3d t;
    t = P1_h - R * P2_h;

    Sophus::SE3 Tge(R , t);
    cout << "Tge : \n" << Tge.matrix() << endl;

    // transform coordinate:
    for(int i = 0 ;i < 612 ; i++)     
    {    
        // T1
        Sophus::SE3 T1;
        T1 = Tge * ori_poses1[i];
        now_poses1.push_back(T1);
        
        // T2
        Sophus::SE3 T2;
        T2 = Tge * ori_poses2[i];
        now_poses2.push_back(T2);
    }    

    DrawTrajectory(now_poses1 , now_poses2);
    return 0;
}

/*******************************************************************************************/
 void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
      if (poses1.empty() || poses2.empty() ) {
         cerr << "Trajectory is empty!" << endl;
          return;
      }
  
      // create pangolin window and plot the trajectory
      //创建一个窗口
      pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
      //启动深度测试
      glEnable(GL_DEPTH_TEST);
      //启动混合
      glEnable(GL_BLEND);
      //混合函数glBlendFunc( GLenum sfactor , GLenum dfactor );sfactor 源混合因子dfactor 目标混合因子
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      // Define Projection and initial ModelView matrix
      pangolin::OpenGlRenderState s_cam(
              pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)      
              pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
      );
  
      pangolin::View &d_cam = pangolin::CreateDisplay()
              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
              .SetHandler(new pangolin::Handler3D(s_cam));
 
 
     while (pangolin::ShouldQuit() == false) {
         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
         d_cam.Activate(s_cam);
         glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
 
         glLineWidth(2);
         for (size_t i = 0; i < poses1.size() - 1; i++) {
             glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());
             glBegin(GL_LINES);
             auto p1 = poses1[i], p2 = poses1[i + 1];
             glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
             glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
             glEnd();
         }
        
         for (size_t j = 0; j < poses2.size() - 1; j++) {
             glColor3f(1 - (float) j / poses2.size(), 0.0f, (float) j / poses2.size());
             glBegin(GL_LINES);
             auto p3 = poses2[j], p4 = poses2[j + 1];
             glVertex3d(p3.translation()[0], p3.translation()[1], p3.translation()[2]);
             glVertex3d(p4.translation()[0], p4.translation()[1], p4.translation()[2]);
             glEnd();
        
         }
         pangolin::FinishFrame();
         usleep(5000);   // sleep 5 ms


    }
}


























































