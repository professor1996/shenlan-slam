#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
// need pangolin for plotting trajectory
 #include <pangolin/pangolin.h>

using namespace std;

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
 
int main(int argc, char **argv) {
 
     vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,poses2;
     /// implement pose reading code
  // start your code here (5~10 lines)
     ifstream infile;
     infile.open("/home/ubuntu/Documents/SL_pengrui/3/6.trajectory_error/estimated.txt");
     if(!infile) cout<<"error"<<endl;
     
     cout<<"存入数组"<<endl;   //先将文件中的数据存入到一个二维数组中
     double data;
     double a[620][8];
     double *p=&a[0][0];

     double as;
     while(infile>>data)             //遇到空白符结�?     {
         *p=data;
          p++;
          as++;
     }
     infile.close();
     cout << as << endl;

     for(int i=0;i<620;i++)   //分别对每一行数据生成一个变换矩阵，然后存入动态数组poses�?     {    
         Eigen::Quaterniond q1 = Eigen::Quaterniond(a[i][7],a[i][4],a[i][5],a[i][6]);
         Eigen::Vector3d t1;
         t1<<a[i][1],a[i][2],a[i][3];
         Sophus::SE3 SE3_qt1(q1,t1);
         poses1.push_back(SE3_qt1);
     }


     ifstream truth;
     truth.open("/home/ubuntu/Documents/SL_pengrui/3/6.trajectory_error/groundtruth.txt");
     if(!truth) cout<<"error"<<endl;
     
     cout<<"存入数组"<<endl;   //先将文件中的数据存入到一个二维数组中
     double data1;
     double b[620][8];
     double *p1=&b[0][0];

     double af;
     while(truth>>data1)             //遇到空白符结�?     {
         *p1=data1;
          p1++;
         af++;
     }
     cout << af << endl;
     truth.close();
     for(int i=0;i<620;i++)   //分别对每一行数据生成一个变换矩阵，然后存入动态数组poses�?     {    
         Eigen::Quaterniond q2 = Eigen::Quaterniond(b[i][7],b[i][4],b[i][5],b[i][6]);
         Eigen::Vector3d t2;
         t2<<b[i][1],b[i][2],b[i][3];
         Sophus::SE3 SE3_qt2(q2,t2);
         poses2.push_back(SE3_qt2);
     }
     // end your code here
 
     // draw trajectory in pangolin
     DrawTrajectory(poses1,poses2);
     return 0;
 }
 
 /*******************************************************************************************/
 void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
      if (poses1.empty() || poses2.empty() ) {
         cerr << "Trajectory is empty!" << endl;
          return;
      }
  
      // create pangolin window and plot the trajectory
      //创建一个窗�?      pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
      //启动深度测试
      glEnable(GL_DEPTH_TEST);
      //启动混合
      glEnable(GL_BLEND);
      //混合函数glBlendFunc( GLenum sfactor , GLenum dfactor );sfactor 源混合因子dfactor 目标混合因子
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      // Define Projection and initial ModelView matrix
      pangolin::OpenGlRenderState s_cam(
              pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        //对应的是gluLookAt,摄像机位�?参考点位置,up vector(上向�?      
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

































 

