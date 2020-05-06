#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "RMVideoCapture.hpp"
using namespace cv;
using namespace std;



RMVideoCapture::CAM_PARA campara;

Mat img;
RMVideoCapture *pcap2;

static void setpara(int para,void *p)
{
    if (p==NULL)
        return;
    RMVideoCapture *pcap=(RMVideoCapture *)p;
    pcap->cam_para=campara;
    pcap->setpara();
}

int main(int argc, char** argv) {
    


    
    //VideoCapture cap(0);
    RMVideoCapture cap2("/dev/video0");
    pcap2=&cap2;
    cap2.info();
    cout<<"\n  Pre Parameter  \n"<<endl;
    cap2.getCurrentSetting();

    campara=cap2.cam_para;
  

    namedWindow("img", 1);  
    createTrackbar("gain", "img",&campara.gain,100,setpara);  
    createTrackbar("exposure", "img",&campara.exposure,100,setpara,pcap2);  
    createTrackbar("brightness", "img",&campara.brightness,128,setpara,pcap2);  
    createTrackbar("whiteness", "img",&campara.whiteness,500,setpara,pcap2);  
    createTrackbar("saturation", "img",&campara.saturation,128,setpara,pcap2);  
    createTrackbar("contrast", "img",&campara.contrast,64,setpara,pcap2); 

    setTrackbarPos("gain", "img", cap2.cam_para.gain);
    setTrackbarPos("exposure", "img", cap2.cam_para.exposure);
    setTrackbarPos("brightness", "img", cap2.cam_para.brightness);
    setTrackbarPos("whiteness", "img", cap2.cam_para.whiteness);
    setTrackbarPos("saturation", "img", cap2.cam_para.saturation);
    setTrackbarPos("contrast", "img", cap2.cam_para.contrast);



    setpara(0,NULL);
    cap2.startStream();
    char c='1';
    while(1)
    {
        cap2>>img;
        imshow("img",img);
        c=waitKey(1);
        if (c=='q')
            break;
    }
    cap2.closeStream();  
    cout<<"\n  Current Parameter  \n"<<endl;
    cap2.getCurrentSetting();
}