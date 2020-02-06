#include <iostream>
#include <opencv2/opencv.hpp>
#include "armor_plate.h"

using namespace cv;
using namespace std;

#define ARMOR_DEVICE 0

bool CamerRead(ArmorPlate& armor_param);

int main(int argc, const char * argv[])
{
    ArmorPlate armor;
    
    VideoCapture capture("/Users/zengjiawei/Desktop/Opencv_/蓝步兵.mov"); //加载本地素材
    
    armor.capture_armor_= capture;
    
    
    //armor.CamaraInit(ARMOR_DEVICE);//摄像头初始化
    while(1)
    {
        if(!CamerRead(armor))
        {
            continue;
        }
        armor.AutoShoot();
    }
    return 0;
}

bool CamerRead(ArmorPlate& armor_param)
{
    armor_param.capture_armor_.read(armor_param.armor_image_);
    if(!armor_param.armor_image_.data)
    {
        cout<<"摄像头没有读取到图像！"<<endl;
        armor_param.CamaraInit(ARMOR_DEVICE);//摄像头初始化  
        return false;
    }
    else
        return true;
}


