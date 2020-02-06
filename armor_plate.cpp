#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "armor_plate.h"

using namespace std;
using namespace cv;

ArmorPlate::ArmorPlate()
{
    our_team_= TEAMRED;
}

bool ArmorPlate::CamaraInit(int device)
{
    capture_armor_.open(device);
    if(!capture_armor_.isOpened())
    {
        cout<<"摄像头打开失败"<<endl;
        return false;
    }
    else
        return true;
}
void ArmorPlate::ImgPreprocess(const cv::Mat& src,cv::Mat& dst)
{
    std::vector<Mat> img_channels;//通道
    split(src, img_channels);
    if(our_team_) //我方蓝方，敌方红方
    {
        Mat img_red_channels;
        img_red_channels=img_channels.at(2);
        //imshow("img_red", img_red_channels);
        
        img_red_channels=img_red_channels-img_channels.at(0)*0.4-img_channels.at(1)*0.4; //去除白色影响
        //blur(img_blue_channels,img_blue_channels,Size(3,3));
        img_red_channels=img_red_channels*1.3;
        //imshow("img_red_pro", img_red_channels);
        
        double maxValue_gary;
        minMaxLoc(img_red_channels, 0, &maxValue_gary,0,0);//获取最大灰度值
        Mat imgBin;
        threshold(img_red_channels, imgBin, maxValue_gary*0.7, maxValue_gary, THRESH_BINARY);
        dst=imgBin;
        imshow("bin_red_pro", dst);
        
//        Mat element=getStructuringElement(MORPH_RECT, Size(3,3));
//        dilate(imgBin, dst, element,Point(-1,-1),1);//膨胀三次
//        erode(dst, dst, element);
//        imshow("dst", dst);
    }
    else //我方红方，敌方蓝方
    {
        Mat img_blue_channels;
        img_blue_channels=img_channels.at(0);
        img_blue_channels=img_blue_channels-img_channels.at(1)*0.4-img_channels.at(2)*0.4; //去除白色影响
        img_blue_channels=img_blue_channels*1.3;

        double maxValue_gary;
        minMaxLoc(img_blue_channels, 0, &maxValue_gary,0,0);//获取最大灰度值
        Mat imgBin;
        threshold(img_blue_channels, imgBin, maxValue_gary*0.7, maxValue_gary, THRESH_BINARY);
        //imshow("threshold", imgBin);
        //blur(imgBin,imgBin,Size(5,5));
        
        Mat element=getStructuringElement(MORPH_RECT, Size(3,3));
        erode(imgBin, dst, element,Point(-1,-1),1);
        //dilate(dst, dst, element,Point(-1,-1),1);//膨胀三次
        
        //blur(dst,dst,Size(1,1));
        //imshow("dst", dst);
    }
    
    
    
    
    
}

void ArmorPlate::FindArmor(Mat& src,Mat& dst,vector<RotatedRect>& all,RotatedRect& target)//定位装甲板
{
    all.clear();
    target.center.x=0;
    target.center.y=0;
    target.size.width=0;
    target.size.height=0;
    target.angle=0;
    
    RotatedRect s,s_fitEllipse,s_minAreaRect; //用于筛选轮廓
    vector<RotatedRect> ss;
    ss.clear();
    
    vector<vector<Point>> contours;//轮廓
    vector<Vec4i> hierarchy;//层次
    
    findContours(dst, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);//寻找轮廓
    Mat drawing=Mat::zeros(dst.size(), CV_8UC3);
    RNG g_rng(12345);
    for(int i=0;i<contours.size();i++)
    {
        Scalar color = Scalar(g_rng.uniform(0, 255),g_rng.uniform(0, 255),g_rng.uniform(0, 255));
        drawContours(drawing, contours, i, color,2,8,hierarchy,0,Point()); //画出所有轮廓、
        //imshow("lunkuo", drawing);
        if(contours[i].size()>=10)
        {
            s_fitEllipse=fitEllipse(contours[i]);//椭圆拟合
            s_minAreaRect=minAreaRect(contours[i]);
            
            s.angle=s_fitEllipse.angle;
            s.center=s_fitEllipse.center;
            if(s_minAreaRect.size.width > s_minAreaRect.size.height)
            {
                s.size.height=s_minAreaRect.size.width;
                s.size.width=s_minAreaRect.size.height;
            }
            else
            {
                s.size.height=s_minAreaRect.size.height;
                s.size.width=s_minAreaRect.size.width;
            }
            if((s.size.width / s.size.height) > L_WH_RAT)
                continue;
            int x=s.center.x-s.size.width;
            if(x<0)
                continue;
            int y=s.center.y-s.size.height;
            if(y<0)
                continue;
            int w=s.size.width+s.size.width;
            if(w>dst.cols-x)
                continue;
            int h=s.size.height+s.size.height;
            if(h>dst.rows-y)
                continue;
            
            if((s.angle<45 || s.angle>135) && (s.size.height>10) && (s.size.height<150))
                ss.push_back(s);
        }
    }
    
    // 判别装甲板
    vector<RotatedRect> armors;
    vector<ArmorBuiled> armor_SECOND;
    ArmorBuiled armor_FIRST;
    static float armor_center_x;
    static float armor_center_y;
    
    armors.clear();
    armor_SECOND.clear();
    int nL,nW;
    
    if(ss.size()<2) //旋转矩形小于两个，直接返回
    {
        target.center.x=0;
        target.center.y=0;
        target.size.width=0;
        target.size.height=0;
        target.angle=0;
        all.push_back(target);
        armor_center_x=0;
        armor_center_y=0;
    }
    else
    {
        for(int i=0;i<ss.size()-1;i++)
        {
            for(int j=i+1;j<ss.size();j++)
            {
                double height_diff = abs(ss[i].size.height - ss[j].size.height); //高度差
                double height_sum = ss[i].size.height + ss[j].size.height; //高度和
                double width_diff = abs(ss[i].size.width - ss[j].size.width); //宽度差
                double width_sum = ss[i].size.width + ss[j].size.width; //宽度和
                double angle_diff = fabs(ss[i].angle - ss[i].angle); //角度差
                double Y_diff =abs(ss[i].center.y - ss[j].center.y); //纵坐标差值
                double X_diff =abs(ss[i].center.x - ss[j].center.x); //横坐标差值
                double MH_diff = (min(ss[i].size.height,ss[j].size.height)) *2/3; //高度差限幅
                double height_max = max(ss[i].size.height,ss[j].size.height); //最大高度
                
                
                if(Y_diff<MH_diff && X_diff<height_max*4 && (angle_diff< T_ANGLE_THRE || 180-angle_diff<T_ANGLE_THRE180) &&
                   height_diff/height_sum<T_HIGH_RAT && width_diff/width_sum<T_WHIDTH_RAT)
                {
                    armor_FIRST.armorS.center.x = ((ss[i].center.x + ss[j].center.x)/2);
                    armor_FIRST.armorS.center.y = ((ss[i].center.y + ss[j].center.y)/2);
                    armor_FIRST.armorS.angle = ((ss[i].angle + ss[j].angle)/2);
                    if(180-angle_diff<T_ANGLE_THRE180)
                        armor_FIRST.armorS.angle += 90;
                    nL = (ss[i].size.height + ss[j].size.height)/2;
                    nW = sqrt((ss[i].center.x- ss[j].center.x)*(ss[i].center.x-ss[j].center.x)+(ss[i].center.y- ss[j].center.y)*(ss[i].center.y-ss[j].center.y));
                    if(nL<nW)
                    {
                        armor_FIRST.armorS.size.height=nL;
                        armor_FIRST.armorS.size.width=nW;
                    }
                    else
                    {
                        armor_FIRST.armorS.size.height=nW;
                        armor_FIRST.armorS.size.width=nL;
                    }
                    if(Y_diff<nW/3)
                    {
                        armor_FIRST.build1_No=i;
                        armor_FIRST.build2_No=j;
                        armor_FIRST.build_features[0]=angle_diff;
                        armor_FIRST.build_features[1]=Y_diff;
                        armor_FIRST.build_features[2]=height_diff;
                        armor_FIRST.build_features[3]=width_diff;
                        armor_SECOND.push_back(armor_FIRST);
                    }
                }
            }
        }
        if(armor_SECOND.size()<1)
        {
            cout<<"无装甲"<<endl;
        }
        else if(armor_SECOND.size() == 1)
        {
            cout<<"一个装甲"<<endl;
            target = armor_SECOND[0].armorS;
            all.push_back(armor_SECOND[0].armorS);
            armor_center_x=target.center.x;
            armor_center_y=target.center.y;
        }
        else
        {
            cout<<"多个装甲"<<endl;
            double min_feature=9999999;
            for(int armor_i=0;armor_i<armor_SECOND.size();armor_i++)//对各个灯带进行遍历
            {
                armors.push_back(armor_SECOND[armor_i].armorS);
                //计算加权特征值
                double feature = armor_SECOND[armor_i].build_features[0]*100+armor_SECOND[armor_i].build_features[1]*10+armor_SECOND[armor_i].build_features[2]*100+abs(armor_SECOND[armor_i].armorS.center.x-armor_center_x)*50+abs(armor_SECOND[armor_i].armorS.center.y-armor_center_y)*50-armor_SECOND[armor_i].armorS.size.height*100-armor_SECOND[armor_i].armorS.size.width*100;
                if(feature<min_feature) //寻找最小特征值
                {
                    min_feature=feature;
                    target=armor_SECOND[armor_i].armorS;
                }
            }
            //存储上一次装甲中心点
            armor_center_x=target.center.x;
            armor_center_y=target.center.y;
            all=armors;
        }
    }
}




void DrawAll(vector<RotatedRect> rect,Mat img)
{
    for(int i=0;i<rect.size();i++)
    {
        Point2f pp[4];
        rect[i].points(pp); //计算二维盒子顶点
        line(img, pp[0], pp[1], CV_RGB(255,255,0),5,8,0);
        line(img, pp[1], pp[2], CV_RGB(255,255,0),5,8,0);
        line(img, pp[2], pp[3], CV_RGB(255,255,0),5,8,0);
        line(img, pp[3], pp[0], CV_RGB(255,255,0),5,8,0);
    }
}

void DrawTarget(RotatedRect box,Mat img)
{
    Point2f pts[8];
    pts[0].x=box.center.x;
    pts[0].y=box.center.y-10;
    pts[1].x=box.center.x;
    pts[1].y=box.center.y+10;
    pts[2].x=box.center.x-10;
    pts[2].y=box.center.y;
    pts[3].x=box.center.x+10;
    pts[3].y=box.center.y;
    
    pts[4].x=img_center_x;
    pts[4].y=img_center_y-10;
    pts[5].x=img_center_x;
    pts[5].y=img_center_y+10;
    pts[6].x=img_center_x-10;
    pts[6].y=img_center_y;
    pts[7].x=img_center_x+10;
    pts[7].y=img_center_y;
    line(img, pts[0], pts[1], CV_RGB(0,255, 0),4,8,0);
    line(img, pts[2], pts[3], CV_RGB(0,255, 0),4,8,0);
    line(img, pts[4], pts[5], CV_RGB(255,255,255),4,8,0);
    line(img, pts[6], pts[7], CV_RGB(255,255,255),4,8,0);
}


void ArmorPlate::AutoShoot()
{
    //armor_image_=imread("/Users/zengjiawei/Downloads/装甲板2.jpg");
    
    ImgPreprocess(armor_image_, pre_image_);
    FindArmor(armor_image_,pre_image_,all_target_,target_);
    DrawAll(all_target_, armor_image_);
    DrawTarget(target_, armor_image_);
    
    
    imshow("yuchuli", armor_image_);
    waitKey(1);
}
