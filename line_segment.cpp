//
//  line_segment.cpp
//  opencv
//
//  Created by incer on 15/5/7.
//  Copyright (c) 2015年 ce. All rights reserved.
//

#include "line_segment.h"

#include "line_descriptor.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <android/log.h>
#include "LSWMS.h"


//-----------------------------------【命名空间声明部分】---------------------------------------
//	     描述：包含程序所使用的命名空间
//-----------------------------------------------------------------------------------------------
using namespace cv;
using namespace std;
using namespace cv::line_descriptor;
//-----------------------------------【main( )函数】--------------------------------------------
//	     描述：控制台应用程序的入口函数，我们的程序从这里开始
//-----------------------------------------------------------------------------------------------
struct LINE
{
    double a;
    double b;
    double c;
    Point start;
    Point end;
};

double ls::line_size(cv::Point &p1,cv::Point &p2)
{
    return sqrt(pow(p1.x - p2.x,2)+pow(p1.y - p2.y,2));
}

float ls::line_jiao(cv::Point &p1,cv::Point &p2)
{
    if(p2.y == p1.y)
        return 0;
    else
    {
        cv::Point min = p2.y<p1.y?p2:p1;
        cv::Point max = p2.y>p1.y?p2:p1;
        return acos((min.x - max.x)/line_size(p1, p2))*(180.0/CV_PI);
    }
}


float ls::ca(LSEG &line, int k)
{
    //float kdx = (float)line[0].x - (float)line[1].x;
    //float kdy = (float)line[0].y - (float)line[1].y;
    cv::Point zp2;
    zp2.x = (line[0].x + line[1].x)/2.0;
    zp2.y = (line[0].y + line[1].y)/2.0;
    double kk = line_jiao(line[0], line[1]);
    if(k == -1)
        return line_jiao(line[0],line[1]);
    else if(kk <= 90)
    {
        return fabs(sin(kk/180.0*CV_PI)*zp2.x+cos(kk/180.0*CV_PI)*zp2.y);
    }
    else
        return fabs(sin(kk/180.0*CV_PI)*zp2.x+cos(kk/180.0*CV_PI)*zp2.y-cos(kk/180.0*CV_PI)*krows);
//    if(k==0)
//    {
//        return (line[0].x+line[1].x)*0.5;
//    }
//    else if(k==1)
//    {
//        return (line[0].y+line[1].y)*0.5;
//    }
//    else if(k==2)
//    {
//        return abs(-(kdx/line_size(line[0],line[1]))*krows*0.7 + (kdx/line_size(line[0],line[1]))*zp2.y - (kdy/line_size(line[0],line[1]))*zp2.x);
//    }
//    else if(k==3)
//    {
//        return abs(-(kdx/line_size(line[0],line[1]))*krows*0.7 + (kdx/line_size(line[0],line[1]))*zp2.y - (kdy/line_size(line[0],line[1]))*zp2.x);
//    }
//    else
//    {
//        return line_jiao(line[0],line[1]);
//    }
}

void ls::run(vector<LSEG> &lines1,int left,int right,int k)
{
    float i,j;
    float middle;
    LSEG iTemp;
    i = left;
    j = right;
    middle = ca(lines1[(left+right)/2],k); //求中间值
    do{
        while((ca(lines1[i],k)<middle) && (i<right))//从左扫描大于中值的数
            i++;
        while((ca(lines1[j],k)>middle) && (j>left))//从右扫描大于中值的数
            j--;
        if(i<=j)//找到了一对值
        {
            //交换
            iTemp = lines1[i];
            lines1[i] = lines1[j];
            lines1[j] = iTemp;
            i++;
            j--;
        }
    }while(i<=j);//如果两边扫描的下标交错，就停止（完成一次）
    //当左边部分有值(left<j)，递归左半边
    if(left<j)
        run(lines1,left,j,k);
    //当右边部分有值(right>i)，递归右半边
    if(right>i)
        run(lines1,i,right,k);
}

void ls::QuickSort(vector<LSEG> &lines1,int Count,int k)
{
    if(Count!=0)
    run(lines1, 0,Count-1,k);
}


double ls::power(Mat &src,Mat &angle,cv::Point &a,cv::Point &b)
{
    double dx = (b.x-a.x)/sqrt(pow(b.x-a.x,2)+pow(b.y-a.y, 2));
    double dy = (b.y-a.y)/sqrt(pow(b.x-a.x,2)+pow(b.y-a.y, 2));
    double sum = 0;
    unsigned int n = 0;
    for(int i=0;i<(int)sqrt(pow(b.x-a.x, 2)+pow(b.y-a.y, 2));i+=1)
    {
        int y = a.y+i*dy;
        int x = a.x+i*dx;
        bool location_p = x<angle.cols&&x>0&&y<angle.rows&&y>0;
        if(location_p)
        {
            double a = abs(dx*cos(angle.at<float>(y,x))+dy*sin(angle.at<float>(y,x)));
            if(a<=1&&a>=-1)
            {
                double e = acos(a)*(180.0/CV_PI);
                sum +=e;
                n++;
            }
        }
        else
        {
            continue;
        }
    }
    return sum/(double)n;
}


cv::Point ls::prpoint(cv::Point &center,int d,cv::Point2f &v)
{

    cv::Point pt;
    pt.x = center.x-v.x*d;
    pt.y = center.y-v.y*d;
    return pt;
}

cv::Point ls::prpoint1(cv::Point &center,int d,cv::Point2f &v,Point &p3)
{
    cv::Point pt;
    pt.x = center.x-v.x*d;
    pt.y = center.y-v.y*d;
    float dd = abs(v.x*pt.x + v.y*pt.y - v.y*p3.y - v.x*p3.x);
    if(dd<2)
        return pt;
    else
    {
        pt.x = center.x+v.x*d;
        pt.y = center.y+v.y*d;
        return pt;
    }
}

int ls::point_line(cv::Point &p1,cv::Point &p2,cv::Point &tp)
{
    cv::Point ap;
    ap.x = -1;
    float kdx = abs((float)p1.x - (float)p2.x);
    float kdy = abs((float)p1.y - (float)p2.y);
    if(kdy < 20)
        ap.y = p1.y;
    else if(kdx < 20)
    {
        ap.x = p1.x;
        ap.y = -1;
    }
    else
        ap.y = p1.y - (kdy/kdx)*(-1-p1.x);

    float min_size = line_size(ap, p1)<line_size(ap, p2)?line_size(ap, p1):line_size(ap, p2);
    float max_size = line_size(ap, p1)>line_size(ap, p2)?line_size(ap, p1):line_size(ap, p2);
    if(line_size(ap, tp) > max_size)
        return  2;
    else if(line_size(ap, tp)<min_size)
        return  1;
    else
        return  0;
}

int ls::warf(Mat &src,Mat &src1,Mat &angle,LSEG &line1,LSEG &line2,vector<Point> &rc,int k,float &tg)
{
    cv::Point2f v1,fv1,vt1,fvt1,v2,fv2,vt2,fvt2;
    cv::Point p1,p2,cp,tp1,tp2,p3,p4,tp3,tp4;
    cv::Point fp1,lp1;
    cv::Point fp2,lp2;
    fp1 = line1[0];
    lp1 = line1[1];
    fp2 = line2[0];
    lp2 = line2[1];
    v1.x = ((float)lp1.x - (float)fp1.x)/line_size(fp1,lp1);
    v1.y = ((float)lp1.y - (float)fp1.y)/line_size(fp1,lp1);

    vt1.x = -v1.y;
    vt1.y = v1.x;
    fv1.x = -v1.x;
    fv1.y = -v1.y;
    fvt1.x = -vt1.x;
    fvt1.y = -vt1.y;
    v2.x = ((float)lp2.x - (float)fp2.x)/line_size(fp2,lp2);
    v2.y = ((float)lp2.y - (float)fp2.y)/line_size(fp2,lp2);
    vt2.x = -v2.y;
    vt2.y = v2.x;
    fv2.x = -v2.x;
    fv2.y = -v2.y;
    fvt2.x = -vt2.x;
    fvt2.y = -vt2.y;
    p1 = fp1;
    p2 = lp1;
    p3 = fp2;
    p4 = lp2;
    float dd1 = abs(v2.y*p1.x - v2.x*p1.y + v2.x*p3.y -v2.y*p3.x);
    float dd2 = abs(v2.y*p2.x - v2.x*p2.y + v2.x*p3.y -v2.y*p3.x);
    float dd3 = abs(v1.y*p3.x - v1.x*p3.y + v1.x*p1.y -v1.y*p1.x);
    float dd4 = abs(v1.y*p4.x - v1.x*p4.y + v1.x*p1.y -v1.y*p1.x);
    tp1 = prpoint1(p1, dd1, vt2,p3);
    tp2 = prpoint1(p2, dd2, vt2,p3);
    tp3 = prpoint1(p3, dd3, fvt1,p1);
    tp4 = prpoint1(p4, dd4, fvt1,p1);
    int a = point_line(p3, p4, tp1);
    int b = point_line(p3, p4, tp2);
    int c = point_line(p1, p2, tp3);
    int d = point_line(p1, p2, tp4);
    if(a!=0&&b!=0&&a == b)
    {
        return 0;
    }
    if(acos(v1.x*v2.x+v1.y*v2.y)*(180.0/CV_PI)>2)
    {
        return 3;
    }
    cv::Point P1,P2,P3,P4;

    P1 = a?tp3:p1;
    P2 = b?tp4:p2;
    P3 = a?p3:tp1;
    P4 = b?p4:tp2;
    cv::Point p11,p12,p21,p22;
    int d1 = 0,d2 = 0;
    double e_min1=0,e_min2=0;
    double active_d1 = true,active_d2 = true,location_p3 = true,location_p4 = true;
    bool location_p1 = true,location_p2 = true;
    float min_x = 0,max_x = 0,min_y = 0,max_y = 0;
    while((active_d1&&location_p1&&location_p3)||(active_d2&&location_p2&&location_p4))
    {
        cv::Point op1,op2,op3,op4;
        p11 = prpoint(P1, d1, v1);
        p12 = prpoint(P2, d2, fv1);
        p21 = prpoint(P3, d1, v2);
        p22 = prpoint(P4, d2, fv2);
        min_x = line_size(p11,p21)<line_size(p12,p22)?line_size(p11,p21):line_size(p12,p22);
        max_x = line_size(p11,p21)>line_size(p12,p22)?line_size(p11,p21):line_size(p12,p22);
        min_y = line_size(p11,p12)<line_size(p21,p22)?line_size(p11,p12):line_size(p21,p22);

        if(min_x < 10||max_x > 150||max_y > 700)
        {
            return 2;
        }
        location_p1 = p11.x<angle.cols&&p11.x>0&&p11.y<angle.rows&&p11.y>0;
        location_p2 = p12.x<angle.cols&&p12.x>0&&p12.y<angle.rows&&p12.y>0;
        location_p3 = p21.x<angle.cols&&p21.x>0&&p21.y<angle.rows&&p21.y>0;
        location_p4 = p22.x<angle.cols&&p22.x>0&&p22.y<angle.rows&&p22.y>0;

        float power1 = power(src, angle, p11, p21);
        float power2 = power(src, angle, p12, p22);

        cv::Point fp1 = prpoint(p11,5,v1);
        cv::Point lp1 = prpoint(p21,5,v1);
        float zpower1_min = power(src, angle, p11, fp1) < power(src, angle, p21, lp1) ? power(src, angle, p11, fp1)
        : power(src, angle, p21, lp1);//第三个条件：取出能量最大值，小于一个阈值表示d不在增长
        float zpower1_max = power(src, angle, p11, fp1) > power(src, angle, p21, lp1) ? power(src, angle, p11, fp1)
        : power(src, angle, p21, lp1);
        float mean_power1 = 0.5 * (zpower1_min + zpower1_max);

        float zpower1 = zpower1_max;

        Point fp2 = prpoint(p12,5,fv1);
        Point lp2 = prpoint(p22,5,fv2);

        float zpower2_min = power(src, angle, p12, fp2) < power(src, angle, p22, lp2) ? power(src, angle, p12, fp2) : power(src, angle, p22, lp2);
        float zpower2_max = power(src, angle, p12, fp2) > power(src, angle, p22, lp2) ? power(src, angle, p12, fp2) : power(src, angle, p22, lp2);
        float mean_power2 = 0.5 * (zpower2_min + zpower2_max);

        float zpower2 = zpower2_max;

        float tg1 = 0.60*mean_power1 +  0.40*mean_power2;//0.65  0.35

        if(active_d1 && location_p1 && location_p3)//上端或左端
        {
            if((power1 > tg1  && zpower1 < 1.2*tg1))//|| abs(power1 - power2) < 10)//)//满足第二个条件和第三个条件，不再生长30,42
            {
                active_d1 = false;
                e_min1 = abs(power1);
            }
            else//否则继续生长
            {
                d1 += 1;//每次增加1个像素

            }
        }
        else
        {
            //            if(power1>60)
            //            {
            //                active_d1 = true;
            //            }
        }

        //        cout<<"2:"<<power2<<"   "<<zpower2<<endl;
        float tg2 = 0.40*mean_power1 +  0.60*mean_power2;
        if(active_d2 && location_p2 && location_p4)
        {
            if((power2 > tg2 && zpower2 < 1.2*tg2))//|| abs(power1 - power2) < 10)//右端和下端
            {
                active_d2 = false;
                e_min2 = abs(power2);
            }
            else
            {
                d2 += 1;
            }
        }
        tg = (tg1+tg2)*0.5;
    }
//    Mat img1 = src.clone();
//    circle(img1, p11, 10, Scalar(0),3,8);
//    circle(img1, p12, 10, Scalar(0),3,8);
//    circle(img1, p21, 10, Scalar(0),3,8);
//    circle(img1, p22, 10, Scalar(0),3,8);
//    imshow("img1", img1);
//    waitKey(0);
    rc.push_back(p11);
    rc.push_back(p12);
    rc.push_back(p21);
    rc.push_back(p22);
    return 1;
}

void ls::warp_change(vector<cv::Point>& in_prs,Mat& src1,Mat& src,Mat& out,vector<cv::Point2f> &corner)
{
    vector<Point> outpoints;
    vector<cv::Point> ptr_tmp;
    float jiao_01 = line_jiao(in_prs[0], in_prs[1]);
    float jiao_23 = line_jiao(in_prs[2], in_prs[3]);
    if (jiao_01 > 160 || jiao_01 < 20)
    {
        if ((in_prs[0].x < in_prs[1].x && in_prs[2].x > in_prs[3].x))// || (in_prs[0].y < in_prs[1].y && in_prs[2].y > in_prs[3].y))
        {
            ptr_tmp.push_back(in_prs[0]);
            ptr_tmp.push_back(in_prs[1]);
            ptr_tmp.push_back(in_prs[3]);
            ptr_tmp.push_back(in_prs[2]);
            in_prs.clear();
            in_prs.push_back(ptr_tmp[0]);
            in_prs.push_back(ptr_tmp[1]);
            in_prs.push_back(ptr_tmp[2]);
            in_prs.push_back(ptr_tmp[3]);
            
        }
        else if ((in_prs[0].x > in_prs[1].x && in_prs[2].x < in_prs[3].x))// || (in_prs[0].y > in_prs[1].y && in_prs[2].y < in_prs[3].y))
        {
            ptr_tmp.push_back(in_prs[1]);
            ptr_tmp.push_back(in_prs[0]);
            ptr_tmp.push_back(in_prs[2]);
            ptr_tmp.push_back(in_prs[3]);
            in_prs.clear();
            in_prs.push_back(ptr_tmp[0]);
            in_prs.push_back(ptr_tmp[1]);
            in_prs.push_back(ptr_tmp[2]);
            in_prs.push_back(ptr_tmp[3]);
        }
        else if ((in_prs[0].x > in_prs[1].x && in_prs[2].x > in_prs[3].x))// || (in_prs[0].y > in_prs[1].y && in_prs[2].y > in_prs[3].y))
        {
            ptr_tmp.push_back(in_prs[1]);
            ptr_tmp.push_back(in_prs[0]);
            ptr_tmp.push_back(in_prs[3]);
            ptr_tmp.push_back(in_prs[2]);
            in_prs.clear();
            in_prs.push_back(ptr_tmp[0]);
            in_prs.push_back(ptr_tmp[1]);
            in_prs.push_back(ptr_tmp[2]);
            in_prs.push_back(ptr_tmp[3]);
        }
        
    }
    else
    {
        ptr_tmp.push_back(in_prs[0]);
        ptr_tmp.push_back(in_prs[1]);
        ptr_tmp.push_back(in_prs[2]);
        ptr_tmp.push_back(in_prs[3]);
        in_prs.clear();
        in_prs.push_back(ptr_tmp[0]);
        in_prs.push_back(ptr_tmp[1]);
        in_prs.push_back(ptr_tmp[2]);
        in_prs.push_back(ptr_tmp[3]);
    }
    
    
    bool lx = line_size(in_prs[0],in_prs[1])<line_size(in_prs[2],in_prs[3]);
    bool ly = line_size(in_prs[0],in_prs[2])<line_size(in_prs[1],in_prs[3]);
    
    int x_size = lx?line_size(in_prs[2],in_prs[3]):line_size(in_prs[0],in_prs[1]);
    int y_size = ly?line_size(in_prs[1],in_prs[3]):line_size(in_prs[0],in_prs[2]);
    cv::Point mid_01,mid_23;
    Point2f v,vt,fv,fvt;
    mid_01.x = (in_prs[0].x + in_prs[1].x) / 2;
    mid_01.y = (in_prs[0].y + in_prs[1].y) / 2;
    mid_23.x = (in_prs[2].x + in_prs[3].x) / 2;
    mid_23.y = (in_prs[2].y + in_prs[3].y) / 2;
    cout<<mid_01.x<<","<<mid_23.x<<endl;
    cout<<mid_01.y<<","<<mid_23.y<<endl;
    if ((jiao_01 > 160 || jiao_01 < 20) && mid_01.y < mid_23.y)//(mid_01.x > mid_23.x || mid_01.y < mid_23.y)
    {
        if(!lx&&!ly)
        {
            v.x = (in_prs[2].x - in_prs[3].x)/line_size(in_prs[2],in_prs[3]);
            v.y = (in_prs[2].y - in_prs[3].y)/line_size(in_prs[2],in_prs[3]);
            vt.x = (in_prs[2].x - in_prs[0].x)/line_size(in_prs[0],in_prs[2]);
            vt.y = (in_prs[2].y - in_prs[0].y)/line_size(in_prs[0],in_prs[2]);
            outpoints.push_back(in_prs[2]);
            outpoints.push_back(prpoint(in_prs[2], x_size, v));
            outpoints.push_back(prpoint(in_prs[2], y_size, vt));
            Point a = prpoint(in_prs[2], x_size, v);
            outpoints.push_back(prpoint(a, y_size, vt));
            
            
            
        }
        else if(!lx&&ly)
        {
            v.x = (in_prs[3].x - in_prs[2].x)/line_size(in_prs[2],in_prs[3]);
            v.y = (in_prs[3].y - in_prs[2].y)/line_size(in_prs[2],in_prs[3]);
            vt.x = (in_prs[3].x - in_prs[1].x)/line_size(in_prs[1],in_prs[3]);
            vt.y = (in_prs[3].y - in_prs[1].y)/line_size(in_prs[1],in_prs[3]);
            outpoints.push_back(prpoint(in_prs[3], x_size, v));
            outpoints.push_back(in_prs[3]);
            Point a = prpoint(in_prs[3], x_size, v);
            outpoints.push_back(prpoint(a, y_size, vt));
            outpoints.push_back(prpoint(in_prs[3], y_size, vt));
            
            
            
        }
        else if(lx&&!ly)
        {
            v.x = (in_prs[0].x - in_prs[1].x)/line_size(in_prs[0],in_prs[1]);
            v.y = (in_prs[0].y - in_prs[1].y)/line_size(in_prs[0],in_prs[1]);
            vt.x = (in_prs[0].x - in_prs[2].x)/line_size(in_prs[0],in_prs[2]);
            vt.y = (in_prs[0].y - in_prs[2].y)/line_size(in_prs[0],in_prs[2]);
            outpoints.push_back(prpoint(in_prs[0], y_size, vt));
            Point a = prpoint(in_prs[0], y_size, vt);
            outpoints.push_back(prpoint(a, x_size, v));
            outpoints.push_back(in_prs[0]);
            outpoints.push_back(prpoint(in_prs[0], x_size, v));
            
            
            
        }
        else
        {
            v.x = (in_prs[1].x - in_prs[0].x)/line_size(in_prs[0],in_prs[1]);
            v.y = (in_prs[1].y - in_prs[0].y)/line_size(in_prs[0],in_prs[1]);
            vt.x = (in_prs[1].x - in_prs[3].x)/line_size(in_prs[1],in_prs[3]);
            vt.y = (in_prs[1].y - in_prs[3].y)/line_size(in_prs[1],in_prs[3]);
            Point a = prpoint(in_prs[1], x_size, v);
            outpoints.push_back(prpoint(a, y_size, vt));
            outpoints.push_back(prpoint(in_prs[1], y_size, vt));
            outpoints.push_back(prpoint(in_prs[1], x_size, v));
            outpoints.push_back(in_prs[1]);
            
            
        }
        
    }
    else
    {
        
        if(lx&&ly)//01 < 23 && 02 < 13
        {
            
            v.x = (in_prs[0].x - in_prs[1].x)/line_size(in_prs[0],in_prs[1]);
            v.y = (in_prs[0].y - in_prs[1].y)/line_size(in_prs[0],in_prs[1]);
            vt.x = (in_prs[0].x - in_prs[2].x)/line_size(in_prs[0],in_prs[2]);
            vt.y = (in_prs[0].y - in_prs[2].y)/line_size(in_prs[0],in_prs[2]);
            outpoints.push_back(in_prs[0]);
            outpoints.push_back(prpoint(in_prs[0], x_size, v));
            outpoints.push_back(prpoint(in_prs[0], y_size, vt));
            Point a = prpoint(in_prs[0], x_size, v);
            outpoints.push_back(prpoint(a, y_size, vt));
        }
        else if(lx&&!ly)
        {
            v.x = (in_prs[1].x - in_prs[0].x)/line_size(in_prs[0],in_prs[1]);
            v.y = (in_prs[1].y - in_prs[0].y)/line_size(in_prs[0],in_prs[1]);
            vt.x = (in_prs[1].x - in_prs[3].x)/line_size(in_prs[1],in_prs[3]);
            vt.y = (in_prs[1].y - in_prs[3].y)/line_size(in_prs[1],in_prs[3]);
            outpoints.push_back(prpoint(in_prs[1], x_size, v));
            outpoints.push_back(in_prs[1]);
            Point a = prpoint(in_prs[1], x_size, v);
            outpoints.push_back(prpoint(a, y_size, vt));
            outpoints.push_back(prpoint(in_prs[1], y_size, vt));
        }
        else if(!lx&&ly)
        {
            v.x = (in_prs[2].x - in_prs[3].x)/line_size(in_prs[2],in_prs[3]);
            v.y = (in_prs[2].y - in_prs[3].y)/line_size(in_prs[2],in_prs[3]);
            vt.x = (in_prs[2].x - in_prs[0].x)/line_size(in_prs[0],in_prs[2]);
            vt.y = (in_prs[2].y - in_prs[0].y)/line_size(in_prs[0],in_prs[2]);
            outpoints.push_back(prpoint(in_prs[2], y_size, vt));
            Point a = prpoint(in_prs[2], y_size, vt);
            outpoints.push_back(prpoint(a, x_size, v));
            outpoints.push_back(in_prs[2]);
            outpoints.push_back(prpoint(in_prs[2], x_size, v));
        }
        else
        {
            v.x = (in_prs[3].x - in_prs[2].x)/line_size(in_prs[2],in_prs[3]);
            v.y = (in_prs[3].y - in_prs[2].y)/line_size(in_prs[2],in_prs[3]);
            vt.x = (in_prs[3].x - in_prs[1].x)/line_size(in_prs[1],in_prs[3]);
            vt.y = (in_prs[3].y - in_prs[1].y)/line_size(in_prs[1],in_prs[3]);
            Point a = prpoint(in_prs[3], x_size, v);
            outpoints.push_back(prpoint(a, y_size, vt));
            outpoints.push_back(prpoint(in_prs[3], y_size, vt));
            outpoints.push_back(prpoint(in_prs[3], x_size, v));
            outpoints.push_back(in_prs[3]);
        }
    }
    
    out = Mat::zeros(x_size*((float)src1.cols/(float)src.cols), y_size*((float)src1.cols/(float)src.cols), CV_8UC3);
    
    vector<cv::Point2f> quad_pts;
    
    corner.push_back(cv::Point(outpoints[0].x*((float)src1.cols/(float)src.cols),outpoints[0].y*((float)src1.cols/(float)src.cols)));
    corner.push_back(cv::Point(outpoints[2].x*((float)src1.cols/(float)src.cols),outpoints[2].y*((float)src1.cols/(float)src.cols)));
    corner.push_back(cv::Point(outpoints[1].x*((float)src1.cols/(float)src.cols),outpoints[1].y*((float)src1.cols/(float)src.cols)));
    corner.push_back(cv::Point(outpoints[3].x*((float)src1.cols/(float)src.cols),outpoints[3].y*((float)src1.cols/(float)src.cols)));
    
    quad_pts.push_back(cv::Point2f(0,0));
    quad_pts.push_back(cv::Point2f(line_size(outpoints[1],outpoints[3])*((float)src1.cols/(float)src.cols),0));
    quad_pts.push_back(cv::Point2f(0,line_size(outpoints[0],outpoints[1])*((float)src1.cols/(float)src.cols)));
    quad_pts.push_back(cv::Point2f(line_size(outpoints[1],outpoints[3])*((float)src1.cols/(float)src.cols),line_size(outpoints[2],outpoints[3])*((float)src1.cols/(float)src.cols)));
    
    Mat transmtx = getPerspectiveTransform(corner, quad_pts);
    
    warpPerspective(src1, out, transmtx,out.size());
}

float ls::area(Point &pt,Point &p1,Point &p2)
{
    Point2f v;
    v.x = ((float)p1.x - (float)p2.x)/line_size(p1,p2);
    v.y = ((float)p1.y - (float)p2.y)/line_size(p1,p2);
    float dd = abs(v.y*pt.x - v.x*pt.y + v.x*p1.y -v.y*p1.x);
    return 0.5*dd*line_size(p1, p2);
}

bool ls::isInRectangle(Point& pt,vector<Point>& pr)
{
    float S01,S12,S23,S30;
    S01 = area(pt, pr[0], pr[1]);
    S12 = area(pt, pr[1], pr[2]);
    S23 = area(pt, pr[2], pr[3]);
    S30 = area(pt, pr[3], pr[0]);
    float s012,s123;
    s012 = area(pr[0], pr[1], pr[2]);
    s123 = area(pr[3], pr[1], pr[2]);
    if(abs(S01+S12+S23+S30 - s012-s123)<100)
        return true;
    else
        return false;
}

///------------alg 3------------
//求两个点的直线方程
LINE makeline(Point p1,Point p2) {
    LINE tl;
    tl.start = p1;
    tl.end = p2;
    int sign = 1;
    tl.a=p2.y-p1.y;
    if (tl.a<0) {
        sign = -1;
        tl.a=sign*tl.a;
    }
    tl.b=sign*(p1.x-p2.x);
    tl.c=sign*(p1.y*p2.x-p1.x*p2.y);
    return tl;
}

// 如果两条直线 l1(a1*x+b1*y+c1 = 0), l2(a2*x+b2*y+c2 = 0)相交，返回true，且返回交点p
bool lineintersect(LINE l1,LINE l2,Point &p) {// 是 L1，L2
    double d=l1.a*l2.b-l2.a*l1.b;
    if (abs(d)<0) return false;// 不相交
    p.x = (l2.c*l1.b-l1.c*l2.b)/d;
    p.y = (l2.a*l1.c-l1.a*l2.c)/d;
    return true;
}
//计算叉积
double multiply(Point sp,Point ep,Point op) {
    return((sp.x-op.x)*(ep.y-op.y)-(ep.x-op.x)*(sp.y-op.y));
}
bool online(LINE l,Point p)
{
    return((((p.x-l.start.x)*(p.x-l.end.x)<=0)&&((p.y-l.start.y)*(p.y-l.end.y)<=0 )));
}

Point vector_g(Point a,Point b)
{
    Point c;
    c.x = b.x - a.x;
    c.y = b.y - a.y;
    return c;

}

double cross_product(Point a,Point b)
{
    double cp,d1,d2,m;
    double cosr;
    m=a.x*b.x+a.y*b.y;
    d1 = sqrt(a.x * a.x + a.y * a.y);
    d2 = sqrt(b.x * b.x + b.y * b.y);
    cosr = m/d1/d2;
    cp = d1 * d2 * sin(acos(cosr));
    return cp;
}

double area_vertice_intersection(vector<Point> points)
{
    vector<int> hull;
    if(points.size()<3)
        return 0;
    convexHull(points, hull, true);
    Point poi = points[hull[hull.size()-1]];
    vector<Point> vec;
    for(int i=0;i < hull.size();i++)
    {
        Point p_new = points[hull[i]];
        Point v;
        v = vector_g(poi, p_new);
        if(v.x!=0||v.y!=0)
        {
            vec.push_back(v);
        }
        poi = p_new;
    }
    double sum = 0.0;
    int i = 0;
    while(i <= (int)vec.size()-2)
    {
        double css;
        css = cross_product(vec[i],vec[i+1]);
        sum = sum + css;
        i=i+2;
    }
    return sum/2;
}


void ls::DecideOberlap(vector<Point> &pr1,vector<Point> &pr2,Size imgsize,int& k1,int & k2)
{
vector<cv::Point> p1,p2;
p1.push_back(pr1[0]);
p1.push_back(pr1[2]);
p1.push_back(pr1[3]);
p1.push_back(pr1[1]);
p2.push_back(pr2[0]);
p2.push_back(pr2[2]);
p2.push_back(pr2[3]);
p2.push_back(pr2[1]);
vector<Point> points;
for (int m = 0; m < 4; m++)
{
if(isInRectangle(p1[m], p2))
{
points.push_back(p1[m]);
}
if(isInRectangle(p2[m], p1))
{
points.push_back(p2[m]);
}
for(int n = 0;n < 4;n++)
{
Point pt;
LINE l1,l2;
l1 = makeline(p1[m%4], p1[(m+1)%4]);
l2 = makeline(p2[n%4], p2[(n+1)%4]);
if(lineintersect(l1,l2,pt))//互相交叉
{
if(online(l1, pt)&&online(l2, pt))
{
points.push_back(pt);
}
}
else
{//互相包含
if(online(l1, l2.start))
points.push_back(l2.start);
else if(online(l1, l2.end))
points.push_back(l2.end);
else if(online(l2, l1.start))
points.push_back(l1.start);
else if(online(l2, l1.end))
points.push_back(l1.end);
//                    if(online(l1, l2.start))
//                        points.push_back(l1.start);
//                    else if(online(l1, l2.end))
//                        points.push_back(l1.end);
//                    else if(online(l2, l1.start))
//                        points.push_back(l2.start);
//                    else if(online(l2, l1.end))
//                        points.push_back(l2.end);
}
}
}
double area = area_vertice_intersection(points);
if (area == 0) {
k1 = 0;k2 = 0;
}
else
{
double area1,area2;
area1 = area_vertice_intersection(pr1);
area2 = area_vertice_intersection(pr2);
float rate = 0.4;
double b1,b2;
b1 = area/area1;
b2 = area/area2;
if((b1 >= rate || b2 >= rate) && b1 > b2)
{
k1 = 1;k2 = 0;
}
else if((b1 >= rate || b2 >= rate) && b2 > b1)
{
k1 = 0;k2 = 1;
}
else
{k1 = 0;k2 = 0;}
}
}

float ls::pr_area(Mat &angle,vector<Point>& pr,float& height)
{
    int x_size = line_size(pr[0],pr[1])<line_size(pr[2],pr[3])?line_size(pr[2],pr[3]):line_size(pr[0],pr[1]);
    int y_size = line_size(pr[0],pr[2])<line_size(pr[1],pr[3])?line_size(pr[1],pr[3]):line_size(pr[0],pr[2]);
    height = x_size < y_size ? y_size : x_size;
    Mat src;double a,b;
    if(x_size > y_size)
    {
        a = 180 - power(src, angle, pr[0], pr[1]);
        b = 180 - power(src, angle, pr[2], pr[3]);
    }
    else
    {
        a = 180 - power(src, angle, pr[0], pr[2]);
        b = 180 - power(src, angle, pr[1], pr[2]);
    }

    return height*(1 - (a+b)/(2*30));
}
//using namespace boost;
//
//template <typename Graph>
//bool read_graph(std::istream& dimacs, Graph& g) {
//    size_t vertices = 0, edges = 0;
//    
//    std::string line;
//    while (getline(dimacs, line)) {
//        std::istringstream iss(line);
//        char ch;
//        
//        if (iss >> ch) {
//            size_t from, to;
//            std::string format;
//            
//            switch(ch) {
//                case 'c': break;
//                case 'p':
//                    if (vertices||edges) return false;
//                    if (iss >> format >> vertices >> edges) {
//                        if ("edge" != format) return false;
//                    }
//                    break;
//                case 'n':
//                    if (!vertices) return false;
//                    size_t v, weight;
//                    if (iss >> v >> weight)
//                        g[v - 1].weight = weight;
//                    break;
//                case 'e':
//                    if (edges-- && (iss >> from >> to) &&
//                        (add_edge(from-1, to-1, g).second))
//                        break;
//                default:
//                    return false;
//            }
//        }
//    }
//    
//    return !(edges || !dimacs.eof());
//}
//
//template<typename Graph>
//class MaxWeightIndependentSet
//{
//    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
//    typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
//    typedef typename boost::property_map<Graph, boost::vertex_index_t>::type VertexIndex;
//    
//    typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
//    typedef boost::adjacency_list<boost::setS,
//    boost::vecS,
//    boost::undirectedS,
//    boost::no_property,
//    EdgeWeightProperty> EdgeWeightGraph;
//    typedef typename boost::graph_traits<EdgeWeightGraph>::vertex_descriptor EWGVertex;
//    typedef typename boost::graph_traits<EdgeWeightGraph>::edge_descriptor EWGEdge;
//    
//public:
//    MaxWeightIndependentSet(const Graph& G, bool separate_odd_cycles = true)
//    : G_(G), separate_odd_cycles_(separate_odd_cycles) {
//        vertex_index_ = get(boost::vertex_index, G_);
//        build_bipartite_graph();
//        setup_lp_problem();
//    }
//    
//    void solve() {
//        run_cut_plane_method();
//    }
//    
//    void print_solution() {
//        std::cout << "Found stable set with weight: "
//        << glp_mip_obj_val(lp_) << std::endl;
//        
//        int n = glp_get_num_cols(lp_);
//        
//        std::cout << "Vertices: " << std::endl;
//        for (int i = 1; i <= n; i++)
//            if (glp_mip_col_val(lp_, i))
//                std::cout << i - 1 << ", ";
//        std::cout << std::endl;
//    }
//    
//    void print_edge_weigths() {
//        std::cout << "Edge weights:" << std::endl;
//        BGL_FORALL_EDGES_T(e, H_, EdgeWeightGraph)
//        std::cout << get(weightmap_, e) << ", ";
//        std::cout << std::endl;
//    }
//    
//    glp_prob *lp_;
//
//private:
//    void build_bipartite_graph() {
//        BGL_FORALL_VERTICES_T(v, G_, Graph) {
//            Vertex v1, v2;
//            
//            map_to_H1_[v] = add_vertex(H_);
//            map_to_H2_[v] = add_vertex(H_);
//            map_to_G_[map_to_H1_[v]] = v;
//            map_to_G_[map_to_H2_[v]] = v;
//        }
//        
//        BGL_FORALL_VERTICES_T(v, G_, Graph) {
//            typename Graph::adjacency_iterator it, end;
//            
//            boost::tie(it, end) = adjacent_vertices(v, G_);
//            for (; it != end; ++it) {
//                const Vertex& u = *it;
//                
//                add_edge(map_to_H1_[v], map_to_H2_[u], H_);
//                add_edge(map_to_H2_[v], map_to_H1_[u], H_);
//            }
//        }
//    }
//    
//    void update_bipartite_graph_weights() {
//        BGL_FORALL_EDGES_T(e, H_, EdgeWeightGraph) {
//            double weight, x_u, x_v;
//            auto u = source(e, H_);
//            auto v = target(e, H_);
//            
//            x_u = glp_get_col_prim(lp_, vertex_index_[map_to_G_[u]] + 1);
//                
//            weight = (1 - x_u - x_v) / 2;
//            if (weight < 0)
//                weight = 0;
//            
//            put(weightmap_, e, weight);
//        }
//    }
//    
//    void setup_lp_problem() {
//        lp_ = glp_create_prob();
//        std::vector<int> row_index;
//        std::vector<double> row_coefficients;
//        
//        glp_add_rows(lp_, num_edges(G_));
//        glp_add_cols(lp_, num_vertices(G_));
//        glp_set_obj_dir(lp_, GLP_MAX);
//        
//        // GLPK indexes start at 1, and it access elements 1..n of the coefficient
//        // vectors, leaving element zero unused. So we need to allocate space
//        // for n + 1 elements.
//        row_index.reserve(num_vertices(G_) + 1);
//        row_coefficients.reserve(num_vertices(G_) + 1);
//        
//        // Setup vertex variables
//        BGL_FORALL_VERTICES_T(v, G_, Graph) {
//            glp_set_obj_coef(lp_, vertex_index_[v] + 1, G_[v].weight);
//            glp_set_col_bnds(lp_, vertex_index_[v] + 1, GLP_DB, 0.0, 1.0);
//            glp_set_col_kind(lp_, vertex_index_[v] + 1, GLP_BV);
//        }
//        
//        int eidx=1;
//        BGL_FORALL_EDGES_T(e, G_, Graph) {
//            Vertex u, v;
//            
//            BGL_FORALL_VERTICES_T(v, G_, Graph) {
//                row_index[vertex_index_[v] + 1] = vertex_index_[v] + 1;
//                row_coefficients[vertex_index_[v] + 1] = 0.0;
//            }
//            
//            u = source(e, G_);
//            v = target(e, G_);
//            
//            row_coefficients[vertex_index_[u] + 1] = 1.0;
//            row_coefficients[vertex_index_[v] + 1] = 1.0;
//            
//            glp_set_mat_row(lp_, eidx, num_vertices(G_),
//                            &row_index[0], &row_coefficients[0]);
//            glp_set_row_bnds(lp_, eidx, GLP_UP, 0.0, 1.0);
//            eidx++;
//        }
//    }
//    
//    void find_initial_solution() {
//        // TODO: use a maximal stable set or clique covering
//        glp_simplex(lp_, NULL);
//    }
//    
//    void add_odd_cycle_inquality(Vertex v, std::vector<EWGVertex> p,
//                                 std::unordered_map<Vertex, bool>& covered) {
//        int new_row = glp_add_rows(lp_, 1);
//        std::vector<int> indexes(num_vertices(G_) + 1);
//        std::vector<double> coefficients(num_vertices(G_) + 1, 0.0);
//        std::list<Vertex> cycle;
//        
//        BGL_FORALL_VERTICES_T(v, G_, Graph)
//        indexes[vertex_index_[v] + 1] = vertex_index_[v] + 1;
//        
//        EWGVertex u = map_to_H2_[v];
//        while (u != map_to_H1_[v]) {
//            cycle.push_back(map_to_G_[u]);
//            covered[map_to_G_[u]] = true;
//            u = p[u];
//        }
//        
//        // Check for repeated vertices. This can happen since we are
//        // not taking only the shortest path, but any path that with
//        // the appropriate distance. In that case, it is possible that
//        // cycle is not actually a cycle.
//        for (auto it1 = cycle.begin(); it1 != cycle.end(); ++it1) {
//            auto it2 = it1;
//            for (++it2; it2 != cycle.end(); ++it2) {
//                if (*it1 == *it2) {
//                    cycle.erase(cycle.begin(), it1);
//                    cycle.erase(it2, cycle.end());
//                    break;
//                }
//            }
//        }
//        
//        for (auto u: cycle)
//            coefficients[vertex_index_[u] + 1] = 1.0;
//        
//        assert(cycle.size() % 2);
//        
//        glp_set_mat_row(lp_, new_row, num_vertices(G_),
//                        &indexes[0], &coefficients[0]);
//        glp_set_row_bnds(lp_, new_row, GLP_UP, 0.0,
//                         (double) (cycle.size() - 1) / 2);
//    }
//    
//    void separate_odd_cycles() {
//        int added_inequalities = 0;
//        std::unordered_map<Vertex, bool> covered;
//        
//        update_bipartite_graph_weights();
//        
//        BGL_FORALL_VERTICES_T(v, G_, Graph)
//        covered[v] = false;
//        
//        BGL_FORALL_VERTICES_T(v, G_, Graph) {
//            std::vector<EWGVertex> p(num_vertices(H_));
//            std::vector<double> d(num_vertices(H_));
//            
//            dijkstra_shortest_paths(H_, map_to_H1_[v],
//                                    predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, H_))).
//                                    distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, H_))));
//            
//            // If the weight of a cycle is < 0.5, then it's odd
//            // cycle inequality is violated.
//            if (!covered[v] && d[map_to_H2_[v]] + 0.000001 < 0.5) {
//                add_odd_cycle_inquality(v, p, covered);
//                added_inequalities++;
//            }
//        }
//        
////        std::cout << "Added " << added_inequalities << " inequalities." << std::endl;
//        
//        if (added_inequalities == 0)
//            separate_odd_cycles_ = false;
//    }
//    
//    void mip_callback(glp_tree *T) {
//        switch (glp_ios_reason(T)) {
//            case GLP_ICUTGEN:
//                if (separate_odd_cycles_)
//                    separate_odd_cycles();
//                break;
//            default:
//                break;
//        }
//    }
//    
//    static void static_mip_callback(glp_tree *T, void *data) {
//        MaxWeightIndependentSet<Graph> *solver =
//        static_cast<MaxWeightIndependentSet<Graph> *>(data);
//        solver->mip_callback(T);
//    }
//    
//    void run_cut_plane_method() {
//        glp_iocp parm;
//        
//        glp_init_iocp(&parm);
//        parm.cb_func = MaxWeightIndependentSet<Graph>::static_mip_callback;
//        parm.cb_info = this;
//        
//        find_initial_solution();
//        
//        glp_intopt(lp_, &parm);
//    }
//    
//    const Graph& G_;
//    VertexIndex vertex_index_;
//    
//    // A bipartite graph where each partition is a copy of the vertices
//    // of G, and two vertices are adjacent if their corresponding vertices
//    // in the original graph form an edge.
//    EdgeWeightGraph H_;
//    
//    // Map vertices from G to H
//    std::unordered_map<Vertex, EWGVertex> map_to_H1_, map_to_H2_;
//    // Map vertices from H back to G
//    std::unordered_map<EWGVertex, Vertex> map_to_G_;
//    
//    boost::property_map<EdgeWeightGraph, boost::edge_weight_t>::type weightmap_;
//    
//    // Algorithm knobs
//    bool separate_odd_cycles_;
//};
//
//struct vertex_properties {
//    double weight;
//};
//
//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, struct vertex_properties> Graph;

double ls::power1(vector<Point> &in_prs,Mat &src,Mat &angle,float &tg)
{
    bool ly = line_size(in_prs[0],in_prs[2])<line_size(in_prs[1],in_prs[3]);
    int y_size = ly?line_size(in_prs[1],in_prs[3]):line_size(in_prs[0],in_prs[2]);
    double w = y_size*abs(1-(power(src, angle, in_prs[1], in_prs[3])+power(src, angle, in_prs[0], in_prs[2]))/(2*tg));

    return w;
}

bool ls::pr_detect(Mat &src,Mat &src1,Mat &angle,vector<LSEG> &oolines,vector<vector<Point> > &out_result,int k)
{
float tg = 0;
vector<vector<Point> > result;
if (oolines.size()>2) {
int i=0,j=i+1;
while(i<oolines.size()-1) {

//Mat quad;
vector<Point> rc;
int bl = warf(src, src1, angle, oolines[i], oolines[j], rc,k,tg);

if(bl == 1)
{
int x_size = line_size(rc[0],rc[1])<line_size(rc[2],rc[3])?line_size(rc[2],rc[3]):line_size(rc[0],rc[1]);
int y_size = line_size(rc[0],rc[2])<line_size(rc[1],rc[3])?line_size(rc[1],rc[3]):line_size(rc[0],rc[2]);
int l_size = x_size < y_size ? y_size : x_size;
int s_size = x_size < y_size ? x_size : y_size;
if((float)l_size/(float)s_size > 3)
{
result.push_back(rc);
i = i+1;
j = i;
}
else
{
j= j + 1;
}
}
else if(bl==3||bl==0||bl==2)
{
j= j + 1;
}
if(j == oolines.size())
{
i = i + 1;
j = i + 1;
}
}

if(result.size()==0)
return 0;
else
{
int ok1 = 0;int ok2 = 0;
int **G;
G = new int *[(int)result.size()];
for(int i =0;i<result.size();i++)
{
G[i] = new int[(int)result.size()];
}

for(int i =0;i<result.size();i++)
{
for(int i =0;i<result.size();i++)
{
G[i][j] = 0;
}
}

for(int i =0;i<result.size();i++)
{
for (int j=0; j<result.size(); j++)
{

DecideOberlap(result[i],result[j],Size(angle.cols,angle.rows),ok1,ok2);
G[i][j] = ok1;
G[j][i] = ok2;
}
}
for(int i = 0;i<result.size();i++)
{
bool isleaf = true;
for (j = 0; j<result.size(); j++)
{
if(G[i][j] != 0)
{
isleaf = false;
}
}
if(isleaf) out_result.push_back(result[i]);
}
return 1;
}

}
return 0;
}

void ls::findline(Mat &flood,Mat &angle,vector<LSEG> &lines,int k)
{
    vector<vector<cv::Point> > contours;
    findContours(flood, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    vector<LSEG> oolines;
    vector<cv::Point2f> x;
    vector<cv::Point2f> y;
    for (int m = 0; m<contours.size(); m++)
    {
        Vec4f ssline;
        vector<cv::Point> zcontours;
        int max_py = 0,min_py = 10000,max_px = 0,min_px = 10000;
        for(int n = 0;n < contours[m].size();n++)
        {
            if(max_py < contours[m][n].y) max_py = contours[m][n].y;
            if(min_py > contours[m][n].y) min_py = contours[m][n].y;
            if(max_px < contours[m][n].x) max_px = contours[m][n].x;
            if(min_px > contours[m][n].x) min_px = contours[m][n].x;
        }

        fitLine(contours[m], ssline, CV_DIST_L2, 0,0.01,0.01);

        Point p0,p1;
        if(abs(max_px - min_px)<=5)
        {
            p0.x = ssline[2];
            p0.y = min_py;
            p1.x = ssline[2];
            p1.y = max_py;
        }
        else if(abs(max_py - min_py)<=5)
        {
            p0.x = min_px;
            p0.y = ssline[3];
            p1.x = max_px;
            p1.y = ssline[3];
        }
        else if(ssline[0]>sqrt(2)*0.5)
        {
            int ay = (ssline[1]/ssline[0])*min_px-(ssline[1]/ssline[0])*ssline[2]+ssline[3];
            int ax = min_px;
            int by = (ssline[1]/ssline[0])*max_px-(ssline[1]/ssline[0])*ssline[2]+ssline[3];
            int bx = max_px;
            p0 = bx>ax?Point(ax,ay):Point(bx,by);
            p1 = bx<ax?Point(ax,ay):Point(bx,by);
        }
        else
        {
            p0.y = min_py;
            p0.x = ssline[2]+((double)ssline[0]/(double)ssline[1])*(min_py - ssline[3]);
            p1.y = max_py;
            p1.x = ssline[2]+((double)ssline[0]/(double)ssline[1])*(max_py - ssline[3]);
        }

        cv::Point2f v,fv;
        v.x = ((float)p1.x - (float)p0.x)/line_size(p0,p1);
        v.y = ((float)p1.y - (float)p0.y)/line_size(p0,p1);
        fv.x = -v.x;
        fv.y = -v.y;
        cv::Point px,py;
        px = prpoint(p0, 0, v);
        py = prpoint(p1, 0, fv);
        LSEG outline;
        outline.push_back(px);
        outline.push_back(py);
        lines.push_back(outline);
    }
}



void ls::parallelines(vector<LSEG> &lines,vector<LSEG> &outlines)
{
    QuickSort(lines, (int)lines.size(), 4);
}

void BubbleSort1(vector<Point> &a, int n)
{
    int i, j;
    Point b;
    for (i = 0; i < n; i++)
        for (j = 1; j < n - i; j++)
            if (a[j-1].y > a[j].y)
            {
                b = a[j-1];
                a[j-1] = a[j];
                a[j] = b;
            }
            else if(abs(a[j-1].y - a[j].y) < 10 && a[j-1].x < a[j].x)
            {
                b = a[j-1];
                a[j-1] = a[j];
                a[j] = b;
            }
}

cv::Point get_gravity_center(LSEG &pr)
{
    cv::Point grav_center;
    LINE l1 = makeline(pr[0], pr[3]);
    LINE l2 = makeline(pr[1], pr[2]);
    lineintersect(l1, l2, grav_center);
    return grav_center;
}


void ls::nonbook_extract(Mat &image,Mat &src)
{
    resize(image,image,Size(((float)image.cols/(float)image.rows)*krows,krows));
    resize(src,src,Size(((float)src.cols/(float)src.rows)*krows,krows));
    Mat img = image.clone();
    Mat dst(image.rows,image.cols,CV_8UC1);
    cvtColor(image, image, CV_RGB2GRAY);
    threshold(image, image, 1, 255, CV_THRESH_BINARY_INV);
    //imshow("thresh", image);
    vector<vector<Point> > contours1;
    findContours(image, contours1, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    // imshow("sfdfsdf", image);
    vector<RotatedRect> minRect(contours1.size());
    vector<cv::Mat> cordi;
    Mat drawing = Mat::zeros(image.size(), CV_8UC3);
    for( int i = 0; i < contours1.size(); i++ )
    {

        Mat tmp(4,2,CV_32FC1);
        Point2f rect_points[4];
        minRect[i] = minAreaRect(Mat(contours1[i]));
        minRect[i].points( rect_points );
        //        for (int k = 0; k < 4; k ++)
        //        {
        //            line( drawing, rect_points[k], rect_points[(k+1)%4], Scalar(0,255,0), 1, 8 );
        //            circle(drawing, rect_points[k], 5, Scalar(255,255,255),-1,5,0);
        //        }

        tmp.at<float>(0, 0) = rect_points[0].x;
        tmp.at<float>(0, 1) = rect_points[0].y;
        tmp.at<float>(2, 0) = rect_points[1].x;
        tmp.at<float>(2, 1) = rect_points[1].y;
        tmp.at<float>(3, 0) = rect_points[2].x;
        tmp.at<float>(3, 1) = rect_points[2].y;
        tmp.at<float>(1, 0) = rect_points[3].x;
        tmp.at<float>(1, 1) = rect_points[3].y;
        cordi.push_back(tmp);

    }
    Mat srcc = src.clone();
    black(src, cordi);

    Canny(src, dst, 200, 600);//200  163 600
    morphologyEx(dst, dst, MORPH_DILATE, Mat(15,15,CV_8U),Point(-1,-1),1);

    vector<vector<Point> > contours;
    //轮廓查找,每一条直线
    findContours(dst, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    int cnt = 0;
    double s;
    cv::Point middle;
    vector<Rect> rects,max_rect;
    for (int i = 0; i < contours.size(); i ++)
    {
        int max_py = 0,min_py = 10000,max_px = 0,min_px = 10000;//最大和最小点的初值
        for(int j = 0;j < contours[i].size();j++)
        {

            //从返回的轮廓中找到最大点和最小点
            if(max_py < contours[i][j].y) max_py = contours[i][j].y;
            if(min_py > contours[i][j].y) min_py = contours[i][j].y;
            if(max_px < contours[i][j].x) max_px = contours[i][j].x;
            if(min_px > contours[i][j].x) min_px = contours[i][j].x;
        }
        Rect rect;
        cv::Point a = cv::Point(min_px,min_py);
        cv::Point b = cv::Point(max_px,max_py);
        rect.x = min_px;
        rect.y = min_py;
        rect.width = b.x - a.x;
        rect.height = b.y -a.y;
        s = rect.area();
        //cout<<"s:"<<s<<endl;

        //            int max_area = 0,min_area = 3000 * 3000;
        //            if(max_area < s) max_area = s;
        //            cout<<max_area<<endl;
        //            if(min_area > s) min_area = s;
        //            cout<<min_area<<endl;
        //            int thresh = (max_area + min_area)/2;
        if (s > 110 * 40 &&  (rect.height < 0.5*image.rows && rect.width < 0.5*image.cols))//&& s <  max_area)
        {
            cnt ++;
            rects.push_back(rect);
            middle.x = min_px + 0.5*rect.width;
            middle.y = min_py + 0.5*rect.height;

            cv::Point offset = Point(3,3);
            cv::Point pc = middle - offset;
            circle(srcc, middle, 5, Scalar(255,0,0),-1,5,0);
            char *text = new char[100];
            sprintf(text, "%d",cnt);
            putText(srcc, text, pc, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(255,255,30),0.2);
            vector<int> hull;
            convexHull(contours[i], hull,true);
            Point poi = contours[i][hull[hull.size()-1]];
            for(int k=0;k < hull.size();k++)
            {
                cv::Point p_new = contours[i][hull[k]];
                line (srcc, poi, p_new, Scalar(255, 0, 0));
                poi = p_new;
            }
        }
    }

#if 0
    int m,n;
    for ( m = 0; m < rects.size(); m++)
    {
        Rect r = rects[m];
        for ( n = 0; n < rects.size(); n++)
            if (n != m && (r & rects[n]) == r)
                break;
        if (n == (rects.size() - 1))
        {
            max_rect.push_back(r);
        }
    }



#endif

    //blur(image, image, Size(3,3));
}

void ls::bookSegmentStart(Mat &src1,vector<Mat> &out_result,vector<Mat> &corner)
{
    Mat image;

    resize(src1,image,Size(((float)src1.cols/(float)src1.rows)*krows,krows));

    Mat blur_out;

    cvtColor(image, blur_out, CV_RGB2GRAY);

    equalizeHist(blur_out, blur_out);

    medianBlur(blur_out, blur_out, 3);

    GaussianBlur(blur_out, blur_out, Size(5,5),0,0);

    Mat out_x,out_y;

    Sobel(blur_out, out_x, CV_32F, 0, 1 ,3);

    Sobel(blur_out, out_y, CV_32F, 1, 0 ,3);

    blur(out_x, out_x, Size(5,5));

    blur(out_x, out_x, Size(5,5));

    Mat magnitude,angle;

    cartToPolar(out_y, out_x, magnitude, angle);

    vector<Vec4i> lSegs,outlines;

    Ptr<LineSegmentDetector> bd = createLineSegmentDetector();

    bd->detect(blur_out, lSegs);

    out_x.release();
    out_y.release();
    magnitude.release();
    blur_out.release();

    vector<LSEG> liness;
    vector<vector<Point> > results;
    for(int i=0;i<lSegs.size();i++)
    {
        cv::Point pt1 = cv::Point(lSegs[i][0],lSegs[i][1]);
        cv::Point pt2 = cv::Point(lSegs[i][2],lSegs[i][3]);

        if(line_size(pt1,pt2)>65)
        {
            cv::Point2f v,fv;
            v.x = ((float)pt2.x - (float)pt1.x)/line_size(pt1,pt2);
            v.y = ((float)pt2.y - (float)pt1.y)/line_size(pt1,pt2);
            fv.x = -v.x;
            fv.y = -v.y;
            cv::Point p1,p2;
            p1 = prpoint(pt1, 0, fv);
            p2 = prpoint(pt2, 0, v);
            LSEG outline;
            outline.push_back(pt1);
            outline.push_back(pt2);
            liness.push_back(outline);
        }
    }

    vector<vector<LSEG> > clines;
    vector<LSEG> zlines;
    QuickSort(liness, (int)liness.size(), -1);
    for(int i = 0; i < liness.size(); i++) {
        if(zlines.size() == 0)
        {
            zlines.push_back(liness[i]);
        }
        else if (i == liness.size()-1 && line_jiao(liness[i][0], liness[i][1])-line_jiao(zlines[zlines.size()-1][0],zlines[zlines.size()-1][1])>2)
        {
            clines.push_back(zlines);
            zlines.clear();
            zlines.push_back(liness[i]);
            clines.push_back(zlines);
        }
        else if(i == liness.size()-1 && line_jiao(liness[i][0], liness[i][1])-line_jiao(zlines[zlines.size()-1][0],zlines[zlines.size()-1][1])<2)
        {
            zlines.push_back(liness[i]);
            clines.push_back(zlines);
        }
        else if (line_jiao(liness[i][0], liness[i][1])-line_jiao(zlines[zlines.size()-1][0],zlines[zlines.size()-1][1])>2) {
            clines.push_back(zlines);
            zlines.clear();
            zlines.push_back(liness[i]);
        }
        else
        {
            zlines.push_back(liness[i]);
        }
    }

    for (int i = 0; i < clines.size(); i++) {
        vector<vector<LSEG> > clines1;
        vector<LSEG> outlines,zlines1;
        Mat a = Mat::zeros(image.rows, image.cols, CV_8UC1);
        for(int j = 0;j<clines[i].size();j++)
        {
            line(a, clines[i][j][0], clines[i][j][1], Scalar(255,255,255),2,8);
        }

        findline(a, angle, outlines, 0);
        QuickSort(outlines, (int)outlines.size(), 1);
        for(int j = 0;j < outlines.size();j++)
        {
            if(zlines1.size() == 0)
            {
                zlines1.push_back(outlines[j]);
            }
            else if (j == outlines.size()-1 && (ca(outlines[j],1)-ca(zlines1[zlines1.size()-1],1) > 3 || abs(line_jiao(outlines[j][0], outlines[j][1])-line_jiao(zlines1[zlines1.size()-1][0],zlines1[zlines1.size()-1][1])) > 1))
            {
                clines1.push_back(zlines1);
                zlines1.clear();
                zlines1.push_back(outlines[j]);
                clines1.push_back(zlines1);
            }
            else if(j == outlines.size()-1 && ca(outlines[j],1)-ca(zlines1[zlines1.size()-1],1) <= 3 && abs(line_jiao(outlines[j][0], outlines[j][1])-line_jiao(zlines1[zlines1.size()-1][0],zlines1[zlines1.size()-1][1])) <= 1)
            {
                zlines1.push_back(outlines[j]);
                clines1.push_back(zlines1);
            }
            else if (ca(outlines[j], 1)-ca(zlines1[zlines1.size()-1],1)>3 || abs(line_jiao(outlines[j][0], outlines[j][1])-line_jiao(zlines1[zlines1.size()-1][0],zlines1[zlines1.size()-1][1])) > 1) {
                clines1.push_back(zlines1);
                zlines1.clear();
                zlines1.push_back(outlines[j]);
            }
            else
            {
                zlines1.push_back(outlines[j]);
            }
        }
        vector<LSEG> out_lines;
        for(int m = 0;m < clines1.size();m++)
        {
            if(clines1[m].size() == 1)
            {
                out_lines.push_back(clines1[m][0]);
            }
            else
            {
                vector<Point> pts;
                for (int n = 0; n < clines1[m].size(); n++) {
                    pts.push_back(clines1[m][n][0]);
                    pts.push_back(clines1[m][n][1]);
                }
                BubbleSort1(pts, (int)pts.size());
                LSEG zline;
                zline.push_back(pts[0]);
                zline.push_back(pts[(int)pts.size()-1]);
                out_lines.push_back(zline);
            }
        }
//         Mat img = image.clone();
//        for (int i = 0; i < out_lines.size(); i++) {
////            line(img, out_lines[i][0], out_lines[i][1], Scalar(255,255,255),2,8);
////            imshow("img",img);
////            waitKey(0);
//        }
////        imshow("iamge", image);
////        waitKey(0);
        pr_detect(image,src1,angle,out_lines,results,0);
    }

//    Mat IMG = image.clone();
//    for (int i = 0; i<results.size(); i++) {
//        Mat pt(1,2,CV_32FC1);
//        cv::Point center = get_gravity_center(results[i]);
//        pt.at<float>(0,0) = center.x;
//        pt.at<float>(0,1) = center.y;
//        gravity_center.push_back(pt);
//    }
    angle.release();

    for (int i = 0; i<results.size(); i++) {
        Mat a;
        Mat coordinate(4,2,CV_32FC1);
        vector<Point2f> points;
        warp_change(results[i],src1,image,a,points);
        coordinate.at<float>(0,0) = points[0].x;
        coordinate.at<float>(0,1) = points[0].y;
        coordinate.at<float>(1,0) = points[1].x;
        coordinate.at<float>(1,1) = points[1].y;
        coordinate.at<float>(2,0) = points[2].x;
        coordinate.at<float>(2,1) = points[2].y;
        coordinate.at<float>(3,0) = points[3].x;
        coordinate.at<float>(3,1) = points[3].y;
        out_result.push_back(a);
        corner.push_back(coordinate);
    }

    image.release();
}

void ls::black(Mat &src1,vector<Mat> &coordinates)
{
    for(int i = 0;i<coordinates.size();i++)
    {
        Mat img1 = Mat::zeros(src1.rows,src1.cols,CV_8UC1);
        line(img1, Point(coordinates[i].at<float>(0,0),coordinates[i].at<float>(0,1)), Point(coordinates[i].at<float>(1,0),coordinates[i].at<float>(1,1)), Scalar(125),2,8);
        line(img1, Point(coordinates[i].at<float>(0,0),coordinates[i].at<float>(0,1)), Point(coordinates[i].at<float>(2,0),coordinates[i].at<float>(2,1)), Scalar(125),2,8);
        line(img1, Point(coordinates[i].at<float>(3,0),coordinates[i].at<float>(3,1)), Point(coordinates[i].at<float>(1,0),coordinates[i].at<float>(1,1)), Scalar(125),2,8);
        line(img1, Point(coordinates[i].at<float>(3,0),coordinates[i].at<float>(3,1)), Point(coordinates[i].at<float>(2,0),coordinates[i].at<float>(2,1)), Scalar(125),2,8);
        Point seed1;
        seed1 = Point((coordinates[i].at<float>(0,0)+coordinates[i].at<float>(1,0)+coordinates[i].at<float>(2,0)+coordinates[i].at<float>(3,0))*0.25,(coordinates[i].at<float>(0,1)+coordinates[i].at<float>(1,1)+coordinates[i].at<float>(2,1)+coordinates[i].at<float>(3,1))*0.25);
        floodFill(img1, seed1, Scalar(125));
        threshold(img1, img1, 1, 255, CV_THRESH_BINARY_INV);
        cvtColor(img1, img1, CV_GRAY2BGR);
        src1 = src1 & img1;
        img1.release();
    }
}