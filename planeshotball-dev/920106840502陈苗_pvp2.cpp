#include "DataStruct.h"
#include "math.h"

#include <iostream>
#include <ctime>
#include <cstdlib>
#include <windows.h>

#define random(x) rand()%x//随机数
#define d0 100//距离

const double PI=3.1415926;
const int WI=2000;//宽
const int HI=1500;//高
const int plane_a=4000;//飞机减速加速度
const double angleMin=3.6;//飞机转动角度

inline int sign(int x)
{
    if(x>0)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

double vecMod_ai2(double x,double y)
{
    return sqrt(x*x+y*y);
}
//向量的模

void crashFault_ai2()
{
    exit(0);
}

void timeFault_ai2()
{
    Sleep(1000);
}

void randFault_ai2()
{
    int flag=0;
    if(flag==0)
    {
        srand((int)time(NULL));
        flag++;
    }
    int flag2=random(3);
    if(flag2==1)
    {
        timeFault_ai2();
    }
    else if(flag2==0)
    {
        crashFault_ai2();
    }
}//随机故障检测

//瞄准球函数
int aimBall_ai2(DataStruct *data,BallStruct &ball,int &leftward,int &rightward)
{
    int bul_v=2000;
    double ball_x=ball.x,ball_y=ball.y,ball_v_x=ball.v_x,ball_v_y=ball.v_y;
    double angleTo,angleDiff,cosPosV,modBall,modPlane;
    double angleCon,vecMul;

    vecMul=ball_v_x*(data->plane2.x - ball_x)+ball_v_y*(data->plane2.y-ball_y);
    modBall=vecMod_ai2(ball_v_x,ball_v_y);
    modPlane=vecMod_ai2(data->plane2.x-ball_x,data->plane2.y-ball_y);
    cosPosV=vecMul/modBall/modPlane;
    //余弦
    angleCon=(asin(sqrt(1-cosPosV*cosPosV)*vecMod_ai2(ball_v_x,ball_v_y)/bul_v)+
              atan2(ball_y-data->plane2.y,ball_x-data->plane2.x))*180/PI;
    angleTo=-90-angleCon;
    angleDiff=fmod(fmod(data->plane2.angle-angleTo,360)+360,360);
    //求余数
    if(angleDiff<angleMin||angleDiff>(360-angleMin))
    {
        return 1;
    }
    else if(angleDiff<180)
    {
        leftward=0;
        rightward=1;
    }
    else
    {
        leftward=1;
        rightward=0;
    }
    return 0;
}

//飞机位置预测
int prePlanePos_ai2(DataStruct *data,double &x,double &y)
{
    int time=3000;
    double v_x0,v_y0,v_x1,v_y1,a_x,a_y;
    v_x0=data->plane2.v_x;
    v_y0=data->plane2.v_y;
    if(v_x0==0&&v_y0==0)
    {
        x=data->plane2.x;
        y=data->plane2.y;
        return 0;
    }
    a_x=-v_x0/vecMod_ai2(v_x0,v_y0)*plane_a;
    a_y=-v_y0/vecMod_ai2(v_x0,v_y0)*plane_a;

    v_x1=v_x0+a_x*time/100;
    if(sign(v_x1)!=sign(v_x0))
    {
        v_x1=0;
    }
    v_y1=v_y0+a_y*time/100;
    if(sign(v_y1)!=sign(v_y0))
    {
        v_y1=0;
    }
    x=data->plane2.x+(v_x1*v_x1-v_x0*v_x0)/2/a_x;
    y=data->plane2.y+(v_y1*v_y1-v_y0*v_y0)/2/a_y;
    return 0;
}

//飞机转向
int turnTo_ai2(DataStruct *data,double x,double y,int &leftward,int &rightward)
{
    double angleCon;
    double angleTo,angleDiff;

    angleCon=atan2(y-data->plane2.y,x-data->plane2.x)*180/PI;
    angleTo=-90-angleCon;
    angleDiff=fmod(fmod(data->plane2.angle-angleTo,360)+360,360);
    if(angleDiff<angleMin||angleDiff>(360-angleMin))
    {
        return 1;
    }
    else if(angleDiff<180)
    {
        leftward=0;
        rightward=1;
    }
    else
    {
        leftward=1;
        rightward=0;
    }

    return 0;
}

//飞机移动
int moveTo_ai2(DataStruct *data,double x,double y,int &forward,int &leftward,int &rightward)
{
    int to,prem=1;
    double pre_x,pre_y,dis;

    dis=vecMod_ai2(y-data->plane2.y,x-data->plane2.x);
    if(dis<prem)
    {
        return 1;
    }

    prePlanePos_ai2(data,pre_x,pre_y);

    double turn_x,turn_y;
    turn_x=x+data->plane2.x-pre_x;
    turn_y=y+data->plane2.y-pre_y;
    to=turnTo_ai2(data,turn_x,turn_y,leftward,rightward);
    if(to==0)
    {
        forward=0;
        return 0;
    }

    dis=vecMod_ai2(y-pre_x,x-pre_y);
    if(dis>=prem)
    {
        forward=1;
        leftward=0;
        rightward=0;
    }
    return 0;
}

//判断飞机是否移动
int moveIf_ai2(DataStruct *data,int &des_x,int &des_y)
{
    int preTime=200;
    double min_t=INT_MAX;
    for(int i=0;i<data->ball_size;i++)
    {
        BallStruct &ball=data->ball[i];
        double ball_x=ball.x,ball_y=ball.y,ball_v_x=ball.v_x,ball_v_y=ball.v_y;
        double k,C,C2,dis,cross_x,cross_y;
        double t;

        k=ball_v_y/ball_v_x;
        C=ball_y-ball_x*k;
        C2=data->plane2.y+data->plane2.x/k;
        dis=fabs((k*data->plane2.x-data->plane2.y+C)/vecMod_ai2(1, k));
        cross_x=(C2-C)/(1/k+k);
        cross_y=k*cross_x+C;
        t=(cross_x-ball_x)/ball_v_x*100;

        if (t<0||t>preTime)
        {
            continue;
        }
        if (dis<(data->plane2.r+ball.r+10)&&t<min_t)
        {
            min_t=t;
            int difx,dify;
            difx=data->plane2.x-cross_x;
            dify=data->plane2.y-cross_y;
            des_x=data->plane2.x+ball.r*difx/vecMod_ai2(difx,dify);
            des_y=data->plane2.y+ball.r*dify/vecMod_ai2(difx,dify);

            if(des_x<data->plane2.r||des_x>WI-data->plane2.r)
            {
                des_x=data->plane2.x;
                if(dify>0)
                {
                    des_y=cross_y+ball.r+data->plane2.r;
                }
                else
                {
                    des_y=cross_y-ball.r-data->plane2.r;
                }
            }
            if (des_y<data->plane2.r||des_y>HI-data->plane2.r)
            {
                des_y=data->plane2.y;
                if(difx>0)
                {
                    des_x=cross_x+ball.r+data->plane2.r;
                }
                else
                {
                    des_x=cross_x-ball.r-data->plane2.r;
                }
            }
        }
    }

    if(min_t!=INT_MAX)
    {
        return 1;
    }
    return 0;
}

void ai_pvp_ai2(DataStruct*data,KeyStruct*key){

    static int moveFlag=1,des_x,des_y;

    key->forward=0;
    key->rotate_left=0;
    key->rotate_right=0;
    key->shoot=1;

    if(moveIf_ai2(data,des_x,des_y)||moveFlag==0)
    {
        moveFlag=moveTo_ai2(data,des_x,des_y,key->forward,key->rotate_left,key->rotate_right);

        if(moveFlag==-1)
        {
            std::cout<<"移动错误"<<std::endl;
        }
        //结束
        return;
    }

    if(data->ball_size!=0)
    {
        int d[50],min_d,j=0,flag=0;
        BallStruct &ball=data->ball[0];
        double x0=data->plane2.x-ball.x,y0=data->plane2.y-ball.y;
        double k1,k2;
        d[0]=vecMod_ai2(x0,y0);
        min_d=d[0];

        for(int i=1;i<data->ball_size;i++)
        {
            BallStruct &ball=data->ball[i];
            double x=ball.x,y=ball.y,v_x=ball.v_x,v_y=ball.v_y;

            d[i]=vecMod_ai2(data->plane2.x-x,data->plane2.y-y);
            k1=v_y/v_x;
            k2=(data->plane2.y-y)/(data->plane2.x-x);
            if(k1==k2)
            {
                flag=1;
            }
            if(d[i]<min_d)
            {
                min_d=d[i];
                j=i;
            }                     
        }
        if(min_d<d0)
        {
            aimBall_ai2(data,data->ball[j],key->rotate_left,key->rotate_right);
        }
        //转向100内最近的球射击

        if(min_d<data->plane2.r&&flag)
        {
            key->shield=true;
        }
        //尝试打开护盾

       aimBall_ai2(data,data->ball[0],key->rotate_left,key->rotate_right);
    }
    else if(data->ball_size==0)
    {
        double p_x,p_y;
        prePlanePos_ai2(data,p_x,p_y);
        int t1=turnTo_ai2(data,data->plane2.x+data->plane1.x-p_x,data->plane2.x+data->plane1.y-p_y,key->rotate_left,key->rotate_right);
        if(t1==0)
        {
            key->forward=0;
            return;
        }
    }
    //没有球时瞄准飞机
}


