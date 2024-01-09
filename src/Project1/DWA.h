#pragma once

#include <math.h>
#include <list>
#include <vector>
#include <iostream>
using namespace std;
#define PI 3.1415926

struct Car //定义小车的参数
{
    float max_speed = 1.0; //最大速度 1.0
    float min_speed = -1.0; //最大减速度 -0.5
    float max_angular_speed = 40 * PI / 180.0; //最大角速度 40
    float min_angular_speed = -40 * PI / 180.0; //最大减角速度 -40
    float max_accel = 0.15; //最大加速度 0.2
    float max_angular_speed_rate = 15 * PI / 180; //最大角加速度 20
    float v_resolution = 0.01;     // 速度采样分辨率 0.01
    float yaw_rate_resolution = 0.1 * PI / 180; //角速度分辨率
    float dt = 0.1;                //运动学模型预测时间 0.1
    float predict_time = 2;  //一段预测时间 2
    float goal_cost_gain = 0.2; //方向角损失0.2
    float speed_cost_gain = 1.0; //速度损失1.0
    float obstacle_cost_gain = 1.0; //障碍物损失1.0
    float distance_cost_gain =0.095; //目标点距离损失0.1
    float astar_cost_gain = 0.16; //最优路径损失0.15
    float radius = 1.0; //小车的半径 1.0
};

struct Pointdwa
{
    double x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    Pointdwa(double _x, double _y) :x(_x), y(_y)  //变量初始化
    {
    }
    Pointdwa()
    {}
};

struct CarState  //小车的状态
{
    float x;
    float y;
    float yaw; //偏航角
    float speed;
    float angular_speed;
    CarState()
    {}
    CarState(float x_, float y_, float yaw_, float speed_, float angular_speed_) :
        x(x_), y(y_), yaw(yaw_), speed(speed_), angular_speed(angular_speed_)
    {}
};

class DWA
{
public:
    DWA(Pointdwa start, Pointdwa destination);
    vector<Pointdwa> planning(vector<Pointdwa> path_astar, double ori_yaw);

private:
    vector<float> dwa_control(const CarState& carstate, vector<Pointdwa> path_astar);
    vector<float> calc_dw(const CarState& carstate);
    vector<float> calc_best_speed(const CarState& carstate, const vector<float>& dw, vector<Pointdwa> path_astar);
    void predict_trajectory(const CarState& carstate, const float& speed, const float& angular_speed, vector<CarState>& trajectory);
    CarState motion_model(const CarState& carstate, const float& speed, const float& angular_speed);
    float calc_goal_cost(const vector<CarState>& trajectory, vector<Pointdwa> path_astar);
    float calc_obstacle_cost(const vector<CarState>& trajectory);
    float calc_astar_cost(const vector<CarState>& trajectory, vector<Pointdwa> path_astar);
private:
    Car car; //定义小车
    vector<vector<CarState>> trajectory; //用于存放所有轨迹
    vector<Pointdwa> barrier; //用于存放障碍点
    int b_num;//记录障碍的个数
    Pointdwa startPoint; //起始点
    Pointdwa destinationPoint; //终止点
    CarState destinationState; //终止状态 
};
