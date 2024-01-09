#pragma once

#include <math.h>
#include <list>
#include <vector>
#include <iostream>
using namespace std;
#define PI 3.1415926

struct Car //����С���Ĳ���
{
    float max_speed = 1.0; //����ٶ� 1.0
    float min_speed = -1.0; //�����ٶ� -0.5
    float max_angular_speed = 40 * PI / 180.0; //�����ٶ� 40
    float min_angular_speed = -40 * PI / 180.0; //�������ٶ� -40
    float max_accel = 0.15; //�����ٶ� 0.2
    float max_angular_speed_rate = 15 * PI / 180; //���Ǽ��ٶ� 20
    float v_resolution = 0.01;     // �ٶȲ����ֱ��� 0.01
    float yaw_rate_resolution = 0.1 * PI / 180; //���ٶȷֱ���
    float dt = 0.1;                //�˶�ѧģ��Ԥ��ʱ�� 0.1
    float predict_time = 2;  //һ��Ԥ��ʱ�� 2
    float goal_cost_gain = 0.2; //�������ʧ0.2
    float speed_cost_gain = 1.0; //�ٶ���ʧ1.0
    float obstacle_cost_gain = 1.0; //�ϰ�����ʧ1.0
    float distance_cost_gain =0.095; //Ŀ��������ʧ0.1
    float astar_cost_gain = 0.16; //����·����ʧ0.15
    float radius = 1.0; //С���İ뾶 1.0
};

struct Pointdwa
{
    double x, y; //�����꣬����Ϊ�˷��㰴��C++�����������㣬x������ţ�y��������
    Pointdwa(double _x, double _y) :x(_x), y(_y)  //������ʼ��
    {
    }
    Pointdwa()
    {}
};

struct CarState  //С����״̬
{
    float x;
    float y;
    float yaw; //ƫ����
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
    Car car; //����С��
    vector<vector<CarState>> trajectory; //���ڴ�����й켣
    vector<Pointdwa> barrier; //���ڴ���ϰ���
    int b_num;//��¼�ϰ��ĸ���
    Pointdwa startPoint; //��ʼ��
    Pointdwa destinationPoint; //��ֹ��
    CarState destinationState; //��ֹ״̬ 
};
