#include "DWA.h"



DWA::DWA(Pointdwa start, Pointdwa destination) //变量初始化
{
    startPoint = start;
    destinationPoint = destination;
    destinationState.x = destination.x;
    destinationState.y = destination.y;
    destinationState.yaw = 0;
    destinationState.speed = 0;
    destinationState.angular_speed = 0;
    //障碍初始化
    b_num = 0;
    float i, j;
    for (i = 1.5; i <= 5.5; i = i + 1.0) {
        barrier.push_back({ i,2.5 });
        b_num++;
    }
    for (j = 3.5; j <= 8.5; j = j + 1.0) {
        barrier.push_back({ 3.5,j });
        b_num++;
    }
    for (j = 21.5; j <= 25.5; j++) {
        barrier.push_back({ 3.5,j });
        b_num++;
    }
    for (i = 4.5; i <= 6.5; i++) {
        barrier.push_back({ i,23.5 });
        b_num++;
    }
    for (i = 4.5; i <= 10.5; i++) {
        barrier.push_back({ i,18.5 });
        b_num++;
    }
    for (j = 12.5; j <= 17.5; j++) {
        barrier.push_back({ 7.5,j });
        b_num++;
    }
    for (j = 3.5; j <= 9.5; j++) {
        barrier.push_back({ 12.5,j });
        b_num++;
    }
    for (j = 21.5; j <= 27.5; j++) {
        barrier.push_back({ 12.5,j });
        b_num++;
    }
    for (i = 13.5; i <= 14.5; i++) {
        barrier.push_back({ i,6.5 });
        b_num++;
    }
    for (i = 13.5; i <= 14.5; i++) {
        barrier.push_back({ i,24.5 });
        b_num++;
    }
    for (i = 14.5; i <= 20.5; i++) {
        barrier.push_back({ i,15.5 });
        b_num++;
    }
    for (j = 9.5; j <= 14.5; j++) {
        barrier.push_back({ 17.5,j });
        b_num++;
    }
    for (j = 5.5; j <= 10.5; j++) {
        barrier.push_back({ 24.5,j });
        b_num++;
    }
    for (j = 19.5; j <= 24.5; j++) {
        barrier.push_back({ 24.5,j });
        b_num++;
    }
    for (i = 22.5; i <= 26.5; i++) {
        barrier.push_back({ i,4.5 });
        b_num++;
    }
    for (i = 22.5; i <= 26.5; i++) {
        barrier.push_back({ i,25.5 });
        b_num++;
    }
}

//路径规划
vector<Pointdwa> DWA::planning(vector<Pointdwa> path_astar,double ori_yaw)
{
    vector<Pointdwa> path;
    CarState currentState(startPoint.x, startPoint.y, ori_yaw, 0, 0); //初始化起点
    vector<CarState> currentTrajectory; //存放当前轨迹
    while (1)
    {
        cout << "**********************************************" << endl;
        vector<float> speed(2);     //v[0]为速度, v[1]角速度
        speed = dwa_control(currentState,path_astar);
        cout << "speed:" << speed[0] << ", " << speed[1] << endl;
        currentTrajectory.clear();
        predict_trajectory(currentState, speed[0], speed[1], currentTrajectory);
        trajectory.push_back(currentTrajectory);
        currentState = currentTrajectory.back();
        //判断是否到达终点
        if (pow(currentState.x - destinationState.x, 2) + pow(currentState.y - destinationState.y, 2) <= car.radius * car.radius)
        {
            cout << "Done" << endl;
            break;
        }
        path.push_back({ currentState.x , currentState.y });
        cout << "currentState:" << currentState.x << ", " << currentState.y << endl;
    }
    return path;
}
//动态窗口法
vector<float> DWA::dwa_control(const CarState& carstate, vector<Pointdwa> path_astar)  //输入车的状态，输出速度
{
    vector<float> dw(4);     //dw[0]为最小速度，dw[1]为最大速度，dw[2]为最小角速度，dw[3]为最大角速度
    //计算动态窗口
    dw = calc_dw(carstate);
    //计算最佳（v, w）
    vector<float> v_w(2);
    v_w = calc_best_speed(carstate, dw,path_astar);
    return v_w;
}
// 计算动态窗口
vector<float> DWA::calc_dw(const CarState& carstate)
{
    // 机器人速度属性限制的动态窗口
    vector<float> dw_robot_state{ car.min_speed, car.max_speed, car.min_angular_speed, car.max_angular_speed };
    // 机器人模型限制的动态窗口
    vector<float> dw_robot_model(4);
    dw_robot_model[0] = carstate.speed - car.max_accel * car.dt; //dt后的最小速度
    dw_robot_model[1] = carstate.speed + car.max_accel * car.dt; //dt后的最大速度
    dw_robot_model[2] = carstate.angular_speed - car.max_angular_speed_rate * car.dt;
    dw_robot_model[3] = carstate.angular_speed + car.max_angular_speed_rate * car.dt;
    vector<float> dw{ max(dw_robot_state[0], dw_robot_model[0]),
                     min(dw_robot_state[1], dw_robot_model[1]),
                     max(dw_robot_state[2], dw_robot_model[2]),
                     min(dw_robot_state[3], dw_robot_model[3]) };//将机器人状态限制的动态窗口和机器人模型限制的动态窗口进行比较
    return dw;
}
//在dw中计算最佳速度和角速度
vector<float> DWA::calc_best_speed(const CarState& carstate, const vector<float>& dw, vector<Pointdwa> path_astar)
{
    vector<float> best_speed{ 0, 0 };//初始化返回参数
    vector<CarState> trajectoryTmp;//临时策略
    float min_cost = 10000;
    float final_cost;
    float goal_cost;
    float speed_cost = 0;
    float obstacle_cost = 0;
    float distance_cost = 0;
    float astar_cost = 0.0;
    for (float i = dw[0]; i < dw[1]; i += car.v_resolution) //速度迭代
    {
        for (float j = dw[2]; j < dw[3]; j += car.yaw_rate_resolution) //角速度迭代
        {
            //预测轨迹
            trajectoryTmp.clear();
            predict_trajectory(carstate, i, j, trajectoryTmp);
            //计算代价
            goal_cost = car.goal_cost_gain * calc_goal_cost(trajectoryTmp,path_astar); //方位角代价
            speed_cost = car.speed_cost_gain * (car.max_speed - trajectoryTmp.back().speed);//vmax-v
            obstacle_cost = car.obstacle_cost_gain * calc_obstacle_cost(trajectoryTmp); //障碍代价
            distance_cost = car.distance_cost_gain * sqrt(pow(destinationState.x - trajectoryTmp.back().x, 2) + pow(destinationState.y - trajectoryTmp.back().y, 2));//距离目标点的代价
            astar_cost = car.astar_cost_gain * calc_astar_cost(trajectoryTmp,path_astar);
            final_cost = goal_cost + speed_cost + obstacle_cost + distance_cost + astar_cost;
            //cout << "goal_cost" << goal_cost << "  " << "speed_cost" << speed_cost << "  " << "obstacle_cost" << obstacle_cost << "  " << "distance_cost" << distance_cost << "  " << "astar_cost" << astar_cost << "  " << "final_cost:" << final_cost << endl;
            if (final_cost < min_cost) //更新最优解
            {
                min_cost = final_cost;
                best_speed[0] = i;
                best_speed[1] = j;
            }
            if (best_speed[0] < 0.001 && carstate.speed < 0.001) //如果最佳速度的值为非常接近于零的小数，并且当前车辆的速度也接近于零，则将最佳角速度设置为负的最大角速度。
            {
                best_speed[1] = - car.max_angular_speed_rate;
                best_speed[0] = car.v_resolution; //避免小车在原地打转
            }
        }
    }
    //cout << "best_speed:" << best_speed[0] << ",   " << best_speed[1] << endl;
    return best_speed;
}
// 在一段时间内预测轨迹
void DWA::predict_trajectory(const CarState& carstate, const float& speed, const float& angular_speed, vector<CarState>& trajectory)
{
    float time = 0;
    CarState nextState = carstate;
    nextState.speed = speed;
    nextState.angular_speed = angular_speed;
    while (time < car.predict_time)
    {
        nextState = motion_model(nextState, speed, angular_speed);
        //cout << "nextState:(" << nextState.x << ", " << nextState.y << ", " << nextState.yaw * 180 / PI << ")" << nextState.speed << "  " << nextState.angular_speed << endl;
        trajectory.push_back(nextState);
        time += car.dt;
    }
}
//根据动力学模型计算下一时刻状态
CarState DWA::motion_model(const CarState& carstate, const float& speed, const float& angular_speed)
{
    CarState nextState;
    nextState.x = carstate.x + speed * car.dt * cos(carstate.yaw);
    nextState.y = carstate.y + speed * car.dt * sin(carstate.yaw);
    nextState.yaw = carstate.yaw + angular_speed * car.dt;
    nextState.speed = carstate.speed;
    nextState.angular_speed = carstate.angular_speed;
    return nextState;
}
// 计算方位角代价
float DWA::calc_goal_cost(const vector<CarState>& trajectory, vector<Pointdwa> path_astar)
{
    float distance = 10000;
    int next_block = 0;//记录当前小车向着哪个中间点前进
    auto last_state = trajectory.back();
    for (int i = 0; i < path_astar.size() - 1; i++) {
        if (distance > sqrt(pow(path_astar[i].x - last_state.x, 2) + pow(path_astar[i].y - last_state.y, 2))) {
            next_block = i+1; //以距离最近的点的下一个点为目标
            distance = sqrt(pow(path_astar[i].x - last_state.x, 2) + pow(path_astar[i].y - last_state.y, 2));
        }
    }
    float error_yaw = atan2(path_astar[next_block].y - trajectory.back().y, path_astar[next_block].x - trajectory.back().x);
    float goal_cost = error_yaw - trajectory.back().yaw;
    goal_cost = atan2(sin(goal_cost), cos(goal_cost));
    if (goal_cost >= 0)
        return goal_cost;
    else
        return -goal_cost;
    /*
    float error_yaw = atan2(destinationState.y - trajectory.back().y, destinationState.x - trajectory.back().x);
    float goal_cost = error_yaw - trajectory.back().yaw;
    //    cout << "error_yaw :" << error_yaw << "    yaw:" << trajectory.back().yaw;
    goal_cost = atan2(sin(goal_cost), cos(goal_cost));
    //    cout << "    final:" << goal_cost << endl;
    if (goal_cost >= 0)
        return goal_cost;
    else
        return -goal_cost;
     */

}

// 计算障碍代价
float DWA::calc_obstacle_cost(const vector<CarState>& trajectory)
{
    //float obstacle_cost;
    float distance;
    for (int i = 0; i < b_num; i++) {
        for (int j = 0; j < trajectory.size(); j++) {
            distance = sqrt(pow(barrier[i].x - trajectory[j].x, 2) + pow(barrier[i].y - trajectory[j].y, 2)); //距离障碍物太近的约束
            if (distance <= car.radius) 
                return 10000.0;
            if (trajectory[j].x > 30 || trajectory[j].x < 0 || trajectory[j].y>30 || trajectory[j].y < 0) //超出边界的约束
                return 10000.0;
        }
    }
    return 0;
}
//距离A*最优路径的代价
float DWA::calc_astar_cost(const vector<CarState>& trajectory, vector<Pointdwa> path_astar) 
{
    float distance = 10000.0;
    int current_block = 0;//记录当前小车向着哪个中间点前进
    auto last_state = trajectory.back();
    for (int i = 0; i < path_astar.size() - 1; i++) {
        if (distance > sqrt(pow(path_astar[i].x - last_state.x, 2) + pow(path_astar[i].y - last_state.y, 2))) {
            current_block = i + 1; //以距离最近的点的下一个点为目标
            distance = sqrt(pow(path_astar[i].x - last_state.x, 2) + pow(path_astar[i].y - last_state.y, 2));
        }
    }
    distance = 0.0;
    for (int j = 0; j < trajectory.size(); j++) {
        distance += sqrt(pow(path_astar[current_block].x - trajectory[j].x, 2) + pow(path_astar[current_block].y - trajectory[j].y, 2));
    }
    return distance;
}



