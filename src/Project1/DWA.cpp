#include "DWA.h"



DWA::DWA(Pointdwa start, Pointdwa destination) //������ʼ��
{
    startPoint = start;
    destinationPoint = destination;
    destinationState.x = destination.x;
    destinationState.y = destination.y;
    destinationState.yaw = 0;
    destinationState.speed = 0;
    destinationState.angular_speed = 0;
    //�ϰ���ʼ��
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

//·���滮
vector<Pointdwa> DWA::planning(vector<Pointdwa> path_astar,double ori_yaw)
{
    vector<Pointdwa> path;
    CarState currentState(startPoint.x, startPoint.y, ori_yaw, 0, 0); //��ʼ�����
    vector<CarState> currentTrajectory; //��ŵ�ǰ�켣
    while (1)
    {
        cout << "**********************************************" << endl;
        vector<float> speed(2);     //v[0]Ϊ�ٶ�, v[1]���ٶ�
        speed = dwa_control(currentState,path_astar);
        cout << "speed:" << speed[0] << ", " << speed[1] << endl;
        currentTrajectory.clear();
        predict_trajectory(currentState, speed[0], speed[1], currentTrajectory);
        trajectory.push_back(currentTrajectory);
        currentState = currentTrajectory.back();
        //�ж��Ƿ񵽴��յ�
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
//��̬���ڷ�
vector<float> DWA::dwa_control(const CarState& carstate, vector<Pointdwa> path_astar)  //���복��״̬������ٶ�
{
    vector<float> dw(4);     //dw[0]Ϊ��С�ٶȣ�dw[1]Ϊ����ٶȣ�dw[2]Ϊ��С���ٶȣ�dw[3]Ϊ�����ٶ�
    //���㶯̬����
    dw = calc_dw(carstate);
    //������ѣ�v, w��
    vector<float> v_w(2);
    v_w = calc_best_speed(carstate, dw,path_astar);
    return v_w;
}
// ���㶯̬����
vector<float> DWA::calc_dw(const CarState& carstate)
{
    // �������ٶ��������ƵĶ�̬����
    vector<float> dw_robot_state{ car.min_speed, car.max_speed, car.min_angular_speed, car.max_angular_speed };
    // ������ģ�����ƵĶ�̬����
    vector<float> dw_robot_model(4);
    dw_robot_model[0] = carstate.speed - car.max_accel * car.dt; //dt�����С�ٶ�
    dw_robot_model[1] = carstate.speed + car.max_accel * car.dt; //dt�������ٶ�
    dw_robot_model[2] = carstate.angular_speed - car.max_angular_speed_rate * car.dt;
    dw_robot_model[3] = carstate.angular_speed + car.max_angular_speed_rate * car.dt;
    vector<float> dw{ max(dw_robot_state[0], dw_robot_model[0]),
                     min(dw_robot_state[1], dw_robot_model[1]),
                     max(dw_robot_state[2], dw_robot_model[2]),
                     min(dw_robot_state[3], dw_robot_model[3]) };//��������״̬���ƵĶ�̬���ںͻ�����ģ�����ƵĶ�̬���ڽ��бȽ�
    return dw;
}
//��dw�м�������ٶȺͽ��ٶ�
vector<float> DWA::calc_best_speed(const CarState& carstate, const vector<float>& dw, vector<Pointdwa> path_astar)
{
    vector<float> best_speed{ 0, 0 };//��ʼ�����ز���
    vector<CarState> trajectoryTmp;//��ʱ����
    float min_cost = 10000;
    float final_cost;
    float goal_cost;
    float speed_cost = 0;
    float obstacle_cost = 0;
    float distance_cost = 0;
    float astar_cost = 0.0;
    for (float i = dw[0]; i < dw[1]; i += car.v_resolution) //�ٶȵ���
    {
        for (float j = dw[2]; j < dw[3]; j += car.yaw_rate_resolution) //���ٶȵ���
        {
            //Ԥ��켣
            trajectoryTmp.clear();
            predict_trajectory(carstate, i, j, trajectoryTmp);
            //�������
            goal_cost = car.goal_cost_gain * calc_goal_cost(trajectoryTmp,path_astar); //��λ�Ǵ���
            speed_cost = car.speed_cost_gain * (car.max_speed - trajectoryTmp.back().speed);//vmax-v
            obstacle_cost = car.obstacle_cost_gain * calc_obstacle_cost(trajectoryTmp); //�ϰ�����
            distance_cost = car.distance_cost_gain * sqrt(pow(destinationState.x - trajectoryTmp.back().x, 2) + pow(destinationState.y - trajectoryTmp.back().y, 2));//����Ŀ���Ĵ���
            astar_cost = car.astar_cost_gain * calc_astar_cost(trajectoryTmp,path_astar);
            final_cost = goal_cost + speed_cost + obstacle_cost + distance_cost + astar_cost;
            //cout << "goal_cost" << goal_cost << "  " << "speed_cost" << speed_cost << "  " << "obstacle_cost" << obstacle_cost << "  " << "distance_cost" << distance_cost << "  " << "astar_cost" << astar_cost << "  " << "final_cost:" << final_cost << endl;
            if (final_cost < min_cost) //�������Ž�
            {
                min_cost = final_cost;
                best_speed[0] = i;
                best_speed[1] = j;
            }
            if (best_speed[0] < 0.001 && carstate.speed < 0.001) //�������ٶȵ�ֵΪ�ǳ��ӽ������С�������ҵ�ǰ�������ٶ�Ҳ�ӽ����㣬����ѽ��ٶ�����Ϊ���������ٶȡ�
            {
                best_speed[1] = - car.max_angular_speed_rate;
                best_speed[0] = car.v_resolution; //����С����ԭ�ش�ת
            }
        }
    }
    //cout << "best_speed:" << best_speed[0] << ",   " << best_speed[1] << endl;
    return best_speed;
}
// ��һ��ʱ����Ԥ��켣
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
//���ݶ���ѧģ�ͼ�����һʱ��״̬
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
// ���㷽λ�Ǵ���
float DWA::calc_goal_cost(const vector<CarState>& trajectory, vector<Pointdwa> path_astar)
{
    float distance = 10000;
    int next_block = 0;//��¼��ǰС�������ĸ��м��ǰ��
    auto last_state = trajectory.back();
    for (int i = 0; i < path_astar.size() - 1; i++) {
        if (distance > sqrt(pow(path_astar[i].x - last_state.x, 2) + pow(path_astar[i].y - last_state.y, 2))) {
            next_block = i+1; //�Ծ�������ĵ����һ����ΪĿ��
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

// �����ϰ�����
float DWA::calc_obstacle_cost(const vector<CarState>& trajectory)
{
    //float obstacle_cost;
    float distance;
    for (int i = 0; i < b_num; i++) {
        for (int j = 0; j < trajectory.size(); j++) {
            distance = sqrt(pow(barrier[i].x - trajectory[j].x, 2) + pow(barrier[i].y - trajectory[j].y, 2)); //�����ϰ���̫����Լ��
            if (distance <= car.radius) 
                return 10000.0;
            if (trajectory[j].x > 30 || trajectory[j].x < 0 || trajectory[j].y>30 || trajectory[j].y < 0) //�����߽��Լ��
                return 10000.0;
        }
    }
    return 0;
}
//����A*����·���Ĵ���
float DWA::calc_astar_cost(const vector<CarState>& trajectory, vector<Pointdwa> path_astar) 
{
    float distance = 10000.0;
    int current_block = 0;//��¼��ǰС�������ĸ��м��ǰ��
    auto last_state = trajectory.back();
    for (int i = 0; i < path_astar.size() - 1; i++) {
        if (distance > sqrt(pow(path_astar[i].x - last_state.x, 2) + pow(path_astar[i].y - last_state.y, 2))) {
            current_block = i + 1; //�Ծ�������ĵ����һ����ΪĿ��
            distance = sqrt(pow(path_astar[i].x - last_state.x, 2) + pow(path_astar[i].y - last_state.y, 2));
        }
    }
    distance = 0.0;
    for (int j = 0; j < trajectory.size(); j++) {
        distance += sqrt(pow(path_astar[current_block].x - trajectory[j].x, 2) + pow(path_astar[current_block].y - trajectory[j].y, 2));
    }
    return distance;
}



