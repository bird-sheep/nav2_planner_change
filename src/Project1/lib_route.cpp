#include "lib_route.h"

Pathdata findPath( std::vector<std::vector<int>>& map,  Point& start,  Point& end) {
    Astar astar;
    astar.InitAstar(map);

    // A*算法找寻路径
    std::list<Point*> path = astar.GetPath(start, end, false);
    std::vector<Point> path_ori; //保存传统A*算法求得的路径
    for (auto& p : path)
    {
        path_ori.push_back({ p->x,p->y });
    }
    std::vector<Point> dl_path = astar.DeletLinePoint(path_ori);   //去除冗余节点
    std::vector<Point> dn_path = astar.DelNBorPoint(dl_path,map); //去除冗余拐点
    
    std::vector<double> yaw_astar;//用来保存每个途径点到下一个途径点的角度
    for (int i = 0; i < dn_path.size() - 1; i++)
    {
        yaw_astar.push_back(atan2((dn_path[i+1].y - dn_path[i].y), (dn_path[i+1].x - dn_path[i].x)));
    }
    Pathdata pathdata;
    dn_path.pop_back(); //去除最后一个途径点，因为它是终点
    pathdata.path = dn_path;
    pathdata.yaw = yaw_astar;
    return pathdata;
}