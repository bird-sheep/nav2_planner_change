// #include <iostream>
// #include "Astar.h"
// #include "DWA.h"
// #include <vector>
# include "lib_route.h"


bool InPath(const int& row, const int& col, const std::vector<Point>& path) {  //判断row和col是否在path中
    for (int i = 0; i < path.size();i++) {
        if (row == path[i].x && col == path[i].y) {
            return true;
        }
    }
    return false;
}

int main() {
    //初始化地图，用二维矩阵代表地图，1表示障碍物，0表示可通
    std::vector<std::vector<int>> map = {
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 1, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 1, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 1, 1, 1,   1, 1, 1, 1, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 1, 1, 1, 1,   1, 0, 0, 0, 0,},
                { 0, 0, 1, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 1, 0,   0, 0, 0, 1, 0,   0, 0, 0, 0, 0,},

                { 0, 0, 1, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 1, 0,   0, 0, 0, 1, 0,   0, 0, 0, 0, 0,},//5
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 1, 0,   0, 0, 0, 1, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 1, 1, 1,   1, 1, 1, 1, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 1, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 1, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},

                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 1, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},//10
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 1, 1,   1, 1, 1, 1, 1,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 1, 1, 1, 1,   1, 1, 1, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 1, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 1,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 1, 0, 0, 0,     0, 0, 0, 0, 1,   0, 0, 0, 0, 0,   0, 0, 0, 0, 1,   0, 0, 0, 0, 0,},

                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 1,   0, 0, 0, 0, 0,   0, 0, 0, 0, 1,   0, 0, 0, 0, 0,},//15
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 1,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 1,     1, 1, 1, 1, 1,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 1,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 1,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},

                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 1,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},//20
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 1,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   1, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 1,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   1, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 1,   1, 1, 1, 1, 1,     1, 0, 0, 0, 0,   0, 0, 0, 0, 1,   1, 1, 1, 1, 1,   1, 0, 0, 0, 0,},

                { 0, 0, 0, 0, 1,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   1, 0, 0, 0, 0,},//25
                { 0, 0, 0, 0, 1,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   1, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,},
                { 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,     0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,   0, 0, 0, 0, 0,}, };
    // Astar astar;
    // astar.InitAstar(map);

    //设置起始和结束点
    Point start(27, 1);
    Point end(6,28);
    Pathdata path = findPath(map, start, end);

    std::cout<<"here is using the lib"<<std::endl;
    for (int i = 0; i < path.path.size();i++) {
        std::cout << "(" << path.path[i].x << "," << path.path[i].y << ") ";
    }

    for (int i = 0; i < path.yaw.size();i++) {
        std::cout << path.yaw[i]<<std::endl;
    }



    // // A*算法找寻路径
    // std::list<Point*> path = astar.GetPath(start, end, false);
    // vector<Point> path_ori; //保存传统A*算法求得的路径
    // for (auto& p : path)
    // {
    //     path_ori.push_back({ p->x,p->y });
    // }
    // vector<Point> dl_path = astar.DeletLinePoint(path_ori);   //去除冗余节点
    // vector<Point> dn_path = astar.DelNBorPoint(dl_path,map); //去除冗余拐点
    // cout << dn_path.size();
    // // 打印A*算法结果
    // for (int i = 0; i < dn_path.size();i++) {
    //     std::cout << "(" << dn_path[i].x << "," << dn_path[i].y << ") ";
    // }
    // std::cout << "\n";
    // vector<double> yaw_astar;//用来保存每个途径点到下一个途径点的角度
    // for (int i = 0; i < dn_path.size() - 1; i++)
    // {
    //     yaw_astar.push_back(atan2((dn_path[i+1].y - dn_path[i].y), (dn_path[i+1].x - dn_path[i].x)));
    // }
    // for (int i = 0; i < yaw_astar.size();i++) {
    //     std::cout << yaw_astar[i]<<std::endl;
    // }
 
    // for (int row = 0; row < map.size(); ++row) {
    //     for (int col = 0; col < map[0].size(); ++col) {
    //         if (InPath(row, col, dn_path)) {
    //             if (map[row][col] != 0) {
    //                 std::cout << "e ";
    //             }
    //             else {
    //                 std::cout << "* ";
    //             }
    //         }
    //         else {
    //             std::cout << map[row][col] << " ";
    //         }
    //     }
    //     std::cout << "\n";
    // }


    // vector<Pointdwa> path_astar; //保存A*算法求得的路径
    // for (int i = 0; i < dn_path.size(); i++)
    // {
    //     path_astar.push_back({dn_path[i].x+0.5,dn_path[i].y+0.5});
    // }
    //DWA初始化部分
    // Pointdwa start_dwa(start.x + 0.5, start.y + 0.5); //由于网格大小原因，dwa的起终点需要进行调整
    // Pointdwa end_dwa(end.x + 0.5, end.y + 0.5); 
    // vector<Pointdwa> path_dwa; //保存dwa算法求得的路径

    // //DWA规划部份
    // double ori_yaw = atan2((path_astar[1].y - start.y) , (path_astar[1].x - start.x)); //初始偏航角指向第一个途径点方向
    // DWA dwa(start_dwa, end_dwa); //初始化DWA类
    // //path_dwa = dwa.planning(path_astar, ori_yaw); //DWA路径规划

    // //画图
    // graph2d g2d(600, 600, { 0,0 }, { 30,30 });
    // g2d.xlabel("x轴");
    // g2d.ylabel("y 轴");
    // g2d.plotline({ {21,27.5},{23,27.5} },RED);
    // g2d.legend("传统A*算法路径", { 23,59 }, {29,59}, RED);
    // g2d.plotline({ {21,28.5},{23,28.5} }, BLUE);
    // g2d.legend("改进A*算法路径", { 23,61 }, { 29,61 }, BLUE);
    // g2d.plotline(path_astar); //画出A*算法路径

    // vector<Pointdwa> path1;
    // for (auto& p : path)
    // {
    //     path1.push_back({ p->x + 0.5,p->y + 0.5 }); //加0.5是由于格子的宽度问题
    // }
    // g2d.plotline(path1,RED);
    // //画障碍物
    // g2d.drawRectangleFuc({ 1,2 }, { 6,3 }); //第一个参数是左下角，第二个参数是右上角啊啊啊啊啊啊啊
    // g2d.drawRectangleFuc({ 3,3 }, { 4,9 });
    // g2d.drawRectangleFuc({ 7,12 }, { 8,18 });
    // g2d.drawRectangleFuc({ 4,18 }, { 11,19 });
    // g2d.drawRectangleFuc({ 3,21 }, { 4,26 });
    // g2d.drawRectangleFuc({ 3,3 }, { 4,9 });
    // g2d.drawRectangleFuc({ 4,23 }, { 7,24 });
    // g2d.drawRectangleFuc({ 3,3 }, { 4,9 });
    // g2d.drawRectangleFuc({ 12,3 }, { 13,10 });
    // g2d.drawRectangleFuc({ 13,6 }, { 15,7 });
    // g2d.drawRectangleFuc({ 12,21 }, { 13,28 });
    // g2d.drawRectangleFuc({ 13,24 }, { 15,25 });
    // g2d.drawRectangleFuc({ 17,9 }, { 18,15 });
    // g2d.drawRectangleFuc({ 14,15 }, { 21,16 });
    // g2d.drawRectangleFuc({ 22,4 }, { 27,5 });
    // g2d.drawRectangleFuc({ 24,5 }, { 25,11 });
    // g2d.drawRectangleFuc({ 24,19 }, { 25,25 });
    // g2d.drawRectangleFuc({ 22,25 }, { 27,26 });
    // //for (int i = 0; i < path_dwa.size(); i++) {       //画出途径点          
    //   //  g2d.plot({ path_dwa[i].x,path_dwa[i].y });
    // //}
    // g2d.waitKey();

    return 0;
}
