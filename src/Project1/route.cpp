// #include <iostream>
// #include "Astar.h"
// #include "DWA.h"
// #include <vector>
# include "lib_route.h"


bool InPath(const int& row, const int& col, const std::vector<Point>& path) {  //�ж�row��col�Ƿ���path��
    for (int i = 0; i < path.size();i++) {
        if (row == path[i].x && col == path[i].y) {
            return true;
        }
    }
    return false;
}

int main() {
    //��ʼ����ͼ���ö�ά��������ͼ��1��ʾ�ϰ��0��ʾ��ͨ
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

    //������ʼ�ͽ�����
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



    // // A*�㷨��Ѱ·��
    // std::list<Point*> path = astar.GetPath(start, end, false);
    // vector<Point> path_ori; //���洫ͳA*�㷨��õ�·��
    // for (auto& p : path)
    // {
    //     path_ori.push_back({ p->x,p->y });
    // }
    // vector<Point> dl_path = astar.DeletLinePoint(path_ori);   //ȥ������ڵ�
    // vector<Point> dn_path = astar.DelNBorPoint(dl_path,map); //ȥ������յ�
    // cout << dn_path.size();
    // // ��ӡA*�㷨���
    // for (int i = 0; i < dn_path.size();i++) {
    //     std::cout << "(" << dn_path[i].x << "," << dn_path[i].y << ") ";
    // }
    // std::cout << "\n";
    // vector<double> yaw_astar;//��������ÿ��;���㵽��һ��;����ĽǶ�
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


    // vector<Pointdwa> path_astar; //����A*�㷨��õ�·��
    // for (int i = 0; i < dn_path.size(); i++)
    // {
    //     path_astar.push_back({dn_path[i].x+0.5,dn_path[i].y+0.5});
    // }
    //DWA��ʼ������
    // Pointdwa start_dwa(start.x + 0.5, start.y + 0.5); //���������Сԭ��dwa�����յ���Ҫ���е���
    // Pointdwa end_dwa(end.x + 0.5, end.y + 0.5); 
    // vector<Pointdwa> path_dwa; //����dwa�㷨��õ�·��

    // //DWA�滮����
    // double ori_yaw = atan2((path_astar[1].y - start.y) , (path_astar[1].x - start.x)); //��ʼƫ����ָ���һ��;���㷽��
    // DWA dwa(start_dwa, end_dwa); //��ʼ��DWA��
    // //path_dwa = dwa.planning(path_astar, ori_yaw); //DWA·���滮

    // //��ͼ
    // graph2d g2d(600, 600, { 0,0 }, { 30,30 });
    // g2d.xlabel("x��");
    // g2d.ylabel("y ��");
    // g2d.plotline({ {21,27.5},{23,27.5} },RED);
    // g2d.legend("��ͳA*�㷨·��", { 23,59 }, {29,59}, RED);
    // g2d.plotline({ {21,28.5},{23,28.5} }, BLUE);
    // g2d.legend("�Ľ�A*�㷨·��", { 23,61 }, { 29,61 }, BLUE);
    // g2d.plotline(path_astar); //����A*�㷨·��

    // vector<Pointdwa> path1;
    // for (auto& p : path)
    // {
    //     path1.push_back({ p->x + 0.5,p->y + 0.5 }); //��0.5�����ڸ��ӵĿ������
    // }
    // g2d.plotline(path1,RED);
    // //���ϰ���
    // g2d.drawRectangleFuc({ 1,2 }, { 6,3 }); //��һ�����������½ǣ��ڶ������������Ͻǰ�������������
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
    // //for (int i = 0; i < path_dwa.size(); i++) {       //����;����          
    //   //  g2d.plot({ path_dwa[i].x,path_dwa[i].y });
    // //}
    // g2d.waitKey();

    return 0;
}
