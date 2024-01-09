#include <math.h>
#include "Astar.h"
#include <iostream>
#include <cmath>

void Astar::InitAstar(std::vector<std::vector<int>>& _maze)
{
	maze = _maze;
}

std::vector<std::vector<int>> Astar::convertTo2DArray(const int input[], int rows, int cols) {
    std::vector<std::vector<int>> output(rows, std::vector<int>(cols));
    int index = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            output[i][j] = abs(input[index++]);
        }
    }
    return output;
}

int Astar::calcG(Point* temp_start, Point* point)  //计算G
{
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空
	return parentG + extraG;
}

int Astar::calcH(Point* point, Point* end)
{
	//用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法
	return sqrt((double)(end->x - point->x) * (double)(end->x - point->x) + (double)(end->y - point->y) * (double)(end->y - point->y)) * kCost1;
}

int Astar::calcF(Point* point, Point& startPoint, Point& endPoint)
{
	//改动一：加入k
	auto L = calcH(&startPoint, &endPoint);
	auto l = point->H;
	auto k = 1 + l / L;
	return point->G + k*(point->H);
}

Point* Astar::getLeastFpoint() //获取开放列表中F值最小的点
{
	if (!openList.empty())
	{
		auto resPoint = openList.front(); //指向第一个元素
		for (auto& point : openList)
			if (point->F < resPoint->F)
				resPoint = point;
		return resPoint;
	}
	return NULL;
}

Point* Astar::findPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{
	openList.push_back(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //找到F值最小的点
		openList.remove(curPoint); //从开启列表中删除
		closeList.push_back(curPoint); //放到关闭列表
		//1,找到当前周围八个格中可以通过的格子
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto& target : surroundPoints)
		{
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target,startPoint,endPoint);

				openList.push_back(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else
			{
				int tempG = calcG(curPoint, target);
				if (tempG < target->G)
				{
					target->parent = curPoint;
					target->G = tempG;
					target->F = calcF(target,startPoint,endPoint);
				}
			}
			Point* resPoint = isInList(openList, &endPoint);
			if (resPoint)
				return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
		}
	}

	return NULL;
}

std::list<Point*> Astar::GetPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{
	Point* result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point*> path;
	//返回路径，如果没找到路径，返回空链表
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}
	// 清空临时开闭列表，防止重复执行GetPath导致结果异常
	openList.clear();
	closeList.clear();

	return path;
}

Point* Astar::isInList(const std::list<Point*>& list, const Point* point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list)
		if (p->x == point->x && p->y == point->y)
			return p;
	return NULL;
}

bool Astar::isCanreach(const Point* point, const Point* target, bool isIgnoreCorner) const
{
	if (target->x<0 || target->x>maze.size() - 1
		|| target->y<0 || target->y>maze[0].size() - 1
		|| maze[target->x][target->y] == 1
		|| target->x == point->x && target->y == point->y
		|| isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
		return false;
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以
			return true;
		else
		{
			//斜对角要判断是否绊住
			if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
				return true;
			else
				return isIgnoreCorner;
		}
	}
}

std::vector<Point*> Astar::getSurroundPoints(const Point* point, bool isIgnoreCorner) const
{
	std::vector<Point*> surroundPoints;

	for (int x = point->x - 1; x <= point->x + 1; x++)
		for (int y = point->y - 1; y <= point->y + 1; y++)
			if (isCanreach(point, new Point(x, y), isIgnoreCorner))
				surroundPoints.push_back(new Point(x, y));

	return surroundPoints;
}

std::vector<Point> Astar::DeletLinePoint(const std::vector<Point> path) { //删除共线的b点
	std::vector<Point> DLPath;
	DLPath.push_back(path[0]);
	for (int i = 0; i < path.size() - 2;i++) {
		Point p1 = path[i]; Point p2 = path[i + 1]; Point p3 = path[i + 2];
		Point AB = { p2.x - p1.x, p2.y - p1.y };
		Point BC = { p3.x - p2.x, p3.y - p2.y };
		if ((AB.x * BC.y - AB.y * BC.x) != 0) {
			//如果不共线
			DLPath.push_back(p2);
		}
	}
	DLPath.push_back(path.back());
	return DLPath;
}

std::vector<Point> Astar::DelNBorPoint(std::vector<Point> path,const std::vector<std::vector<int>> map) //删除ac不经过障碍的b点
{
	int size = path.size();
	int now = 0;//记录当前三点中起点的位置
	while (now != size - 2)
	{
		bool bar = false;
		for (int row = std::min(path[now+2].x, path[now].x); row <= std::max(path[now+2].x, path[now].x); row++)
		{
			for (int col = std::min(path[now + 2].y, path[now].y); col <= std::max(path[now + 2].y, path[now].y); col++) 
			{
				auto dis = GetNearest(Point(row, col), path[now+2], path[now]);
				if (dis <= kCost2 / 2) 
				{
					if (map[row][col] == 1) { bar = true; } //AC的连线确实存在障碍则b点无法舍弃
					//std::cout << dis << "   ";
				}
			}
		}
		if (bar)
		{
			now++;
		}
		else
		{
			path.erase(path.begin()+now+1);
			size--;
		}
		// std::cout << now << "/";
		// std::cout << size << "   ";
	}
	/*
		for (auto& p : Path) {
		bool bar = false;
		for (int row = std::min(p1->x, p->x); row <= std::max(p->x, p1->x); row++) {
			for (int col = std::min(p1->y, p->y); col <= std::max(p->y, p1->y); col++) {
				auto dis = GetNearest(Point(row, col), *p1, *p);
				if (dis <= kCost2 / 2) {
					if (map[row][col] == 1) { bar = true; } //AC的连线确实存在障碍则b点无法舍弃
				//std::cout << dis << "   ";
				}
			}
		}
		if (bar)
		{ 
			DNBPath.push_back(p2); 
		}
		else
		{

		}
		p1 = p2; p2 = p;
	}
	*/
	return path;
}


double Astar::GetDistance(Point A, Point B) { //求两点的欧氏距离
	return sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

double Astar::GetNearest(Point A, Point B, Point C) { //求A到BC线的距离
	double a = GetDistance(A, B);
	double b = GetDistance(A, C);
	double c = GetDistance(B, C);
	if (a * a > b * b + c * c)
		return b;
	if (b * b > a * a + c * c)
		return a;
	double l = (a + b + c) / 2;
	double s = sqrt(l * (l - a) * (l - b) * (l - c));
	return kCost1 * 2 * s / c;
}

