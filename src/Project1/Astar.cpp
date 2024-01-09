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

int Astar::calcG(Point* temp_start, Point* point)  //����G
{
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	int parentG = point->parent == NULL ? 0 : point->parent->G; //����ǳ�ʼ�ڵ㣬���丸�ڵ��ǿ�
	return parentG + extraG;
}

int Astar::calcH(Point* point, Point* end)
{
	//�ü򵥵�ŷ����þ������H�����H�ļ����ǹؼ������кܶ��㷨
	return sqrt((double)(end->x - point->x) * (double)(end->x - point->x) + (double)(end->y - point->y) * (double)(end->y - point->y)) * kCost1;
}

int Astar::calcF(Point* point, Point& startPoint, Point& endPoint)
{
	//�Ķ�һ������k
	auto L = calcH(&startPoint, &endPoint);
	auto l = point->H;
	auto k = 1 + l / L;
	return point->G + k*(point->H);
}

Point* Astar::getLeastFpoint() //��ȡ�����б���Fֵ��С�ĵ�
{
	if (!openList.empty())
	{
		auto resPoint = openList.front(); //ָ���һ��Ԫ��
		for (auto& point : openList)
			if (point->F < resPoint->F)
				resPoint = point;
		return resPoint;
	}
	return NULL;
}

Point* Astar::findPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{
	openList.push_back(new Point(startPoint.x, startPoint.y)); //�������,��������һ���ڵ㣬�������
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
		openList.remove(curPoint); //�ӿ����б���ɾ��
		closeList.push_back(curPoint); //�ŵ��ر��б�
		//1,�ҵ���ǰ��Χ�˸����п���ͨ���ĸ���
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
		for (auto& target : surroundPoints)
		{
			//2,��ĳһ�����ӣ���������ڿ����б��У����뵽�����б����õ�ǰ��Ϊ�丸�ڵ㣬����F G H
			if (!isInList(openList, target))
			{
				target->parent = curPoint;

				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target,startPoint,endPoint);

				openList.push_back(target);
			}
			//3����ĳһ�����ӣ����ڿ����б��У�����Gֵ, �����ԭ���Ĵ�, ��ʲô������, �����������ĸ��ڵ�Ϊ��ǰ��,������G��F
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
				return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
		}
	}

	return NULL;
}

std::list<Point*> Astar::GetPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{
	Point* result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point*> path;
	//����·�������û�ҵ�·�������ؿ�����
	while (result)
	{
		path.push_front(result);
		result = result->parent;
	}
	// �����ʱ�����б���ֹ�ظ�ִ��GetPath���½���쳣
	openList.clear();
	closeList.clear();

	return path;
}

Point* Astar::isInList(const std::list<Point*>& list, const Point* point) const
{
	//�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
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
		|| isInList(closeList, target)) //������뵱ǰ�ڵ��غϡ�������ͼ�����ϰ�������ڹر��б��У�����false
		return false;
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //��б�ǿ���
			return true;
		else
		{
			//б�Խ�Ҫ�ж��Ƿ��ס
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

std::vector<Point> Astar::DeletLinePoint(const std::vector<Point> path) { //ɾ�����ߵ�b��
	std::vector<Point> DLPath;
	DLPath.push_back(path[0]);
	for (int i = 0; i < path.size() - 2;i++) {
		Point p1 = path[i]; Point p2 = path[i + 1]; Point p3 = path[i + 2];
		Point AB = { p2.x - p1.x, p2.y - p1.y };
		Point BC = { p3.x - p2.x, p3.y - p2.y };
		if ((AB.x * BC.y - AB.y * BC.x) != 0) {
			//���������
			DLPath.push_back(p2);
		}
	}
	DLPath.push_back(path.back());
	return DLPath;
}

std::vector<Point> Astar::DelNBorPoint(std::vector<Point> path,const std::vector<std::vector<int>> map) //ɾ��ac�������ϰ���b��
{
	int size = path.size();
	int now = 0;//��¼��ǰ����������λ��
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
					if (map[row][col] == 1) { bar = true; } //AC������ȷʵ�����ϰ���b���޷�����
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
					if (map[row][col] == 1) { bar = true; } //AC������ȷʵ�����ϰ���b���޷�����
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


double Astar::GetDistance(Point A, Point B) { //�������ŷ�Ͼ���
	return sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

double Astar::GetNearest(Point A, Point B, Point C) { //��A��BC�ߵľ���
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

