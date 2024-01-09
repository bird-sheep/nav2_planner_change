#ifndef LIB_ROUTE_H
# define LIB_ROUTE_H
#include <iostream>
#include "Astar.h"
#include "DWA.h"
#include <vector>

struct Pathdata
{
    std::vector<Point> path;
    std::vector<double> yaw;
};

Pathdata findPath( std::vector<std::vector<int>>& map,  Point& start,  Point& end);
#endif