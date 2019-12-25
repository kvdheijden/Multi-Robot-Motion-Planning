#ifndef MULTI_ROBOT_MOTION_PLANNING_MOTION_GRAPH_H
#define MULTI_ROBOT_MOTION_PLANNING_MOTION_GRAPH_H

#include "cgal_types.h"
#include "Configuration.h"
#include "MotionGraph.h"

bool check_inside(const Point &point, const Polygon &polygon);

void generate_motion_graph(const Polygon &F_i,
                           const std::vector<Configuration> &S,
                           const std::vector<Configuration> &T,
                           MotionGraph &G_i);

#endif //MULTI_ROBOT_MOTION_PLANNING_MOTION_GRAPH_H
