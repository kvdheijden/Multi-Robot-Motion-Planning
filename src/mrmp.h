#ifndef MULTI_ROBOT_MOTION_PLANNING_MRMP_H
#define MULTI_ROBOT_MOTION_PLANNING_MRMP_H

#include "cgal_types.h"

#include <boost/graph/undirected_graph.hpp>

bool check_inside(const Point &point, const Input_polygon &polygon);

void generate_free_space(const Input_polygon &W, std::vector<Polygon> &F);

General_polygon_set remove_start_target_configs(const Polygon &F,
                                                const std::vector<Point> &S,
                                                const std::vector<Point> &T);

boost::undirected_graph<> generate_motion_graph(const Polygon &F_i,
                                                const std::vector<Polygon_with_holes>& F_star,
                                                const std::vector<Point> &S,
                                                const std::vector<Point> &T);


#endif //MULTI_ROBOT_MOTION_PLANNING_MRMP_H
