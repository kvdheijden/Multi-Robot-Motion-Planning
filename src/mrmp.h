#ifndef MULTI_ROBOT_MOTION_PLANNING_MRMP_H
#define MULTI_ROBOT_MOTION_PLANNING_MRMP_H

#include "cgal_types.h"

#include <boost/graph/undirected_graph.hpp>

bool check_inside(const Input_point& point, const Input_polygon& polygon);

void generate_free_space(const Input_polygon& W, std::vector<Polygon>& F);

General_polygon_set remove_start_target_configs(const Polygon& F,
                                const std::vector<Input_point>& S,
                                const std::vector<Input_point>& T);

//int generate_motion_graph(const Polygon_with_holes& F_i,
//                          const std::vector<Point>& S,
//                          const std::vector<Point>& T,
//                          boost::undirected_graph<>& G_i);


#endif //MULTI_ROBOT_MOTION_PLANNING_MRMP_H
