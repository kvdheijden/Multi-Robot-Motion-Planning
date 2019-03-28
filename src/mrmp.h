#ifndef MULTI_ROBOT_MOTION_PLANNING_MRMP_H
#define MULTI_ROBOT_MOTION_PLANNING_MRMP_H

#include "cgal_types.h"

#include <boost/graph/undirected_graph.hpp>

void generate_free_space(const Polygon& W, std::vector<Inset_polygon>& F);

bool check_inside(const Point& point, const Polygon& polygon);
bool check_inside(const Point& point, const std::vector<Point>& polyline);

int remove_start_target_configs(std::vector<Inset_polygon>& F,
                                const std::vector<Point>& S,
                                const std::vector<Point>& T,
                                std::vector<Inset_polygon_with_holes>& F_star);

int generate_motion_graph(const Inset_polygon_with_holes& F_i,
                          const std::vector<Point>& S,
                          const std::vector<Point>& T,
                          boost::undirected_graph<>& G_i);


#endif //MULTI_ROBOT_MOTION_PLANNING_MRMP_H
