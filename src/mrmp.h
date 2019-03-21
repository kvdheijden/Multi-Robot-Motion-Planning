#ifndef MULTI_ROBOT_MOTION_PLANNING_MRMP_H
#define MULTI_ROBOT_MOTION_PLANNING_MRMP_H

#include "cgal_types.h"

void generate_free_space(const Polygon& W, std::vector<Inset_polygon>& F);

void remove_start_target_configs(
        const std::vector<Inset_polygon>& F,
        const std::vector<Point>& S,
        const std::vector<Point>& T,
        std::vector<Inset_polygon_with_holes>& F_star);

#endif //MULTI_ROBOT_MOTION_PLANNING_MRMP_H
