#ifndef MULTI_ROBOT_MOTION_PLANNING_MOTION_GRAPH_H
#define MULTI_ROBOT_MOTION_PLANNING_MOTION_GRAPH_H

#include "cgal_types.h"
#include "Configuration.h"
#include "MotionGraph.h"

template<int n>
Polygon D(const Point &p) {
    Circle circle(p, Kernel::FT(n * n));

    Traits traits;
    Traits::Make_x_monotone_2 make_x_monotone = traits.make_x_monotone_2_object();

    Traits::Curve_2 curve(circle);
    std::vector<CGAL::Object> objects;
    make_x_monotone(curve, std::back_inserter(objects));
    CGAL_assertion(objects.size() == 2);

    Traits::X_monotone_curve_2 arc;
    Polygon result;
    for (const CGAL::Object &obj : objects) {
        CGAL::assign(arc, obj);
        result.push_back(arc);
    }

    return result;
}

bool check_inside(const Point &point, const Polygon &polygon);

void generate_motion_graph(const Polygon &F_i,
                           const std::vector<Configuration> &S,
                           const std::vector<Configuration> &T,
                           MotionGraph &G_i);

#endif //MULTI_ROBOT_MOTION_PLANNING_MOTION_GRAPH_H
