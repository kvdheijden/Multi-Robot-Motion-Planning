#ifndef MULTI_ROBOT_MOTION_PLANNING_MRMP_H
#define MULTI_ROBOT_MOTION_PLANNING_MRMP_H

#include "cgal_types.h"

#include "ConfigurationSet.h"
#include "FreeSpace.h"
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

bool check_inside(const Point &point,
                  const Workspace &workspace);

bool check_inside(const Point &point,
                  const Polygon &polygon);

bool do_intersect(const Polygon &pgn1,
                  const Polygon::X_monotone_curve_2 &curve);

bool do_intersect(const Polygon &pgn1,
                  const Polygon &pgn2);

void generate_free_space(const Workspace &W,
                         FreeSpace &F);

void generate_motion_graph(const Polygon &F_i,
                           const ConfigurationSet &configurations,
                           MotionGraph &G_i);

void solve_motion_graph(MotionGraph& G_i);

#endif //MULTI_ROBOT_MOTION_PLANNING_MRMP_H
