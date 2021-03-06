#ifndef MULTI_ROBOT_MOTION_PLANNING_MRMP_H
#define MULTI_ROBOT_MOTION_PLANNING_MRMP_H

#include "cgal_types.h"

#include "ConfigurationSet.h"
#include "MotionGraph.h"
#include "Path.h"

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

std::string get(const std::function<std::string(void *)> &name, void *vd);

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

void generate_free_space(const Workspace_with_holes &W,
                         FreeSpace &F);

void generate_motion_graph(const Polygon &F_i,
                           const ConfigurationSet &configurations,
                           MotionGraph &G_i);

void solve_motion_graph(const MotionGraph &G_i,
                        std::vector<Move> &moves);

void get_shortest_path(const Move &move,
                       const Polygon &f,
                       std::vector<std::reference_wrapper<const Configuration>> &robots,
                       std::vector<Path> &paths);

void simplify_moves(std::vector<Move> &moves);

#endif //MULTI_ROBOT_MOTION_PLANNING_MRMP_H
