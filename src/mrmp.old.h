#ifndef MULTI_ROBOT_MOTION_PLANNING_MRMP_H
#define MULTI_ROBOT_MOTION_PLANNING_MRMP_H

#include <vector>

#include <CGAL/CORE_algebraic_number_traits.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Arr_conic_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/offset_polygon_2.h>

typedef CGAL::CORE_algebraic_number_traits                          Nt_traits;

typedef Nt_traits::Rational                                         Rational;
typedef CGAL::Cartesian<Rational>                                   Rat_kernel;

typedef Nt_traits::Algebraic                                        Algebraic;
typedef CGAL::Cartesian<Algebraic>                                  Alg_kernel;

typedef CGAL::Arr_conic_traits_2<Rat_kernel, Alg_kernel, Nt_traits> ConicTraits;

typedef CGAL::Polygon_2<Rat_kernel>                                 Polygon;
typedef CGAL::Point_2<Rat_kernel>                                   Point;

typedef CGAL::Gps_traits_2<ConicTraits>                             Gps_traits;
typedef Gps_traits::Polygon_2                                       Inset_polygon;
typedef Gps_traits::Polygon_with_holes_2                            Inset_polygon_with_holes;

void entrypoint(const Polygon& W, const std::vector<Point>& S, const std::vector<Point>& T);

#endif //MULTI_ROBOT_MOTION_PLANNING_MRMP_H
