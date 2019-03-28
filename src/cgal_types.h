#ifndef MULTI_ROBOT_MOTION_PLANNING_CGAL_TYPES_H
#define MULTI_ROBOT_MOTION_PLANNING_CGAL_TYPES_H

#include <CGAL/CORE_algebraic_number_traits.h>
#include <CGAL/Cartesian.h>

#include <CGAL/Arr_conic_traits_2.h>
#include <CGAL/Gps_traits_2.h>

#include <CGAL/Polygon_2.h>

typedef CGAL::CORE_algebraic_number_traits                          Nt_traits;

typedef Nt_traits::Rational                                         Rational;
typedef CGAL::Cartesian<Rational>                                   Kernel;

typedef Nt_traits::Algebraic                                        Algebraic;
typedef CGAL::Cartesian<Algebraic>                                  Alg_kernel;

typedef CGAL::Arr_conic_traits_2<Kernel, Alg_kernel, Nt_traits>     ConicTraits;
typedef CGAL::Gps_traits_2<ConicTraits>                             Gps_traits;

typedef CGAL::Polygon_2<Kernel>                                     Polygon;
typedef CGAL::Point_2<Kernel>                                       Point;
typedef CGAL::Circle_2<Kernel>                                      Circle;
typedef CGAL::Segment_2<Kernel>                                     Segment;

typedef Gps_traits::Polygon_2                                       Inset_polygon;
typedef Gps_traits::Polygon_with_holes_2                            Inset_polygon_with_holes;

#endif //MULTI_ROBOT_MOTION_PLANNING_CGAL_TYPES_H
