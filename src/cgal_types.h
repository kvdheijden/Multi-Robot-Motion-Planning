#ifndef MULTI_ROBOT_MOTION_PLANNING_CGAL_TYPES_H
#define MULTI_ROBOT_MOTION_PLANNING_CGAL_TYPES_H

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Algebraic_kernel_for_circles_2_2.h>
#include <CGAL/Circular_kernel_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Gps_circle_segment_traits_2.h>
#include <CGAL/General_polygon_set_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt                         Kernel;
typedef CGAL::Algebraic_kernel_for_circles_2_2<Kernel::FT>                                  Algebraic_kernel;
typedef CGAL::Circular_kernel_2<Kernel, Algebraic_kernel>                                   CircularKernel;
typedef Kernel::Point_2                                                                     Point;
typedef CGAL::Polygon_2<Kernel>                                                             Input_polygon;
typedef Kernel::Circle_2                                                                    Circle;
typedef Kernel::Segment_2                                                                   Segment;

typedef CGAL::Gps_circle_segment_traits_2<Kernel>                                           Traits;
typedef Traits::Polygon_2                                                                   Polygon;
typedef Traits::Polygon_with_holes_2                                                        Polygon_with_holes;

typedef CGAL::General_polygon_set_2<Traits>                                                 General_polygon_set;
typedef General_polygon_set::Arrangement_2                                                  Arrangement;

#endif //MULTI_ROBOT_MOTION_PLANNING_CGAL_TYPES_H
