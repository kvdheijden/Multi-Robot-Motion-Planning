#include "mrmp.old.h"

#include <CGAL/Gps_traits_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Arr_batched_point_location.h>

typedef CGAL::Gps_traits_2<ConicTraits>     Gps_traits;
typedef Gps_traits::Polygon_2               Inset_polygon;
typedef Gps_traits::Polygon_with_holes_2    Inset_polygon_with_holes;

template <int n>
static Inset_polygon D(const Point& p)
{
    // Construct the unit disc
    Rat_kernel::Circle_2 circle(p, Rational(n));

    // Subdivide the circle into two x-monotone arcs
    Gps_traits traits;
    Gps_traits::Make_x_monotone_2 make_x_monotone = traits.make_x_monotone_2_object();
    Gps_traits::Curve_2 curve(circle);
    std::vector<CGAL::Object> objects;
    make_x_monotone(curve, std::back_inserter(objects));
    CGAL_assertion(objects.size() == 2);

    // Construct the polygon
    Inset_polygon pgn;
    Gps_traits::X_monotone_curve_2 arc;
    std::vector<CGAL::Object>::iterator iter;
    for (iter = objects.begin(); iter != objects.end(); ++iter) {
        CGAL::assign(arc, *iter);
        pgn.push_back(arc);
    }

    return pgn;
}

void create_free_space(const Polygon& W, std::vector<Inset_polygon>& F)
{
    CGAL::inset_polygon_2(W, 1, ConicTraits(), std::back_inserter(F));
}

void remove_start_target_configs(const std::vector<Point>& U, const Inset_polygon& F_i, std::vector<Inset_polygon_with_holes>& F_star)
{
    typedef CGAL::General_polygon_set_2<Gps_traits>                 General_polygon_set;
    typedef CGAL::General_polygon_set_2<Gps_traits>::Arrangement_2  Arrangement;
    typedef CGAL::Arr_point_location_result<Arrangement>            Point_location_result;
    typedef std::pair<Point, Point_location_result::Type>           Query_result;
    Inset_polygon_with_holes pgn;

    General_polygon_set gps(F_i);
    Arrangement& arr = gps.arrangement();
    std::vector<Query_result> query_results;
    CGAL::locate(arr, U.begin(), U.end(), std::back_inserter(query_results));

    for (const Query_result& result : query_results) {
        if (const Arrangement::Face_const_handle* face = boost::get<Arrangement::Face_const_handle>(&(result.second))) {
            // Inside face
            if ((*face)->has_outer_ccb()) {
                pgn.add_hole(D<2>(result.first));
            }
        } else if (boost::get<Arrangement::Halfedge_const_handle>(&(result.second))) {
            // On halfedge
            pgn.add_hole(D<2>(result.first));
        } else if (boost::get<Arrangement::Vertex_const_handle>(&(result.second))) {
            // On vertex
            pgn.add_hole(D<2>(result.first));
        }
    }

    CGAL::intersection(F_i, pgn, std::back_inserter(F_star));
}

void entrypoint(const Polygon& W, const std::vector<Point>& S, const std::vector<Point>& T)
{
    CGAL_precondition(W.is_simple());
    CGAL_precondition(S.size() == T.size());
    std::vector<Point> U;
    U.reserve(S.size() + T.size());
    U.insert( U.end(), S.begin(), S.end() );
    U.insert( U.end(), T.begin(), T.end() );

#ifndef NDEBUG
    std::cerr << "m: " << S.size() << std::endl;
    std::cerr << "n: " << W.size() << std::endl;
    std::cerr << std::endl;

    std::cerr << "S: {" << std::endl;
    for (const Point& s : S) {
        std::cerr << s << std::endl;
    }
    std::cerr << "}" << std::endl;
    std::cerr << std::endl;

    std::cerr << "T: {" << std::endl;
    for (const Point& t : T) {
        std::cerr << t << std::endl;
    }
    std::cerr << "}" << std::endl;
    std::cerr << std::endl;

    std::cerr << "W: " << W << std::endl;
#endif

    std::vector<Inset_polygon> F;
    create_free_space(W, F);

#ifndef NDEBUG
    std::cerr << "F: [" << std::endl;
    for (const Inset_polygon& f : F) {
        std::cerr << f;
    }
    std::cerr << "]" << std::endl;
#endif

    std::vector<Inset_polygon_with_holes> F_star;
    for (const Inset_polygon& f : F) {
        remove_start_target_configs(U, f, F_star);
    }

#ifndef NDEBUG
    std::cerr << "F*: [" << std::endl;
    for (const Inset_polygon_with_holes& F_i : F_star) {
        std::cerr << F_i << std::endl;
    }
    std::cerr << "]" << std::endl;
#endif
}