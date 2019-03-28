#include "mrmp.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Arr_batched_point_location.h>
#include <CGAL/offset_polygon_2.h>


static constexpr int POLYLINE_APPROXIMATION = 100;

static void polyline_approximation(const Inset_polygon& f, std::vector<Point>& polyline)
{
    for (auto iter = f.curves_begin(); iter != f.curves_end(); ++iter) {
        std::vector<std::pair<double, double>> polyline_d;

        iter->polyline_approximation(POLYLINE_APPROXIMATION, std::back_inserter(polyline_d));

        for (auto p : polyline_d) {
            polyline.emplace_back(p.first, p.second);
        }
    }
}

template <int n>
static Inset_polygon D(const Point& p)
{
    // Construct the unit disc
    Circle circle(p, Rational(n*n));


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


bool check_inside(const Point& point, const Polygon& polygon)
{
    CGAL::Bounded_side bounded_side = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), point, Kernel());
    return (bounded_side != CGAL::ON_UNBOUNDED_SIDE);
}

bool check_inside(const Point& point, const std::vector<Point>& polyline)
{
    CGAL::Bounded_side bounded_side = CGAL::bounded_side_2(polyline.begin(), polyline.end(), point, Kernel());
    return (bounded_side != CGAL::ON_UNBOUNDED_SIDE);
}


void generate_free_space(const Polygon& W, std::vector<Inset_polygon>& F)
{
    F.clear();
    CGAL::inset_polygon_2(W, 1, ConicTraits(), std::back_inserter(F));
}


int remove_start_target_configs(std::vector<Inset_polygon>& F,
                                const std::vector<Point>& S,
                                const std::vector<Point>& T,
                                std::vector<Inset_polygon_with_holes>& F_star)
{
    for (Inset_polygon& f : F) {
        Inset_polygon_with_holes pgn;

        std::vector<Point> polyline;
        polyline_approximation(f, polyline);

        int s_i = 0;
        for (const Point& s : S) {
            if (check_inside(s, polyline)) {
                pgn.add_hole(D<2>(s));
                s_i++;
            }
        }

        int t_i = 0;
        for (const Point& t : T) {
            if (check_inside(t, polyline)) {
                pgn.add_hole(D<2>(t));
                t_i++;
            }
        }

        // Ensure every free space part has equal no. of start and target configurations
        if (s_i != t_i) {
            return -1;
        }

        std::cout << f.orientation() << std::endl;
        f.reverse_orientation();
        std::cout << f.orientation() << std::endl;

        CGAL::intersection(f, pgn, std::back_inserter(F_star));
    }

    return 0;
}

int generate_motion_graph(const Inset_polygon_with_holes& F_i,
                          const std::vector<Point>& S,
                          const std::vector<Point>& T,
                          boost::undirected_graph<>& G_i)
{
    const Inset_polygon& ccb = F_i.outer_boundary();
    std::vector<Point> polyline;
    polyline_approximation(ccb, polyline);

    for (const Point& s : S) {
        if (check_inside(s, polyline)) {
            // s \in H_i
        } else {
            // if |s - polyline| < 2, then s \in B_i
        }
    }

    return 0;
}