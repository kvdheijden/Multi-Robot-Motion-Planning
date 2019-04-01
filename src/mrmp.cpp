#include "mrmp.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/approximated_offset_2.h>

//template <int k>
//static Polygon kgon(const Point& p, double radius)
//{
//    std::vector<Point> polyline;
//    double angle = 360.0 / k;
//
//    for (int i = 0; i < k; i++) {
//        double x = radius * sin(i * angle);
//        double y = radius * cos(i * angle);
//        polyline.push_back(Point(x, y));
//    }
//
//    return Polygon(polyline.begin(), polyline.end());
//}
//
template <int n>
static Polygon D(const Input_point& p)
{
    Circle circle(p, Kernel::FT(n*n));

    Traits traits;
    Traits::Make_x_monotone_2 make_x_monotone = traits.make_x_monotone_2_object();

    Traits::Curve_2 curve(circle);
    std::vector<CGAL::Object> objects;
    make_x_monotone(curve, std::back_inserter(objects));
    CGAL_assertion(objects.size() == 2);

    Traits::X_monotone_curve_2 arc;
    Polygon result;
    for (const CGAL::Object& obj : objects) {
        CGAL::assign(arc, obj);
        result.push_back(arc);
    }

    return result;
}

bool check_inside(const Input_point& point, const Input_polygon& polygon)
{
    CGAL::Bounded_side bounded_side = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), point, Kernel());
    return (bounded_side != CGAL::ON_UNBOUNDED_SIDE);
}

bool check_inside(const Input_point& point, const Polygon& polygon)
{
    const General_polygon_set gps(polygon);
    const Arrangement& arr = gps.arrangement();

    typedef CGAL::Arr_naive_point_location<Arrangement> Point_location;
    typedef Point_location::Point_2                     Point_to_locate;
    typedef Point_location::Result                      Point_location_result;
    typedef Point_location_result::Type                 Point_location_result_type;

    Point_location pl(arr);
    const Point_to_locate ptl(point.x(), point.y());

    Point_location_result_type location = pl.locate(ptl);

    typedef Point_location_result::Vertex_const_handle   Vertex_const_handle;
    typedef Point_location_result::Halfedge_const_handle Halfedge_const_handle;
    typedef Point_location_result::Face_const_handle     Face_const_handle;

    const Vertex_const_handle *v;
    const Halfedge_const_handle *e;
    const Face_const_handle *f;

    if ((f = boost::get<Face_const_handle>(&location))) {
        return (*f)->has_outer_ccb();
    }
    return static_cast<bool>(boost::get<Halfedge_const_handle>(&location) || boost::get<Vertex_const_handle>(&location));
}


void generate_free_space(const Input_polygon& W, std::vector<Polygon>& F)
{
    Kernel::FT r(1);
    double eps = 0.001; //std::numeric_limits<double>::epsilon();

    F.clear();
    CGAL::approximated_inset_2(W, r, eps, std::back_inserter(F));

    for (Polygon& f : F) {
        if (f.orientation() != CGAL::Orientation::COUNTERCLOCKWISE) {
            f.reverse_orientation();
        }
        CGAL_assertion(f.orientation() == CGAL::Orientation::COUNTERCLOCKWISE);
    }

}


General_polygon_set remove_start_target_configs(const Polygon& F,
                                                const std::vector<Input_point>& S,
                                                const std::vector<Input_point>& T)
{
    General_polygon_set gps(F);

    int s_i = 0;
    for (const Input_point& s : S) {
        if (check_inside(s, F)) {
            gps.difference(D<2>(s));
            s_i++;
        }
    }

    int t_i = 0;
    for (const Input_point& t : T) {
        if (check_inside(t, F)) {
            gps.difference(D<2>(t));
            t_i++;
        }
    }

    // Ensure every free space part has equal no. of start and target configurations
    CGAL_assertion(s_i == t_i);

    return gps;
}

//int generate_motion_graph(const Polygon_with_holes& F_i,
//                          const std::vector<Point>& S,
//                          const std::vector<Point>& T,
//                          boost::undirected_graph<>& G_i)
//{
//    const Polygon& ccb = F_i.outer_boundary();
//
//    for (const Point& s : S) {
//        if (check_inside(s, ccb)) {
//            // s \in H_i
//        } else {
//            // if |s - polyline| < 2, then s \in B_i
//        }
//    }
//
//    return 0;
//}