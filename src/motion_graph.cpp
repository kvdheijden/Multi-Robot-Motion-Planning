#include "motion_graph.h"

#include <functional>

#include "edge_weight.h"

template<int n>
static Polygon D(const Point &p) {
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

bool check_inside(const Point &point, const Polygon &polygon)
{
    const General_polygon_set gps(polygon);
    const Arrangement &arr = gps.arrangement();

    typedef CGAL::Arr_naive_point_location<Arrangement> Point_location;
    typedef Point_location::Point_2 Point_to_locate;
    typedef Point_location::Result Point_location_result;
    typedef Point_location_result::Type Point_location_result_type;

    Point_location pl(arr);
    const Point_to_locate ptl(point.x(), point.y());

    Point_location_result_type location = pl.locate(ptl);

    typedef Point_location_result::Vertex_const_handle Vertex_const_handle;
    typedef Point_location_result::Halfedge_const_handle Halfedge_const_handle;
    typedef Point_location_result::Face_const_handle Face_const_handle;

    const Face_const_handle *f;

    if ((f = boost::get<Face_const_handle>(&location)))
        return (*f)->has_outer_ccb();
    return static_cast<bool>(boost::get<Halfedge_const_handle>(&location) ||
                             boost::get<Vertex_const_handle>(&location));
}

static bool shared_boundary(const Polygon& p1, const Polygon &p2)
{
    for (auto iter1 = p1.curves_begin(); iter1 != p1.curves_end(); ++iter1) {
        for (auto iter2 = p2.curves_begin(); iter2 != p2.curves_end(); ++iter2) {
            std::vector<CGAL::Object> objects;
            iter1->intersect(*iter2, std::back_inserter(objects));
            for (auto object : objects) {
                Polygon::X_monotone_curve_2 curve;
                if (CGAL::assign(curve, object)) {
                    return true;
                }
            }
        }
    }
    return false;
}

void generate_motion_graph(const Polygon &F_i,
                           const std::vector<Configuration> &S,
                           const std::vector<Configuration> &T,
                           MotionGraph &G_i)
{
    // Reset the motion graph
    G_i.clear();

    // Initialize the vertices (i.e. the start/target configurations)
    for (const Configuration &s : S) {
        if (check_inside(s.getPoint(), F_i)) {
            MotionGraphVertexDescriptor v = boost::add_vertex(G_i);
            G_i[v].configuration = &s;
            G_i[v].color = RED;
            G_i[v].hasPebble = true;
        }
    }
    for (const Configuration &t : T) {
        if (check_inside(t.getPoint(), F_i)) {
            MotionGraphVertexDescriptor v = boost::add_vertex(G_i);
            G_i[v].configuration = &t;
            G_i[v].color = BLUE;
            G_i[v].hasPebble = false;
        }
    }

    General_polygon_set gps(F_i);
    for (const Configuration &s : S) {
        if (check_inside(s.getPoint(), F_i)) {
            gps.difference(D<2>(s.getPoint()));
        }
    }
    for (const Configuration &t : T) {
        if (check_inside(t.getPoint(), F_i)) {
            gps.difference(D<2>(t.getPoint()));
        }
    }
    std::vector<Polygon_with_holes> F_star;
    gps.polygons_with_holes(std::back_inserter(F_star));

    // Add edges between vertices in H_i
    for (const Polygon_with_holes &F_star_i : F_star) {
        std::vector<MotionGraphVertexDescriptor> vertices;
        const Polygon &boundary = F_star_i.outer_boundary();

        typename boost::graph_traits<MotionGraph>::vertex_iterator vi, v_end;
        for (boost::tie(vi, v_end) = boost::vertices(G_i); vi != v_end; ++vi) {
            const MotionGraphVertexDescriptor &vd = *vi;
            const Point &point = G_i[vd].configuration->getPoint();

            if (check_inside(point, boundary) || shared_boundary(D<2>(point), boundary)) {
                vertices.push_back(vd);
            }
        }

        for (int i = 0; i < vertices.size(); i++) {
            for (int j = 0; j < i; j++) {
                boost::add_edge(vertices[i], vertices[j], G_i);
            }
        }
    }
}
