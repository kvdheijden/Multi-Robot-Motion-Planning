#include "motion_graph.h"

#include <functional>

#include <CGAL/Boolean_set_operations_2.h>

typedef CGAL::Arr_naive_point_location<Arrangement> Point_location;
typedef Point_location::Point_2 Point_to_locate;
typedef Point_location::Result Point_location_result;
typedef Point_location_result::Type Point_location_result_type;
typedef Point_location_result::Vertex_const_handle Vertex_const_handle;
typedef Point_location_result::Halfedge_const_handle Halfedge_const_handle;
typedef Point_location_result::Face_const_handle Face_const_handle;

static Point_location_result_type locate(const Point &point, const Arrangement &arr) {
    Point_location pl(arr);
    const Point_to_locate ptl(point.x(), point.y());
    return pl.locate(ptl);
}

static Face_const_handle get_face(const Point &point, const Arrangement &arr) {
    Point_location_result_type loc = locate(point, arr);

    Face_const_handle result;
    const Face_const_handle *f;
    if (!(f = boost::get<Face_const_handle>(&loc))) {
        const Halfedge_const_handle *e;
        if (!(e = boost::get<Halfedge_const_handle>(&loc))) {
            const Vertex_const_handle *v;
            if (!(v = boost::get<Vertex_const_handle>(&loc))) {
                CGAL_assertion(false);
            } else {
                result = (*v)->face();
            }
        } else {
            result = (*e)->face();
        }
    } else {
        result = (*f);
    }
    return result;
}

static bool check_inside(const Point &point, const Arrangement &arr) {
    const Face_const_handle face = get_face(point, arr);
    return face->has_outer_ccb();
}

bool check_inside(const Point &point, const Polygon &polygon) {
    const General_polygon_set gps(polygon);
    const Arrangement &arr = gps.arrangement();
    return check_inside(point, arr);
}

static bool do_intersect(const Polygon &p1, const Polygon &p2) {
    for (auto c1 = p1.curves_begin(); c1 != p1.curves_end(); ++c1) {
        const Polygon::X_monotone_curve_2 &curve1 = *c1;
        for (auto c2 = p2.curves_begin(); c2 != p2.curves_end(); ++c2) {
            const Polygon::X_monotone_curve_2 &curve2 = *c2;
            std::list<CGAL::Object> res;
            curve1.intersect(curve2, std::back_inserter(res));
            if (!res.empty()) {
                return true;
            }
        }
    }

    return CGAL::do_intersect(p1, p2);
}

void generate_motion_graph(const Polygon &F_i,
                           const std::vector<Configuration> &S,
                           const std::vector<Configuration> &T,
                           MotionGraph &G_i) {
    // Reset the motion graph
    G_i.clear();

    // Initialize the vertices (i.e. the start/target configurations)
    General_polygon_set gps(F_i);
    for (const Configuration &s : S) {
        const Arrangement &arr = gps.arrangement();
        const Point &point = s.getPoint();
        if (check_inside(point, arr)) {
            MotionGraphVertexDescriptor v = boost::add_vertex(G_i);
            G_i[v].configuration = &s;
            G_i[v].hasPebble = true;
            const Polygon D2 = D<2>(point);
            gps.difference(D2);
        }
    }
    for (const Configuration &t : T) {
        const Arrangement &arr = gps.arrangement();
        const Point &point = t.getPoint();
        if (check_inside(point, arr)) {
            MotionGraphVertexDescriptor v = boost::add_vertex(G_i);
            G_i[v].configuration = &t;
            G_i[v].hasPebble = false;
            const Polygon D2 = D<2>(point);
            gps.difference(D2);
        }
    }

    // Add edges between vertices
    Traits traits;
    std::vector<Polygon_with_holes> F_star;
    gps.polygons_with_holes(std::back_inserter(F_star));
    for (const Polygon_with_holes &F_star_i : F_star) {
        std::vector<MotionGraphVertexDescriptor> vertices;
        const Polygon &boundary = F_star_i.outer_boundary();

        typename boost::graph_traits<MotionGraph>::vertex_iterator vi, v_end;
        for (boost::tie(vi, v_end) = boost::vertices(G_i); vi != v_end; ++vi) {
            const MotionGraphVertexDescriptor &vd = *vi;
            const Point &point = G_i[vd].configuration->getPoint();

            if (do_intersect(boundary, D<2>(point))) {
                vertices.push_back(vd);
            }
        }

        for (std::size_t i = 0; i < vertices.size(); i++) {
            for (std::size_t j = 0; j < i; j++) {
                boost::add_edge(vertices[i], vertices[j], G_i);
            }
        }
    }
}
