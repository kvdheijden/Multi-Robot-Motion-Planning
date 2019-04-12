#include "mrmp.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/approximated_offset_2.h>

bool check_inside(const Point &point, const Workspace &workspace) {
    CGAL::Bounded_side bounded_side = CGAL::bounded_side_2(workspace.vertices_begin(),
                                                           workspace.vertices_end(),
                                                           point,
                                                           Kernel());
    return (bounded_side != CGAL::ON_UNBOUNDED_SIDE);
}

bool check_inside(const Point &point, const Polygon &polygon) {
    const General_polygon_set gps(polygon);
    const Arrangement &arr = gps.arrangement();

    typedef CGAL::Arr_naive_point_location <Arrangement> Point_location;
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

    if ((f = boost::get<Face_const_handle>(&location))) {
        return (*f)->has_outer_ccb();
    }
    return static_cast<bool>(boost::get<Halfedge_const_handle>(&location) ||
                             boost::get<Vertex_const_handle>(&location));
}


void generate_free_space(const Workspace &W, FreeSpace &F) {
    Kernel::FT r(1);
    double eps = 0.001; //std::numeric_limits<double>::epsilon();

    F.clear();
    CGAL::approximated_inset_2(W, r, eps, std::back_inserter(F.container()));

    for (Polygon &f : F) {
        if (f.orientation() != CGAL::Orientation::COUNTERCLOCKWISE) {
            f.reverse_orientation();
        }
        CGAL_assertion(f.orientation() == CGAL::Orientation::COUNTERCLOCKWISE);
    }
}


static General_polygon_set remove_start_target_configs(const Polygon &F,
                                                const ConfigurationSet &U) {
    General_polygon_set gps(F);

    int s_i = 0, t_i = 0;
    for (const Configuration &u : U) {
        const Point& p = u.getPoint();
        if (check_inside(p, F)) {
            gps.difference(D<2>(p));
            if (u.isStart()) {
                s_i++;
            } else {
                t_i++;
            }
        }
    }

    // Ensure every free space part has equal no. of start and target configurations
    CGAL_assertion(s_i == t_i);

    return gps;
}

bool do_intersect(const Polygon &pgn1, const Polygon::X_monotone_curve_2 &curve) {
    for (auto iter1 = pgn1.curves_begin(); iter1 != pgn1.curves_end(); ++iter1) {
        std::list<CGAL::Object> objects;
        auto result = iter1->intersect(curve, std::back_inserter(objects));

        if (!objects.empty()) {
            return true;
        }
    }
    return false;
}

bool do_intersect(const Polygon &pgn1, const Polygon &pgn2) {
    for (auto iter = pgn2.curves_begin(); iter != pgn2.curves_end(); ++iter) {
        if (do_intersect(pgn1, *iter)) {
            return true;
        }
    }
    return false;
}

void generate_motion_graph(const Polygon &F_i,
                           const ConfigurationSet &U,
                           MotionGraph &G_i) {
    G_i.clear();

    for (const Configuration& u : U) {
        if (check_inside(u.getPoint(), F_i)) {
            G_i.add_vertex(u);
        }
    }

    General_polygon_set gps = remove_start_target_configs(F_i, U);
    std::vector<Polygon_with_holes> F_star;
    gps.polygons_with_holes(std::back_inserter(F_star));

    // Add edges between vertices in H_i
    for (const Polygon_with_holes &F_star_i : F_star) {
        std::vector<MotionGraphVertexDescriptor> B_i, H_i;
        const Polygon &boundary = F_star_i.outer_boundary();

        for (int i = 0; i < G_i.num_vertices(); i++) {
            const MotionGraphVertexDescriptor v = boost::vertex(i, G_i);
            const Configuration &config = G_i[v];
            const Point& point = config.getPoint();
            if (check_inside(point, boundary)) {
                H_i.push_back(v);
            } else if (check_inside(point, F_i) && do_intersect(D<2>(point), boundary)) {
                B_i.push_back(v);
            }
        }

        for (const MotionGraphVertexDescriptor &b : B_i) {
            for (const MotionGraphVertexDescriptor &h : H_i) {
                G_i.add_edge(b, h);
            }
        }

        for (int i = 0; i < B_i.size(); i++) {
            for (int j = 0; j < i; j++) {
                G_i.add_edge(B_i[i], B_i[j]);
            }
        }
        for (int i = 0; i < H_i.size(); i++) {
            for (int j = 0; j < i; j++) {
                G_i.add_edge(H_i[i], H_i[j]);
            }
        }
    }
}