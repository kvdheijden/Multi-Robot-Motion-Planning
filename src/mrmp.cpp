#include "mrmp.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/approximated_offset_2.h>
#include <CGAL/Triangular_expansion_visibility_2.h>

#include <CGAL/graph_traits_Arrangement_2.h>
#include <CGAL/Arr_vertex_index_map.h>

#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_utility.hpp>

#include "IntervisibilityGraph.h"
#include "interference_forest.h"

std::string get(const std::function<std::string(void *)> &name,
                void *vd) {
    return name(vd);
}

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

    if ((f = boost::get<Face_const_handle>(&location))) {
        return (*f)->has_outer_ccb();
    }
    return static_cast<bool>(boost::get<Halfedge_const_handle>(&location) ||
                             boost::get<Vertex_const_handle>(&location));
}


void generate_free_space(const Workspace &W, FreeSpace &F) {
    double eps = 0.0001; //std::numeric_limits<double>::epsilon();
    Kernel::FT r(1 - eps);

    F.clear();
    CGAL::approximated_inset_2(W, r, eps, std::back_inserter(F));

    for (Polygon &f : F) {
        if (f.orientation() != CGAL::Orientation::COUNTERCLOCKWISE) {
            f.reverse_orientation();
        }
        CGAL_assertion(f.orientation() == CGAL::Orientation::COUNTERCLOCKWISE);
    }
}

void generate_free_space(const Workspace_with_holes &W, FreeSpace &F) {
    generate_free_space(W.outer_boundary(), F);

    Kernel::FT r(1);
    double eps = 0.001; //std::numeric_limits<double>::epsilon();

    for (auto iter = W.holes_begin(); iter != W.holes_end(); ++iter) {
        Polygon_with_holes h = CGAL::approximated_offset_2(*iter, r, eps);
        /* TODO */
    }

    for (Polygon &f : F) {
        if (f.orientation() != CGAL::Orientation::COUNTERCLOCKWISE) {
            f.reverse_orientation();
        }
        CGAL_assertion(f.orientation() == CGAL::Orientation::COUNTERCLOCKWISE);
    }
}


static void remove_start_target_configs(const Polygon &F,
                                        const ConfigurationSet &U,
                                        General_polygon_set &gps) {
    int s_i = 0, t_i = 0;
    for (const Configuration &u : U) {
        const Point &p = u.getPoint();
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
}

bool do_intersect(const Polygon::X_monotone_curve_2 &curve1, const Polygon::X_monotone_curve_2 &curve2) {
    std::list<CGAL::Object> objects;
    curve1.intersect(curve2, std::back_inserter(objects));
    return !objects.empty();
}

bool do_intersect(const Polygon &pgn1, const Polygon::X_monotone_curve_2 &curve) {
    for (auto iter = pgn1.curves_begin(); iter != pgn1.curves_end(); ++iter) {
        if (do_intersect(*iter, curve)) {
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

bool shared_boundary(const Polygon &p1, const Polygon &p2) {
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
                           const ConfigurationSet &U,
                           MotionGraph &G_i) {
    G_i.clear();

    for (const Configuration &u : U) {
        if (check_inside(u.getPoint(), F_i)) {
            MotionGraphVertexDescriptor v = boost::add_vertex(G_i);
            G_i[v].configuration = &u;
            G_i[v].hasPebble = u.isStart();
        }
    }

    General_polygon_set gps(F_i);
    remove_start_target_configs(F_i, U, gps);
    std::vector<Polygon_with_holes> F_star;
    gps.polygons_with_holes(std::back_inserter(F_star));

    // Add edges between vertices in H_i
    for (const Polygon_with_holes &F_star_i : F_star) {
        std::vector<MotionGraphVertexDescriptor> vertices;
        const Polygon &boundary = F_star_i.outer_boundary();

        typename boost::graph_traits<MotionGraph>::vertex_iterator vi, v_end;
        for (boost::tie(vi, v_end) = boost::vertices(G_i);
             vi != v_end;
             ++vi) {
            const MotionGraphVertexDescriptor &vd = *vi;
            const MotionGraphVertex &v = G_i[vd];
            const Configuration &configuration = *v.configuration;
            const Point &point = configuration.getPoint();

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

class ST_Weight_Map {
public:
    typedef Kernel::RT value_type;
    typedef const value_type &reference;
    typedef MotionGraphEdgeDescriptor key_type;
    typedef boost::readable_property_map_tag category;

    friend value_type get(ST_Weight_Map me, key_type key) {
        return me[key];
    }

    value_type operator[](key_type key) {
        const MotionGraphVertexDescriptor &sd = boost::source(key, graph);
        const MotionGraphVertexDescriptor &td = boost::target(key, graph);

        const MotionGraphVertex &s = graph[sd];
        const MotionGraphVertex &t = graph[td];

        const Point &sp = s.configuration->getPoint();
        const Point &tp = t.configuration->getPoint();

        return CGAL::sqrt(CGAL::squared_distance(sp, tp));
    }

    explicit ST_Weight_Map(const MotionGraph &graph) : graph(graph) {}

private:
    const MotionGraph &graph;
};

class ST_Vertex_Index_Map {
public:
    typedef int value_type;
    typedef const value_type &reference;
    typedef MotionGraphVertexDescriptor key_type;
    typedef boost::readable_property_map_tag category;

    friend value_type get(ST_Vertex_Index_Map me, key_type key) {
        return me[key];
    }

    value_type operator[](key_type key) {
        boost::graph_traits<MotionGraph>::vertex_iterator g_i, g_end;

        int result = 0;
        for (boost::tie(g_i, g_end) = boost::vertices(graph);
             g_i != g_end;
             ++g_i) {
            const MotionGraphVertexDescriptor &vd = *g_i;
            if (graph[vd] == graph[key]) {
                return result;
            }
            result++;
        }
        return -1;
    }

    explicit ST_Vertex_Index_Map(const MotionGraph &graph) : graph(graph) {}

private:
    const MotionGraph &graph;
};

void solve_motion_graph(const MotionGraph &G_i,
                        std::vector<Move> &moves) {
    std::vector<MotionGraphEdgeDescriptor> spanning_tree;
    ST_Weight_Map weight_map(G_i);
    ST_Vertex_Index_Map vertex_index_map(G_i);
    boost::kruskal_minimum_spanning_tree(G_i, std::back_inserter(spanning_tree),
                                         boost::vertex_index_map(vertex_index_map).weight_map(weight_map));

    // TODO: Optimize
    MotionGraph T_g;
    boost::graph_traits<MotionGraph>::vertex_iterator G_i_v, G_i_v_end;
    for (boost::tie(G_i_v, G_i_v_end) = boost::vertices(G_i);
         G_i_v != G_i_v_end;
         ++G_i_v) {
        MotionGraphVertexDescriptor vd = boost::add_vertex(T_g);
        T_g[vd] = G_i[*G_i_v];
    }

    for (const MotionGraphEdgeDescriptor &e : spanning_tree) {
        MotionGraphVertexDescriptor G_i_s = boost::source(e, G_i);
        MotionGraphVertexDescriptor G_i_t = boost::target(e, G_i);

        MotionGraphVertexDescriptor s, t;
        for (boost::tie(G_i_v, G_i_v_end) = boost::vertices(T_g);
             G_i_v != G_i_v_end;
             ++G_i_v) {
            if (G_i[G_i_s] == (T_g[*G_i_v])) {
                s = *G_i_v;
            }
            if (G_i[G_i_t] == (T_g[*G_i_v])) {
                t = *G_i_v;
            }
        }

        T_g.add_edge(s, t);
    }

    while (boost::num_vertices(T_g)) {
        std::cerr << "T_g:" << std::endl;
        boost::print_graph(T_g, [&](MotionGraphVertexDescriptor vd) {
            const Configuration *c = T_g[vd].configuration;
            return (c->isStart() ? "s" : "t") + std::to_string(c->getIndex());
        }, std::cerr);
        std::cerr << std::endl;

        // Find a leave (preferable target)
        boost::graph_traits<MotionGraph>::vertex_iterator v_begin, v_j, v_end;
        boost::tie(v_begin, v_end) = boost::vertices(T_g);
        v_j = v_end;
        for (auto v_i = v_begin; v_i != v_end; ++v_i) {
            if (boost::degree(*v_i, T_g) <= 1) {
                v_j = v_i;
                if (!T_g[*v_i].configuration->isStart()) {
                    break;
                }
            }
        }

        CGAL_assertion(v_j != v_end);

        MotionGraphVertexDescriptor &v = *v_j;
        std::cerr << "Current leave: "
                  << (T_g[v].configuration->isStart() ? "s" : "t") + std::to_string(T_g[v].configuration->getIndex())
                  << std::endl;

        bool processed = pebble_game_process(T_g, v, moves);

        CGAL_assertion(T_g[v].hasPebble != T_g[v].configuration->isStart());

        if (processed) {
            // Remove vertex from T_g
            boost::clear_vertex(v, T_g);
            boost::remove_vertex(v, T_g);
        }

        std::cerr << std::endl;
    }
}

class edge_weight_map {
public:
    typedef Kernel::RT value_type;
    typedef Arrangement::Halfedge_handle key_type;
    typedef const value_type &reference;
    typedef boost::readable_property_map_tag category;

    friend value_type get(edge_weight_map me, key_type key) {
        if (me._map.find(key) == me._map.end()) {
            return value_type(INFINITY);
        }
        return me._map[key];
    }

    friend void put(edge_weight_map me, key_type key, value_type value) {
        if (me._map.find(key) == me._map.end()) {
            me._map.insert(std::make_pair(key, value));
        } else {
            me._map[key] = value;
        }
    }

    reference operator[](key_type key) {
        return _map[key];
    }

private:
    std::map<key_type, value_type> _map;
};

Line get_line(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q) {
    Kernel::RT Px = p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root());
    Kernel::RT Py = p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root());
    Kernel::RT Qx = q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root());
    Kernel::RT Qy = q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root());
    return Line(Point(Px, Py), Point(Qx, Qy));
}

Arrangement::X_monotone_curve_2 get_edge(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q) {
    Line l = get_line(p, q);
    return Arrangement::X_monotone_curve_2(l, p, q);
}

Kernel::RT get_distance(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q) {
    Kernel::RT Px = p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root());
    Kernel::RT Py = p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root());
    Kernel::RT Qx = q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root());
    Kernel::RT Qy = q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root());
    auto diff_x = Px - Qx;
    auto diff_y = Py - Qy;
    return CGAL::sqrt(diff_x * diff_x + diff_y * diff_y);
}

Kernel::RT get_distance(const Arrangement::X_monotone_curve_2 &curve) {
    if (curve.is_linear()) {
        return get_distance(curve.source(), curve.target());
    }

    std::vector<std::pair<double, double>> polyline;
    curve.approximate(std::back_inserter(polyline), 10);
    Kernel::RT r(0);
    for (int i = 1; i < polyline.size(); i++) {
        Arrangement::Point_2 p(polyline[i - 1].first, polyline[i - 1].second);
        Arrangement::Point_2 q(polyline[i].first, polyline[i].second);
        r += get_distance(p, q);
    }
    return r;
}

void get_tangent_points_on_circle(const Circle &circle, const Arrangement::Point_2 &p,
                                  Arrangement::Point_2 &s, Arrangement::Point_2 &t) {
    /// See: https://math.stackexchange.com/questions/543496/how-to-find-the-equation-of-a-line-tangent-a-circle-and-a-given-point-outside-of#answer-3190374
    // TODO - This function takes way too long

    const Point &c = circle.center();
    Kernel::RT Cx = c.x();
    Kernel::RT Cy = c.y();
    Kernel::RT r = CGAL::sqrt(circle.squared_radius());
    Kernel::RT Px = p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root());
    Kernel::RT Py = p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root());

    auto dx = Px - Cx;
    auto dy = Py - Cy;

    auto d = CGAL::sqrt(dx * dx + dy * dy);

    auto rho = r / d;
    auto ad = rho * rho;
    auto bd = rho * CGAL::sqrt(1 - ad);
    auto T1x = Cx + ad * dx - bd * dy;
    auto T1y = Cy + ad * dy + bd * dx;
    auto T2x = Cx + ad * dx + bd * dy;
    auto T2y = Cy + ad * dy - bd * dx;

    s.set(T1x, T1y);
    t.set(T2x, T2y);
}

void get_tangent_point(const Arrangement::X_monotone_curve_2 &curve, const Arrangement::Point_2 &p,
                       std::vector<std::pair<Arrangement::Point_2, Arrangement::Point_2>> &points) {
    CGAL_assertion(curve.is_circular());
    const Circle &circle = curve.supporting_circle();

    Arrangement::Point_2 T[2];
    get_tangent_points_on_circle(circle, p, T[0], T[1]);

    for (Arrangement::Point_2 &t : T) {
        if (curve.point_position(t) == CGAL::EQUAL) {
            points.emplace_back(p, t);
        }
    }
}

void get_tangent_points(const Arrangement::X_monotone_curve_2 &curve1, const Arrangement::X_monotone_curve_2 &curve2,
                        std::vector<std::pair<Arrangement::Point_2, Arrangement::Point_2>> &points) {
    CGAL_assertion(curve1.is_circular());
    CGAL_assertion(curve2.is_circular());

    const Circle &circle1 = curve1.supporting_circle();
    const Circle &circle2 = curve2.supporting_circle();

    Kernel::RT r1 = CGAL::sqrt(circle1.squared_radius());
    Kernel::RT r2 = CGAL::sqrt(circle2.squared_radius());

    const Point &c1 = circle1.center();
    const Point &c2 = circle2.center();

    Kernel::RT C1x = c1.x();
    Kernel::RT C1y = c1.y();
    Kernel::RT C2x = c2.x();
    Kernel::RT C2y = c2.y();

    const Arrangement::Point_2 C1(C1x, C1y);
    const Arrangement::Point_2 C2(C2x, C2y);

    Arrangement::Point_2 T1, T2;
    Point p[8], q[8];

    if (r1 >= r2) {
        // Outer tangents
        const Circle circle3(circle1.center(), CGAL::square(r1 - r2));
        get_tangent_points_on_circle(circle3, C2, T1, T2);
        Kernel::RT T1x = T1.x().a0() + T1.x().a1() * CGAL::sqrt(T1.x().root());
        Kernel::RT T1y = T1.y().a0() + T1.y().a1() * CGAL::sqrt(T1.y().root());
        Kernel::RT T2x = T2.x().a0() + T2.x().a1() * CGAL::sqrt(T2.x().root());
        Kernel::RT T2y = T2.y().a0() + T2.y().a1() * CGAL::sqrt(T2.y().root());
        Point t1(T1x, T1y);
        Point t2(T2x, T2y);

        Line l = get_line(C2, T1);
        Line l2 = l.perpendicular(circle2.center());
        // p \gets translate C2 along l2 (positive)
        // q \gets translate T1 along l2 (positive)
        Vector v2 = l2.to_vector() * r2;
        CGAL::Aff_transformation_2<Kernel> transformation(CGAL::TRANSLATION, v2);
        p[0] = transformation.transform(c2);
        q[0] = transformation.transform(t1);

        // p \gets translate C2 along l2 (negative)
        // q \gets translate T1 along l2 (negative)
        v2 = l2.to_vector() * -r2;
        transformation = CGAL::Aff_transformation_2<Kernel>(CGAL::TRANSLATION, v2);
        p[1] = transformation.transform(c2);
        q[1] = transformation.transform(t1);

        l = get_line(C2, T2);
        l2 = l.perpendicular(circle2.center());
        // p \gets translate C2 along l2 (positive)
        // q \gets translate T2 along l2 (positive)
        v2 = l2.to_vector() * r2;
        transformation = CGAL::Aff_transformation_2<Kernel>(CGAL::TRANSLATION, v2);
        p[2] = transformation.transform(c2);
        q[2] = transformation.transform(t2);

        // p \gets translate C2 along l2 (negative)
        // q \gets translate T2 along l2 (negative)
        v2 = l2.to_vector() * -r2;
        transformation = CGAL::Aff_transformation_2<Kernel>(CGAL::TRANSLATION, v2);
        p[3] = transformation.transform(c2);
        q[3] = transformation.transform(t2);

        // Inner tangents
        const Circle circle4(circle1.center(), CGAL::square(r1 + r2));
        get_tangent_points_on_circle(circle4, C2, T1, T2);
        T1x = T1.x().a0() + T1.x().a1() * CGAL::sqrt(T1.x().root());
        T1y = T1.y().a0() + T1.y().a1() * CGAL::sqrt(T1.y().root());
        T2x = T2.x().a0() + T2.x().a1() * CGAL::sqrt(T2.x().root());
        T2y = T2.y().a0() + T2.y().a1() * CGAL::sqrt(T2.y().root());
        t1 = Point(T1x, T1y);
        t2 = Point(T2x, T2y);

        l = get_line(C2, T1);
        l2 = l.perpendicular(circle2.center());
        // p \gets translate C2 along l2 (positive)
        // q \gets translate T1 along l2 (positive)
        v2 = l2.to_vector() * r2;
        transformation = CGAL::Aff_transformation_2<Kernel>(CGAL::TRANSLATION, v2);
        p[4] = transformation.transform(c2);
        q[4] = transformation.transform(t1);

        // p \gets translate C2 along l2 (negative)
        // q \gets translate T1 along l2 (negative)
        v2 = l2.to_vector() * -r2;
        transformation = CGAL::Aff_transformation_2<Kernel>(CGAL::TRANSLATION, v2);
        p[5] = transformation.transform(c2);
        q[5] = transformation.transform(t1);

        l = get_line(C2, T2);
        l2 = l.perpendicular(circle2.center());
        // p \gets translate C2 along l2 (positive)
        // q \gets translate T2 along l2 (positive)
        v2 = l2.to_vector() * r2;
        transformation = CGAL::Aff_transformation_2<Kernel>(CGAL::TRANSLATION, v2);
        p[6] = transformation.transform(c2);
        q[6] = transformation.transform(t2);

        // p \gets translate C2 along l2 (negative)
        // q \gets translate T2 along l2 (negative)
        v2 = l2.to_vector() * -r2;
        transformation = CGAL::Aff_transformation_2<Kernel>(CGAL::TRANSLATION, v2);
        p[7] = transformation.transform(c2);
        q[7] = transformation.transform(t2);

        for (int i = 0; i < 8; i++) {
            Arrangement::Point_2 P(p[i].x(), p[i].y());
            Arrangement::Point_2 Q(q[i].x(), q[i].y());
            // Determine if (p, q) is valid tangent segment
            if (curve1.point_position(Q) == CGAL::EQUAL && curve2.point_position(P) == CGAL::EQUAL) {
                points.emplace_back(P, Q);
            }
        }
    } else {
        get_tangent_points(curve2, curve1, points);
    }
}

IntervisibilityGraphVertexDescriptor add_vertex(IntervisibilityGraph &graph, const Arrangement::Point_2 *p) {
    IntervisibilityGraphVertexDescriptor d = boost::add_vertex(graph);
    graph[d].p = p;
    return d;
}

IntervisibilityGraphEdgeDescriptor add_edge_linear(IntervisibilityGraph &graph,
                                                   const IntervisibilityGraphVertexDescriptor &vdp,
                                                   const IntervisibilityGraphVertexDescriptor &vdq) {
    IntervisibilityGraphEdgeDescriptor e;
    bool b;

    IntervisibilityGraphVertex &vp = graph[vdp];
    IntervisibilityGraphVertex &vq = graph[vdq];

    const Arrangement::Point_2 &p = *(vp.p);
    const Arrangement::Point_2 &q = *(vq.p);

    Kernel::RT px = (p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root()));
    Kernel::RT py = (p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root()));
    Kernel::RT qx = (q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root()));
    Kernel::RT qy = (q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root()));

    Kernel::RT x = px - qx;
    Kernel::RT y = py - qy;

    boost::tie(e, b) = boost::add_edge(vdp, vdq, (CGAL::sqrt(x * x + y * y)), graph);

    return e;
}

IntervisibilityGraphEdgeDescriptor add_edge_circular(IntervisibilityGraph &graph,
                                                     const IntervisibilityGraphVertexDescriptor &vp,
                                                     const IntervisibilityGraphVertexDescriptor &vq,
                                                     const Circle &circle) {
    IntervisibilityGraphEdgeDescriptor e;
    bool b;

    const Arrangement::Point_2 &p = *(graph[vp].p);
    const Arrangement::Point_2 &q = *(graph[vq].p);

#if defined(CGAL_EXACT_PREDICATES_EXACT_CONSTRUCTIONS_KERNEL_WITH_SQRT_H)
    double px = (p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root())).doubleValue();
    double py = (p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root())).doubleValue();
    double qx = (q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root())).doubleValue();
    double qy = (q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root())).doubleValue();
    double cx = circle.center().x().doubleValue();
    double cy = circle.center().y().doubleValue();
#elif defined(CGAL_EXACT_PREDICATES_INEXACT_CONSTRUCTIONS_KERNEL_H)
    double px = (p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root()));
    double py = (p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root()));
    double qx = (q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root()));
    double qy = (q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root()));
    double cx = circle.center().x();
    double cy = circle.center().y();
#elif defined (CGAL_SIMPLE_CARTESIAN_H)
    double px = (p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root()));
    double py = (p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root()));
    double qx = (q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root()));
    double qy = (q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root()));
    double cx = circle.center().x();
    double cy = circle.center().y();
#endif
    double angle = abs(atan2(qy - cy, qx - cx) - atan2(py - cy, px - cx));

    boost::tie(e, b) = boost::add_edge(vp, vq, Kernel::RT(angle * CGAL::sqrt(circle.squared_radius())), graph);
    return e;
}

//void add_edge_linear(IntervisibilityGraph &graph,
//                     const Arrangement::Point_2 &p,
//                     const Arrangement::Point_2 &q) {
//    IntervisibilityGraphVertexDescriptor d1 = add_vertex(graph, p);
//
//    IntervisibilityGraphVertexDescriptor d2 = add_vertex(graph, q);
//
//    IntervisibilityGraphEdgeDescriptor e;
//    bool b;
//    boost::tie(e, b) = boost::add_edge(d1, d2, graph);
//    if (b) {
//        graph[e].edge_weight = get_distance(p, q);
//    }
//}
//
//void add_edge_circular(IntervisibilityGraph &graph,
//                       const Arrangement::Point_2 &p,
//                       const Arrangement::Point_2 &q,
//                       const Circle &circle) {
//    IntervisibilityGraphVertexDescriptor d1 = add_vertex(graph, p);
//
//    IntervisibilityGraphVertexDescriptor d2 = add_vertex(graph, q);
//
//    IntervisibilityGraphEdgeDescriptor e;
//    bool b;
//    boost::tie(e, b) = boost::add_edge(d1, d2, graph);
//    if (b) {
//#if defined(CGAL_EXACT_PREDICATES_EXACT_CONSTRUCTIONS_KERNEL_WITH_SQRT_H)
//        double px = (p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root())).doubleValue();
//        double py = (p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root())).doubleValue();
//        double qx = (q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root())).doubleValue();
//        double qy = (q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root())).doubleValue();
//        double cx = circle.center().x().doubleValue();
//        double cy = circle.center().y().doubleValue();
//        double angle = atan2(qy - cy, qx - cx) - atan2(py - cy, px - cx);
//#elif defined(CGAL_EXACT_PREDICATES_INEXACT_CONSTRUCTIONS_KERNEL_H)
//        double px = (p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root()));
//        double py = (p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root()));
//        double qx = (q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root()));
//        double qy = (q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root()));
//        double cx = circle.center().x();
//        double cy = circle.center().y();
//        double angle = atan2(qy - cy, qx - cx) - atan2(py - cy, px - cx);
//#elif defined (CGAL_SIMPLE_CARTESIAN_H)
//        double px = (p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root()));
//        double py = (p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root()));
//        double qx = (q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root()));
//        double qy = (q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root()));
//        double cx = circle.center().x();
//        double cy = circle.center().y();
//        double angle = atan2(qy - cy, qx - cx) - atan2(py - cy, px - cx);
//#endif
//        graph[e].edge_weight = Kernel::RT(angle * CGAL::sqrt(circle.squared_radius()));
//    }
//}
//
//IntervisibilityGraphVertexDescriptor process_point(IntervisibilityGraph &graph,
//                                                   const Polygon_with_holes &pgn,
//                                                   const Arrangement::Point_2 &p) {
//    IntervisibilityGraphVertexDescriptor result = add_vertex(graph, p);
//
//    for (auto target_curve : Polygon_with_holes_curve_iterator(pgn)) {
//        const Arrangement::Point_2 &r = target_curve.source();
//        if (p != r) {
//            Arrangement::X_monotone_curve_2 pr = get_edge(p, r);
//            bool intersects = false;
//            for (auto intersecting_curve : Polygon_with_holes_curve_iterator(pgn)) {
//                if (do_intersect(intersecting_curve, pr)) {
//                    intersects = true;
//                    break;
//                }
//            }
//            if (!intersects) {
//                // Add edge pr
//                add_edge_linear(graph, p, r);
//            }
//        }
//
//
//        const Arrangement::Point_2 &s = target_curve.target();
//        if (p != s) {
//            Arrangement::X_monotone_curve_2 ps = get_edge(p, s);
//            bool intersects = false;
//            for (auto intersecting_curve : Polygon_with_holes_curve_iterator(pgn)) {
//                if (do_intersect(intersecting_curve, ps)) {
//                    intersects = true;
//                    break;
//                }
//            }
//            if (!intersects) {
//                // Add edge ps
//                add_edge_linear(graph, p, s);
//            }
//        }
//
//    }
//
//    return result;
//}
//
//class Intervisibility_vertex_index_map {
//public:
//    typedef int value_type;
//    typedef const value_type &reference;
//    typedef IntervisibilityGraphVertexDescriptor key_type;
//    typedef boost::readable_property_map_tag category;
//
//    friend value_type get(Intervisibility_vertex_index_map me, key_type key) {
//        auto iter = std::find(me.vertices.begin(), me.vertices.end(), key);
//        return iter - me.vertices.begin();
//    }
//
//    explicit Intervisibility_vertex_index_map(const std::vector<IntervisibilityGraphVertexDescriptor> &vertices)
//            : vertices(vertices) {}
//
//private:
//    const std::vector<IntervisibilityGraphVertexDescriptor> &vertices;
//};
//
//class Intervisibility_weight_map {
//public:
//    typedef Kernel::RT value_type;
//    typedef const value_type &reference;
//    typedef IntervisibilityGraphEdgeDescriptor key_type;
//    typedef boost::readable_property_map_tag category;
//
//    friend value_type get(Intervisibility_weight_map me, key_type key) {
//        return me.graph[key].edge_weight;
//    }
//
//    explicit Intervisibility_weight_map(const IntervisibilityGraph &graph) : graph(graph) {}
//
//private:
//    const IntervisibilityGraph &graph;
//};

class Intervisibility_predecessor_map {
public:
    typedef IntervisibilityGraphVertexDescriptor key_type;
    typedef IntervisibilityGraphVertexDescriptor value_type;
    typedef value_type &reference;
    typedef boost::read_write_property_map_tag category;

    friend value_type get(Intervisibility_predecessor_map me, key_type key) {
        std::cout << "Get";
        if (me._map.find(key) == me._map.end()) {
            return boost::graph_traits<IntervisibilityGraph>::null_vertex();
        }
        return me._map.at(key);
    }

    friend void put(Intervisibility_predecessor_map me, key_type key, value_type value) {
        std::cout << "Put";
        if (me._map.find(key) == me._map.end()) {
            me._map.insert(std::make_pair(key, value));
        } else {
            me._map.at(key) = value;
        }
    }

    reference operator[](key_type key) {
        std::cout << "[]";
        return _map[key];
    }

private:
    std::map<key_type, value_type> _map;
};

CircularKernel::Line_arc_2 get_line_arc(const Arrangement::Point_2 &source, const Arrangement::Point_2 &target) {
    Kernel::RT sourceX = source.x().a0() + source.x().a1() * CGAL::sqrt(source.x().root());
    Kernel::RT sourceY = source.y().a0() + source.y().a1() * CGAL::sqrt(source.y().root());
    Kernel::RT targetX = target.x().a0() + target.x().a1() * CGAL::sqrt(target.x().root());
    Kernel::RT targetY = target.y().a0() + target.y().a1() * CGAL::sqrt(target.y().root());

    CircularKernel::Point_2 p1(sourceX, sourceY);
    CircularKernel::Point_2 p2(targetX, targetY);

    return CircularKernel::Line_arc_2(p1, p2);
}

CircularKernel::Circular_arc_2 get_circular_arc(const Arrangement::X_monotone_curve_2 &curve) {
    CGAL_assertion(curve.is_circular());

    const Circle &supporting_circle = curve.supporting_circle();
    const Point &center = supporting_circle.center();
    const CircularKernel::FT &squared_radius = supporting_circle.squared_radius();

    const Arrangement::Point_2 &source = curve.source();
    const Arrangement::Point_2 &target = curve.target();

    Kernel::FT sourceX = source.x().a0() + source.x().a1() * CGAL::sqrt(source.x().root());
    Kernel::FT sourceY = source.y().a0() + source.y().a1() * CGAL::sqrt(source.y().root());
    Kernel::FT targetX = target.x().a0() + target.x().a1() * CGAL::sqrt(target.x().root());
    Kernel::FT targetY = target.y().a0() + target.y().a1() * CGAL::sqrt(target.y().root());

    CircularKernel::Circle_2 c(CircularKernel::Point_2(center.x(), center.y()), squared_radius);
    CircularKernel::Circular_arc_point_2 p(CircularKernel::Point_2(sourceX, sourceY));
    CircularKernel::Circular_arc_point_2 q(CircularKernel::Point_2(targetX, targetY));

    return CircularKernel::Circular_arc_2(c, p, q);
}

bool intersect_inclusive(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q,
                         const Arrangement::X_monotone_curve_2 &curve2) {
    if (p.equals(curve2.source()) ||
        p.equals(curve2.target()) ||
        q.equals(curve2.source()) ||
        q.equals(curve2.target())) {
        return false;
    }

    if (curve2.is_linear()) {
        return CGAL::do_intersect(get_line_arc(p, q), get_line_arc(curve2.source(), curve2.target()));
    } else {
        return CGAL::do_intersect(get_line_arc(p, q), get_circular_arc(curve2));
    }
}

bool is_visible(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q, const Polygon_with_holes &pgn) {
    for (auto iter = pgn.outer_boundary().curves_begin(); iter != pgn.outer_boundary().curves_end(); ++iter) {
        if (intersect_inclusive(p, q, *iter)) {
            return false;
        }
    }
    for (auto h_iter = pgn.holes_begin(); h_iter != pgn.holes_end(); ++h_iter) {
        for (auto iter = h_iter->curves_begin(); iter != h_iter->curves_end(); ++iter) {
            if (intersect_inclusive(p, q, *iter)) {
                return false;
            }
        }
    }

    Kernel::RT px = p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root());
    Kernel::RT py = p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root());
    Kernel::RT qx = q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root());
    Kernel::RT qy = q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root());

    Kernel::RT x = (px + qx) / 2;
    Kernel::RT y = (py + qy) / 2;
    Point point(x, y);

    if (!check_inside(point, pgn.outer_boundary())) {
        return false;
    }

    for (auto h_iter = pgn.holes_begin(); h_iter != pgn.holes_end(); ++h_iter) {
        if (check_inside(point, *h_iter)) {
            return false;
        }
    }

    return true;
}

void find_tangents(IntervisibilityGraph &graph,
                   const IntervisibilityGraphVertexDescriptor &source_descriptor,
                   const Arrangement::X_monotone_curve_2 &curve,
                   const Polygon_with_holes &pgn,
                   std::vector<Arrangement::Point_2> &tangents) {
    const IntervisibilityGraphVertex &source = graph[source_descriptor];
    const Arrangement::Point_2 &source_point = *source.p;
    const Circle &circle = curve.supporting_circle();

    auto dx = source_point.x() - circle.center().x();
    auto dy = source_point.y() - circle.center().y();
    if ((dx * dx + dy * dy) <= circle.squared_radius()) {
        return;
    }

    Arrangement::Point_2 t1, t2;
    get_tangent_points_on_circle(circle, source_point, t1, t2);

    IntervisibilityGraphVertexIterator first, last;
    boost::tie(first, last) = boost::vertices(graph);

    const Arrangement::Point_2 &p = curve.source();
    auto vip = std::find_if(first, last,
                            [&](IntervisibilityGraphVertexDescriptor vd) {
                                return graph[vd].p->equals(p);
                            });
    CGAL_assertion(vip != last);
    IntervisibilityGraphVertexDescriptor vdp = *vip;

    const Arrangement::Point_2 &q = curve.source();
    auto viq = std::find_if(first, last,
                            [&](IntervisibilityGraphVertexDescriptor vd) {
                                return graph[vd].p->equals(p);
                            });
    CGAL_assertion(viq != last);
    IntervisibilityGraphVertexDescriptor vdq = *viq;

    if (curve.point_position(t1) == CGAL::EQUAL && is_visible(t1, *graph[source_descriptor].p, pgn)) {
        tangents.emplace_back(t1);
        IntervisibilityGraphVertexDescriptor vd = add_vertex(graph, &tangents.back());
        add_edge_circular(graph, vdp, vd, curve.supporting_circle());
        add_edge_circular(graph, vd, vdq, curve.supporting_circle());
        add_edge_linear(graph, source_descriptor, vd);
    }
    if (curve.point_position(t2) == CGAL::EQUAL && is_visible(t2, *graph[source_descriptor].p, pgn)) {
        tangents.emplace_back(t2);
        IntervisibilityGraphVertexDescriptor vd = add_vertex(graph, &tangents.back());
        add_edge_circular(graph, vdp, vd, curve.supporting_circle());
        add_edge_circular(graph, vd, vdq, curve.supporting_circle());
        add_edge_linear(graph, source_descriptor, vd);
    }
}

void get_shortest_path(const Move &move,
                       const Polygon &f,
                       std::vector<std::reference_wrapper<const Configuration>> &robots,
                       std::vector<Path> &paths) {
    Polygon_with_holes pgn(f);
    for (const auto &robot : robots) {
        const Point &p = robot.get().getPoint();
        if (p != move.first.getPoint() && p != move.second.getPoint() && check_inside(p, f)) {
            pgn.add_hole(D<2>(robot.get().getPoint()));
        }
    }

    IntervisibilityGraph graph;
    std::vector<IntervisibilityGraphVertexDescriptor> vertices;
    Arrangement::Point_2 s(move.first.getPoint().x(), move.first.getPoint().y());
    IntervisibilityGraphVertexDescriptor source = add_vertex(graph, &s);
    vertices.push_back(source);

    Arrangement::Point_2 t(move.second.getPoint().x(), move.second.getPoint().y());
    IntervisibilityGraphVertexDescriptor target = add_vertex(graph, &t);
    vertices.push_back(target);

    for (auto iter = pgn.outer_boundary().curves_begin(); iter != pgn.outer_boundary().curves_end(); ++iter) {
        const Arrangement::Point_2 &p = iter->source();
        auto vip = std::find_if(vertices.begin(), vertices.end(),
                                [&](IntervisibilityGraphVertexDescriptor vd) {
                                    return graph[vd].p->equals(p);
                                });
        IntervisibilityGraphVertexDescriptor vdp;
        if (vip == vertices.end()) {
            vdp = add_vertex(graph, &p);
            vertices.push_back(vdp);
        } else {
            vdp = *vip;
        }

        const Arrangement::Point_2 &q = iter->target();
        auto viq = std::find_if(vertices.begin(), vertices.end(),
                                [&](IntervisibilityGraphVertexDescriptor vd) {
                                    return graph[vd].p->equals(q);
                                });
        IntervisibilityGraphVertexDescriptor vdq;
        if (viq == vertices.end()) {
            vdq = add_vertex(graph, &q);
            vertices.push_back(vdq);
        } else {
            vdq = *viq;
        }

        if (iter->is_linear()) {
            add_edge_linear(graph, vdp, vdq);
        } else {
            add_edge_circular(graph, vdp, vdq, iter->supporting_circle());
        }
    }
    for (auto h_iter = pgn.holes_begin(); h_iter != pgn.holes_end(); ++h_iter) {
        for (auto iter = h_iter->curves_begin(); iter != h_iter->curves_end(); ++iter) {
            const Arrangement::Point_2 &p = iter->source();
            auto vip = std::find_if(vertices.begin(), vertices.end(),
                                    [&](IntervisibilityGraphVertexDescriptor vd) {
                                        return graph[vd].p->equals(p);
                                    });
            IntervisibilityGraphVertexDescriptor vdp;
            if (vip == vertices.end()) {
                vdp = add_vertex(graph, &p);
                vertices.push_back(vdp);
            } else {
                vdp = *vip;
            }

            const Arrangement::Point_2 &q = iter->target();
            auto viq = std::find_if(vertices.begin(), vertices.end(),
                                    [&](IntervisibilityGraphVertexDescriptor vd) {
                                        return graph[vd].p->equals(q);
                                    });
            IntervisibilityGraphVertexDescriptor vdq;
            if (viq == vertices.end()) {
                vdq = add_vertex(graph, &q);
                vertices.push_back(vdq);
            } else {
                vdq = *viq;
            }

            if (iter->is_linear()) {
                add_edge_linear(graph, vdp, vdq);
            } else {
                add_edge_circular(graph, vdp, vdq, iter->supporting_circle());
            }
        }
    }

    std::vector<Arrangement::Point_2> tangents;
    for (int i = 0; i < vertices.size(); i++) {
        const IntervisibilityGraphVertexDescriptor &vdi = vertices[i];
        const IntervisibilityGraphVertex &vi = graph[vdi];

        for (int j = 0; j < i; j++) {
            const IntervisibilityGraphVertexDescriptor &vdj = vertices[j];
            const IntervisibilityGraphVertex &vj = graph[vdj];

            if (is_visible(*vi.p, *vj.p, pgn)) {
                add_edge_linear(graph, vdi, vdj);
            }
        }

        for (auto iter = pgn.outer_boundary().curves_begin(); iter != pgn.outer_boundary().curves_end(); ++iter) {
            if (iter->is_circular()) {
                find_tangents(graph, vdi, *iter, pgn, tangents);
            }
        }

        for (auto h_iter = pgn.holes_begin(); h_iter != pgn.holes_end(); ++h_iter) {
            for (auto iter = h_iter->curves_begin(); iter != h_iter->curves_end(); ++iter) {
                if (iter->is_circular()) {
                    find_tangents(graph, vdi, *iter, pgn, tangents);
                }
            }
        }
    }

    std::cerr << std::endl;

    typedef boost::property_map<IntervisibilityGraph, boost::vertex_index_t>::type VertexIndexMap;
    VertexIndexMap vertex_index_map = boost::get(boost::vertex_index, graph);

    boost::property_map<IntervisibilityGraph, boost::edge_weight_t>::type weight_map = boost::get(boost::edge_weight,
                                                                                                  graph);

    typedef boost::iterator_property_map<IntervisibilityGraphVertexDescriptor *, VertexIndexMap, IntervisibilityGraphVertexDescriptor, IntervisibilityGraphVertexDescriptor &> VertexPredecessorMap;
    std::vector<IntervisibilityGraphVertexDescriptor> predecessors(boost::num_vertices(graph),
                                                                   boost::graph_traits<IntervisibilityGraph>::null_vertex());
    VertexPredecessorMap predecessor_map(predecessors.data(), vertex_index_map);

    std::less_equal<Kernel::RT> distance_compare;
    boost::closed_plus<Kernel::RT> distance_combine;
    Kernel::RT distance_inf = std::numeric_limits<double>::max();
    Kernel::RT distance_zero = 0.0;
    typedef boost::iterator_property_map<Kernel::RT *, VertexIndexMap, Kernel::RT, Kernel::RT &> VertexDistanceMap;
    std::vector<Kernel::RT> distances(boost::num_vertices(graph), distance_inf);
    VertexDistanceMap distance_map(distances.data(), vertex_index_map);

    std::cerr << "Graph: " << std::endl;
    boost::print_graph(graph, [&](IntervisibilityGraphVertexDescriptor vd) {
        std::stringstream ss;
        ss << "(" << *(graph[vd].p) << ")";
        return ss.str();
    }, std::cerr);
    std::cerr << std::endl;

    std::cerr << "Weights: " << std::endl;
    for (auto ei = boost::edges(graph).first; ei != boost::edges(graph).second; ++ei) {
        const IntervisibilityGraphEdgeDescriptor &ed = *ei;
        const IntervisibilityGraphVertexDescriptor &vs = boost::source(ed, graph);
        const IntervisibilityGraphVertexDescriptor &vt = boost::target(ed, graph);
        std::cerr << "weight(" << *(graph[vs].p) << ", " << *(graph[vt].p) << ") = " << weight_map[ed] << std::endl;
    }
    std::cerr << std::endl;

    boost::dijkstra_shortest_paths(graph, source, boost::vertex_index_map(vertex_index_map)
            .weight_map(weight_map)
            .predecessor_map(predecessor_map)
            .distance_map(distance_map)
            .distance_compare(distance_compare)
            .distance_combine(distance_combine)
            .distance_inf(distance_inf)
            .distance_zero(distance_zero));

    std::cerr << "Predecessors: " << std::endl;
    for (auto vi = boost::vertices(graph).first; vi != boost::vertices(graph).second; ++vi) {
        const IntervisibilityGraphVertexDescriptor &vd = *vi;
        std::cerr << "parent(" << *graph[vd].p << ") = ";
        if (predecessor_map[vd] == boost::graph_traits<IntervisibilityGraph>::null_vertex()) {
            std::cerr << "no parent";
        } else {
            std::cerr << "(" << *graph[predecessor_map[vd]].p << ")";
        }
        std::cerr << std::endl;
    }
    std::cerr << std::endl;

    Path path;
    IntervisibilityGraphVertexDescriptor previous = nullptr;
    for (IntervisibilityGraphVertexDescriptor current = target;
         current != predecessor_map[current]; current = predecessor_map[current]) {
        const Arrangement::Point_2 *p = graph[current].p;
        auto px = p->x();
        auto py = p->y();
#if defined(CGAL_EXACT_PREDICATES_EXACT_CONSTRUCTIONS_KERNEL_WITH_SQRT_H)
        double x = (px.a0() + px.a1() * CGAL::sqrt(px.root())).doubleValue();
        double y = (py.a0() + py.a1() * CGAL::sqrt(py.root())).doubleValue();
#elif defined(CGAL_EXACT_PREDICATES_INEXACT_CONSTRUCTIONS_KERNEL_H)
        double x = px.a0() + px.a1() * CGAL::sqrt(px.root());
        double y = py.a0() + py.a1() * CGAL::sqrt(py.root());
#endif
        if (previous != nullptr) {
            volatile bool circ = false;
            volatile double cx = 0.0, cy = 0.0;
            const Arrangement::Point_2 *q = graph[previous].p;
            auto iter = std::find_if(pgn.outer_boundary().curves_begin(), pgn.outer_boundary().curves_end(),
                                     [&](const Polygon_with_holes::General_polygon_2::X_monotone_curve_2 &curve) {
                                         const Arrangement::Point_2 &source = curve.source();
                                         const Arrangement::Point_2 &target = curve.target();
                                         if ((source.equals(*p) && target.equals(*q)) ||
                                             (source.equals(*q) && target.equals(*p))) {
                                             return curve.is_circular();
                                         }
                                         return false;
                                     });
            if (iter != pgn.outer_boundary().curves_end()) {
                circ = true;
#if defined(CGAL_EXACT_PREDICATES_EXACT_CONSTRUCTIONS_KERNEL_WITH_SQRT_H)
                cx = iter->supporting_circle().center().x().doubleValue();
                cy = iter->supporting_circle().center().y().doubleValue();
#elif defined(CGAL_EXACT_PREDICATES_INEXACT_CONSTRUCTIONS_KERNEL_H)
                cx = iter->supporting_circle().center().x();
                cy = iter->supporting_circle().center().y();
#endif
            } else {
                for (auto hole = pgn.holes_begin(); hole != pgn.holes_end(); ++hole) {
                    iter = std::find_if(hole->curves_begin(), hole->curves_end(),
                                        [&](const Polygon_with_holes::General_polygon_2::X_monotone_curve_2 &curve) {
                                            const Arrangement::Point_2 &source = curve.source();
                                            const Arrangement::Point_2 &target = curve.target();
                                            if ((source.equals(*p) && target.equals(*q)) ||
                                                (source.equals(*q) && target.equals(*p))) {
                                                return curve.is_circular();
                                            }
                                            return false;
                                        });
                    if (iter != hole->curves_end()) {
                        circ = true;
#if defined(CGAL_EXACT_PREDICATES_EXACT_CONSTRUCTIONS_KERNEL_WITH_SQRT_H)
                        cx = iter->supporting_circle().center().x().doubleValue();
                        cy = iter->supporting_circle().center().y().doubleValue();
#elif defined(CGAL_EXACT_PREDICATES_INEXACT_CONSTRUCTIONS_KERNEL_H)
                        cx = iter->supporting_circle().center().x();
                cy = iter->supporting_circle().center().y();
#endif
                        break;
                    }
                }
            }

            path.elements.push_back({circ, std::make_pair(x, y), std::make_pair(cx, cy)});
        } else {
            path.elements.push_back({false, std::make_pair(x, y), std::make_pair(0, 0)});
        }

        previous = current;
    }
    const Arrangement::Point_2 *p = graph[source].p;
    auto px = p->x();
    auto py = p->y();
#if defined(CGAL_EXACT_PREDICATES_EXACT_CONSTRUCTIONS_KERNEL_WITH_SQRT_H)
    double x = (px.a0() + px.a1() * CGAL::sqrt(px.root())).doubleValue();
    double y = (py.a0() + py.a1() * CGAL::sqrt(py.root())).doubleValue();
#elif defined(CGAL_EXACT_PREDICATES_INEXACT_CONSTRUCTIONS_KERNEL_H)
    double x = px.a0() + px.a1() * CGAL::sqrt(px.root());
    double y = py.a0() + py.a1() * CGAL::sqrt(py.root());
#endif
    volatile bool circ = false;
    volatile double cx = 0.0, cy = 0.0;
    const Arrangement::Point_2 *q = graph[previous].p;
    auto iter = std::find_if(pgn.outer_boundary().curves_begin(), pgn.outer_boundary().curves_end(),
                             [&](const Polygon_with_holes::General_polygon_2::X_monotone_curve_2 &curve) {
                                 const Arrangement::Point_2 &source = curve.source();
                                 const Arrangement::Point_2 &target = curve.target();
                                 if ((source.equals(*p) && target.equals(*q)) ||
                                     (source.equals(*q) && target.equals(*p))) {
                                     return curve.is_circular();
                                 }
                                 return false;
                             });
    if (iter != pgn.outer_boundary().curves_end()) {
        circ = true;
#if defined(CGAL_EXACT_PREDICATES_EXACT_CONSTRUCTIONS_KERNEL_WITH_SQRT_H)
        cx = iter->supporting_circle().center().x().doubleValue();
        cy = iter->supporting_circle().center().y().doubleValue();
#elif defined(CGAL_EXACT_PREDICATES_INEXACT_CONSTRUCTIONS_KERNEL_H)
        cx = iter->supporting_circle().center().x();
                cy = iter->supporting_circle().center().y();
#endif
    } else {
        for (auto hole = pgn.holes_begin(); hole != pgn.holes_end(); ++hole) {
            iter = std::find_if(hole->curves_begin(), hole->curves_end(),
                                [&](const Polygon_with_holes::General_polygon_2::X_monotone_curve_2 &curve) {
                                    const Arrangement::Point_2 &source = curve.source();
                                    const Arrangement::Point_2 &target = curve.target();
                                    if ((source.equals(*p) && target.equals(*q)) ||
                                        (source.equals(*q) && target.equals(*p))) {
                                        return curve.is_circular();
                                    }
                                    return false;
                                });
            if (iter != hole->curves_end()) {
                circ = true;
#if defined(CGAL_EXACT_PREDICATES_EXACT_CONSTRUCTIONS_KERNEL_WITH_SQRT_H)
                cx = iter->supporting_circle().center().x().doubleValue();
                cy = iter->supporting_circle().center().y().doubleValue();
#elif defined(CGAL_EXACT_PREDICATES_INEXACT_CONSTRUCTIONS_KERNEL_H)
                cx = iter->supporting_circle().center().x();
                cy = iter->supporting_circle().center().y();
#endif
                break;
            }
        }
    }

    path.elements.push_back({circ, std::make_pair(x, y), std::make_pair(cx, cy)});
    paths.push_back(path);

    std::cerr << "Source: (" << *graph[source].p << ")" << std::endl << "Target: (" << *graph[target].p << ")"
              << std::endl
              << "Shortest path (reversed): ";
    for (auto e : path.elements) {
        std::cerr << "(" << e.next.first << ", " << e.next.second << ") <- ";
    }
    std::cerr << std::endl;

    // Move the robot
    robots.erase(
            std::remove_if(robots.begin(), robots.end(),
                           [&move](const std::reference_wrapper<const Configuration> &robot) {
                               return move.first.getPoint() == robot.get().getPoint();
                           }),
            robots.end()
    );
    robots.emplace_back(move.second);
}

void simplify_moves(std::vector<Move> &moves) {
    std::vector<Move> newMoves;

    std::vector<Move> simplified;
    for (int i = 0; i < moves.size(); i++) {
        if (std::find(simplified.begin(), simplified.end(), moves[i]) != simplified.end()) {
            continue;
        }

        std::reference_wrapper<const Configuration> source = moves[i].first;
        std::reference_wrapper<const Configuration> target = moves[i].second;

        for (int j = i + 1; j < moves.size(); j++) {
            if (moves[j].first == target) {
                target = moves[j].second;
                simplified.push_back(moves[j]);
            }
        }

        newMoves.emplace_back(source, target);
    }

    moves.clear();
    for (const Move &move : newMoves) {
        moves.emplace_back(move);
    }
}