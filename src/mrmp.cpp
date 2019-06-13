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
        for (boost::tie(vi, v_end) = boost::vertices(G_i); vi != v_end; ++vi) {
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

//        std::vector<MotionGraphVertexDescriptor> B_i, H_i;
//        const Polygon &boundary = F_star_i.outer_boundary();
//
//        typename boost::graph_traits<MotionGraph>::vertex_iterator vi, v_end;
//        for (boost::tie(vi, v_end) = boost::vertices(G_i); vi != v_end; ++vi) {
//            const MotionGraphVertexDescriptor &vd = *vi;
//            const MotionGraphVertex &v = G_i[vd];
//            const Configuration &configuration = *v.configuration;
//            const Point &point = configuration.getPoint();
//
//            if (check_inside(point, boundary)) {
//                H_i.push_back(vd);
//            } else if (check_inside(point, F_i) && do_intersect(D<2>(point), boundary)) {
//                B_i.push_back(vd);
//            }
//        }
//
//        for (const MotionGraphVertexDescriptor &b : B_i) {
//            for (const MotionGraphVertexDescriptor &h : H_i) {
//                boost::add_edge(b, h, G_i);
//            }
//        }
//
//        for (int i = 0; i < B_i.size(); i++) {
//            for (int j = 0; j < i; j++) {
//                boost::add_edge(B_i[i], B_i[j], G_i);
//            }
//        }
//        for (int i = 0; i < H_i.size(); i++) {
//            for (int j = 0; j < i; j++) {
//                boost::add_edge(H_i[i], H_i[j], G_i);
//            }
//        }
    }
}

MotionGraphVertexDescriptor find_shortest_path(MotionGraph &graph,
                                               const MotionGraphVertexDescriptor &start,
                                               const std::function<bool(MotionGraphVertexDescriptor)> &predicate) {

    if (predicate(start)) {
        return start;
    }

    std::queue<MotionGraphVertexDescriptor> queue;

    typename boost::graph_traits<MotionGraph>::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(graph);
         vi != vi_end;
         ++vi) {
        graph[*vi].visited = false;
        graph[*vi].predecessor = nullptr;
    }

    graph[start].visited = true;
    graph[start].predecessor = start;
    queue.push(start);

    while (!queue.empty()) {
        MotionGraphVertexDescriptor u = queue.front();
        queue.pop();

        auto neighbours = boost::adjacent_vertices(u, graph);
        for (auto target : boost::make_iterator_range(neighbours)) {
            if (!graph[target].visited) {
                graph[target].visited = true;
                graph[target].predecessor = u;
                queue.push(target);

                if (predicate(target)) {
                    return target;
                }
            }

        }
    }

    CGAL_assertion(false);
    return nullptr;
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
    for (boost::tie(G_i_v, G_i_v_end) = boost::vertices(G_i); G_i_v != G_i_v_end; ++G_i_v) {
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

        const MotionGraphVertexDescriptor &v = *v_j;
        std::cerr << "Current leave: "
                  << (T_g[v].configuration->isStart() ? "s" : "t") + std::to_string(T_g[v].configuration->getIndex())
                  << std::endl;

        const MotionGraphVertex &n = T_g[v];

        // Find closest pebble w with/without a pebble
        MotionGraphVertexDescriptor w;
        if (n.configuration->isStart()) {
            w = find_shortest_path(T_g, v, [&](MotionGraphVertexDescriptor vd) {
                return !T_g[vd].hasPebble;
            });
        } else {
            w = find_shortest_path(T_g, v, [&](MotionGraphVertexDescriptor vd) {
                return T_g[vd].hasPebble;
            });
        }

        std::cerr << "Closest corresponding node: "
                  << (T_g[w].configuration->isStart() ? "s" : "t") + std::to_string(T_g[w].configuration->getIndex())
                  << std::endl;

        for (MotionGraphVertexDescriptor current = w; current != v; current = T_g[current].predecessor) {
            if (n.configuration->isStart()) {
                MotionGraphVertex &target = T_g[current];
                MotionGraphVertex &source = T_g[target.predecessor];

                CGAL_assertion(source.hasPebble);
                CGAL_assertion(!target.hasPebble);
                source.hasPebble = false;
                target.hasPebble = true;
                moves.emplace_back(source.configuration, target.configuration);
            } else {
                MotionGraphVertex &source = T_g[current];
                MotionGraphVertex &target = T_g[source.predecessor];

                CGAL_assertion(source.hasPebble);
                CGAL_assertion(!target.hasPebble);
                source.hasPebble = false;
                target.hasPebble = true;
                moves.emplace_back(source.configuration, target.configuration);
            }
        }

        CGAL_assertion(n.hasPebble != n.configuration->isStart());

        // Remove vertex from T_g
        boost::clear_vertex(v, T_g);
        boost::remove_vertex(v, T_g);

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

    boost::tie(e, b) = boost::add_edge(vdp, vdq, graph);

    if (b) {
        const Arrangement::Point_2 &p = *(vp.p);
        const Arrangement::Point_2 &q = *(vq.p);

        Kernel::RT px = (p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root()));
        Kernel::RT py = (p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root()));
        Kernel::RT qx = (q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root()));
        Kernel::RT qy = (q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root()));

        graph[e].edge_weight = CGAL::sqrt((px - qx) * (px - qx)) + ((py - qy) * (py - qy));
    }

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

    boost::tie(e, b) = boost::add_edge(vp, vq, graph);

    if (b) {
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
        double angle = atan2(qy - cy, qx - cx) - atan2(py - cy, px - cx);
        graph[e].edge_weight = Kernel::RT(angle * CGAL::sqrt(circle.squared_radius()));
    }
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

class Intervisibility_vertex_index_map {
public:
    typedef int value_type;
    typedef const value_type &reference;
    typedef IntervisibilityGraphVertexDescriptor key_type;
    typedef boost::readable_property_map_tag category;

    friend value_type get(Intervisibility_vertex_index_map me, key_type key) {
        auto iter = std::find(me.vertices.begin(), me.vertices.end(), key);
        return iter - me.vertices.begin();
    }

    explicit Intervisibility_vertex_index_map(const std::vector<IntervisibilityGraphVertexDescriptor> &vertices)
            : vertices(vertices) {}

private:
    const std::vector<IntervisibilityGraphVertexDescriptor> &vertices;
};

class Intervisibility_weight_map {
public:
    typedef Kernel::RT value_type;
    typedef const value_type &reference;
    typedef IntervisibilityGraphEdgeDescriptor key_type;
    typedef boost::readable_property_map_tag category;

    friend value_type get(Intervisibility_weight_map me, key_type key) {
        return me.graph[key].edge_weight;
    }

    explicit Intervisibility_weight_map(const IntervisibilityGraph &graph) : graph(graph) {}

private:
    const IntervisibilityGraph &graph;
};

class Intervisibility_predecessor_map {
public:
    typedef IntervisibilityGraphVertexDescriptor key_type;
    typedef IntervisibilityGraphVertexDescriptor value_type;
    typedef const value_type &reference;
    typedef boost::read_write_property_map_tag category;

    friend value_type get(Intervisibility_predecessor_map me, key_type key) {
        if (me._map.find(key) == me._map.end()) {
            return key;
        }
        return me._map[key];
    }

    friend void put(Intervisibility_predecessor_map me, key_type key, value_type value) {
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

bool intersect_inclusive(const Arrangement::X_monotone_curve_2 &curve1, const Arrangement::X_monotone_curve_2 &curve2) {
    // TODO - Find new implementation. Intersect is SUPER slow
    Arrangement::Point_2 tmp;
    std::vector<CGAL::Object> objects;
//    CircularKernel ck;
//    CircularKernel::Do_intersect_2 doIntersect2 = ck.do_intersect_2_object();
//    return doIntersect2(curve1, curve2);

    curve1.intersect(curve2, std::back_inserter(objects));

    for (auto object : objects) {
        if (CGAL::assign(tmp, object)) {
            if (!tmp.equals(curve1.source()) && !tmp.equals(curve1.target()) && !tmp.equals(curve2.source()) && !tmp.equals(curve2.target())) {
                return true;
            }
        }
    }
    return false;
}

bool is_visible(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q, const Polygon_with_holes &pgn) {
    Arrangement::X_monotone_curve_2 edge = get_edge(p, q);
    for (auto iter = pgn.outer_boundary().curves_begin(); iter != pgn.outer_boundary().curves_end(); ++iter) {
//        if (intersect_inclusive(edge, *iter)) {
//            return false;
//        }
    }
    for (auto h_iter = pgn.holes_begin(); h_iter != pgn.holes_end(); ++h_iter) {
        for (auto iter = h_iter->curves_begin(); iter != h_iter->curves_end(); ++iter) {
//            if (intersect_inclusive(edge, *iter)) {
//                return false;
//            }
        }
    }

    // TODO: Check if edge is inside pgn (and not inside pgn.holes)

    return true;
}

void find_tangents(IntervisibilityGraph &graph,
                   const IntervisibilityGraphVertexDescriptor &source_descriptor,
                   const Arrangement::X_monotone_curve_2 &curve,
                   std::vector<Arrangement::Point_2> &tangents,
                   std::vector<IntervisibilityGraphVertexDescriptor> &vertex_index_map) {
    const IntervisibilityGraphVertex &source = graph[source_descriptor];
    const Arrangement::Point_2 &source_point = *source.p;

    Arrangement::Point_2 t1, t2;
    get_tangent_points_on_circle(curve.supporting_circle(), source_point, t1, t2);

    const Arrangement::Point_2 &p = curve.source();
    auto vip = std::find_if(vertex_index_map.begin(), vertex_index_map.end(),
                            [&](IntervisibilityGraphVertexDescriptor vd) {
                                return graph[vd].p->equals(p);
                            });
    CGAL_assertion(vip != vertex_index_map.end());
    IntervisibilityGraphVertexDescriptor vdp = *vip;

    const Arrangement::Point_2 &q = curve.source();
    auto viq = std::find_if(vertex_index_map.begin(), vertex_index_map.end(),
                            [&](IntervisibilityGraphVertexDescriptor vd) {
                                return graph[vd].p->equals(q);
                            });
    CGAL_assertion(viq != vertex_index_map.end());
    IntervisibilityGraphVertexDescriptor vdq = *viq;

    if (curve.point_position(t1) == CGAL::EQUAL) {
        tangents.emplace_back(t1);
        IntervisibilityGraphVertexDescriptor vd = add_vertex(graph, &tangents.back());
        vertex_index_map.push_back(vd);
        add_edge_circular(graph, vdp, vd, curve.supporting_circle());
        add_edge_circular(graph, vd, vdq, curve.supporting_circle());
        add_edge_linear(graph, source_descriptor, vd);
    }
    if (curve.point_position(t2) == CGAL::EQUAL) {
        tangents.emplace_back(t2);
        IntervisibilityGraphVertexDescriptor vd = add_vertex(graph, &tangents.back());
        vertex_index_map.push_back(vd);
        add_edge_circular(graph, vdp, vd, curve.supporting_circle());
        add_edge_circular(graph, vd, vdq, curve.supporting_circle());
        add_edge_linear(graph, source_descriptor, vd);
    }
}

void get_shortest_path(const Move &move,
                       const Polygon &f,
                       std::vector<std::reference_wrapper<Configuration>> &robots) {
    Polygon_with_holes pgn(f);
    for (std::reference_wrapper<Configuration> robot : robots) {
        const Point &p = robot.get().getPoint();
        if (p != move.first->getPoint() && p != move.second->getPoint() && check_inside(p, f)) {
            pgn.add_hole(D<2>(robot.get().getPoint()));
        }
    }

    IntervisibilityGraph graph;
    std::vector<IntervisibilityGraphVertexDescriptor> vertex_index_map;
    Arrangement::Point_2 s(move.first->getPoint().x(), move.first->getPoint().y());
    IntervisibilityGraphVertexDescriptor source = add_vertex(graph, &s);
    vertex_index_map.push_back(source);

    Arrangement::Point_2 t(move.second->getPoint().x(), move.second->getPoint().y());
    IntervisibilityGraphVertexDescriptor target = add_vertex(graph, &t);
    vertex_index_map.push_back(target);

    for (auto iter = pgn.outer_boundary().curves_begin(); iter != pgn.outer_boundary().curves_end(); ++iter) {
        const Arrangement::Point_2 &p = iter->source();
        auto vip = std::find_if(vertex_index_map.begin(), vertex_index_map.end(),
                                [&](IntervisibilityGraphVertexDescriptor vd) {
                                    return graph[vd].p->equals(p);
                                });
        IntervisibilityGraphVertexDescriptor vdp;
        if (vip == vertex_index_map.end()) {
            vdp = add_vertex(graph, &p);
            vertex_index_map.push_back(vdp);
        } else {
            vdp = *vip;
        }

        const Arrangement::Point_2 &q = iter->target();
        auto viq = std::find_if(vertex_index_map.begin(), vertex_index_map.end(),
                                [&](IntervisibilityGraphVertexDescriptor vd) {
                                    return graph[vd].p->equals(q);
                                });
        IntervisibilityGraphVertexDescriptor vdq;
        if (viq == vertex_index_map.end()) {
            vdq = add_vertex(graph, &q);
            vertex_index_map.push_back(vdq);
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
            auto vip = std::find_if(vertex_index_map.begin(), vertex_index_map.end(),
                                    [&](IntervisibilityGraphVertexDescriptor vd) {
                                        return graph[vd].p->equals(p);
                                    });
            IntervisibilityGraphVertexDescriptor vdp;
            if (vip == vertex_index_map.end()) {
                vdp = add_vertex(graph, &p);
                vertex_index_map.push_back(vdp);
            } else {
                vdp = *vip;
            }

            const Arrangement::Point_2 &q = iter->target();
            auto viq = std::find_if(vertex_index_map.begin(), vertex_index_map.end(),
                                    [&](IntervisibilityGraphVertexDescriptor vd) {
                                        return graph[vd].p->equals(q);
                                    });
            IntervisibilityGraphVertexDescriptor vdq;
            if (viq == vertex_index_map.end()) {
                vdq = add_vertex(graph, &q);
                vertex_index_map.push_back(vdq);
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
    for (int i = 0; i < vertex_index_map.size(); i++) {
        const IntervisibilityGraphVertexDescriptor &vdi = vertex_index_map[i];
        const IntervisibilityGraphVertex &vi = graph[vdi];

        for (int j = 0; j < i; j++) {
            const IntervisibilityGraphVertexDescriptor &vdj = vertex_index_map[j];
            const IntervisibilityGraphVertex &vj = graph[vdj];

            if (is_visible(*vi.p, *vj.p, pgn)) {
                add_edge_linear(graph, vdi, vdj);
            }
        }

        for (auto iter = pgn.outer_boundary().curves_begin(); iter != pgn.outer_boundary().curves_end(); ++iter) {
            if (iter->is_circular()) {
                find_tangents(graph, vdi, *iter, tangents, vertex_index_map);
            }
        }

        for (auto h_iter = pgn.holes_begin(); h_iter != pgn.holes_end(); ++h_iter) {
            for (auto iter = h_iter->curves_begin(); iter != h_iter->curves_end(); ++iter) {
                if (iter->is_circular()) {
                    find_tangents(graph, vdi, *iter, tangents, vertex_index_map);
                }
            }
        }
    }

    boost::print_graph(graph, [&](IntervisibilityGraphVertexDescriptor vd) {
        std::stringstream ss;
        ss << "(" << *(graph[vd].p) << ")";
        return ss.str();
    });

    Intervisibility_weight_map weight_map(graph);
    Intervisibility_predecessor_map predecessor_map;
    boost::dijkstra_shortest_paths(graph, source,
                                   boost::vertex_index_map(Intervisibility_vertex_index_map(vertex_index_map))
                                           .weight_map(weight_map)
                                           .predecessor_map(predecessor_map)
    );

    std::list<const Arrangement::Point_2 *> path;
    for (IntervisibilityGraphVertexDescriptor current = target;
         current != nullptr; current = predecessor_map[current]) {
        path.push_front(graph[current].p);
    }
    path.push_front(graph[source].p);


    for (auto p : path) {
        std::cerr << "(" << *p << "), ";
    }
    std::cerr << std::endl;
}

void simplify_moves(std::vector<Move> &moves) {
    std::vector<Move> newMoves;

    std::vector<Move> simplified;
    for (int i = 0; i < moves.size(); i++) {
        if (std::find(simplified.begin(), simplified.end(), moves[i]) != simplified.end()) {
            continue;
        }

        const Configuration *source = moves[i].first;
        const Configuration *target = moves[i].second;

        for (int j = i + 1; j < moves.size(); j++) {
            if (moves[j].first == target) {
                target = moves[j].second;
                simplified.push_back(moves[j]);
            }
        }

        newMoves.emplace_back(source, target);
    }

    moves = newMoves;
}