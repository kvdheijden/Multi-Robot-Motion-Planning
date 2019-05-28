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

std::string get(const std::function<std::string(void * )> &name,
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


static General_polygon_set remove_start_target_configs(const Polygon &F,
                                                       const ConfigurationSet &U) {
    General_polygon_set gps(F);

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

    return gps;
}

bool do_intersect(const Polygon &pgn1, const Polygon::X_monotone_curve_2 &curve) {
    for (auto iter1 = pgn1.curves_begin(); iter1 != pgn1.curves_end(); ++iter1) {
        std::list <CGAL::Object> objects;
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

    for (const Configuration &u : U) {
        if (check_inside(u.getPoint(), F_i)) {
            MotionGraphVertexDescriptor v = boost::add_vertex(G_i);
            G_i[v].configuration = &u;
            G_i[v].hasPebble = u.isStart();
        }
    }

    General_polygon_set gps = remove_start_target_configs(F_i, U);
    std::vector <Polygon_with_holes> F_star;
    gps.polygons_with_holes(std::back_inserter(F_star));

    // Add edges between vertices in H_i
    for (const Polygon_with_holes &F_star_i : F_star) {
        std::vector <MotionGraphVertexDescriptor> B_i, H_i;
        const Polygon &boundary = F_star_i.outer_boundary();

        typename boost::graph_traits<MotionGraph>::vertex_iterator vi, v_end;
        for (boost::tie(vi, v_end) = boost::vertices(G_i);
        vi != v_end;
        ++vi) {
            const MotionGraphVertexDescriptor &vd = *vi;
            const MotionGraphVertex &v = G_i[vd];
            const Configuration &configuration = *v.configuration;
            const Point &point = configuration.getPoint();

            if (check_inside(point, boundary)) {
                H_i.push_back(vd);
            } else if (check_inside(point, F_i) && do_intersect(D<2>(point), boundary)) {
                B_i.push_back(vd);
            }
        }

        for (const MotionGraphVertexDescriptor &b : B_i) {
            for (const MotionGraphVertexDescriptor &h : H_i) {
                boost::add_edge(b, h, G_i);
            }
        }

        for (int i = 0; i < B_i.size(); i++) {
            for (int j = 0; j < i; j++) {
                boost::add_edge(B_i[i], B_i[j], G_i);
            }
        }
        for (int i = 0; i < H_i.size(); i++) {
            for (int j = 0; j < i; j++) {
                boost::add_edge(H_i[i], H_i[j], G_i);
            }
        }
    }
}

MotionGraphVertexDescriptor find_shortest_path(MotionGraph &graph,
                                               const MotionGraphVertexDescriptor &start,
                                               const std::function<bool(MotionGraphVertexDescriptor)> &predicate) {

    if (predicate(start)) {
        return start;
    }

    std::queue <MotionGraphVertexDescriptor> queue;

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
    typedef CORE::Expr value_type;
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
        for (boost::tie(g_i, g_end) = boost::vertices(graph); g_i != g_end; ++g_i) {
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
                        std::vector <Move> &moves) {
    std::vector <MotionGraphEdgeDescriptor> spanning_tree;
    ST_Weight_Map weight_map(G_i);
    ST_Vertex_Index_Map vertex_index_map(G_i);
    boost::kruskal_minimum_spanning_tree(G_i, std::back_inserter(spanning_tree),
                                         boost::vertex_index_map(vertex_index_map).weight_map(weight_map));

    MotionGraph T_g;
    boost::graph_traits<MotionGraph>::vertex_iterator G_i_v, G_i_v_end;
    for (boost::tie(G_i_v, G_i_v_end) = boost::vertices(G_i);
    G_i_v != G_i_v_end;
    ++G_i_v) {
        MotionGraphVertexDescriptor mgvd = T_g.add_vertex();
        T_g[mgvd] = G_i[*G_i_v];
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
    typedef Arrangement::Halfedge_handle key_type;
    typedef CORE::Expr value_type;
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
    std::map <key_type, value_type> _map;
};

class Polygon_with_holes_curve_iterator {
public:
    class iterator {
    private:
        const Polygon_with_holes *pgn;
        Polygon_with_holes::General_polygon_2::Curve_const_iterator c_iter;
        Polygon_with_holes::Hole_const_iterator h_iter;
        bool iterating_boundary;

    public:
        iterator() : pgn(nullptr), iterating_boundary(false) {

        }

        explicit iterator(const Polygon_with_holes &pgn) :
                pgn(&pgn),
                c_iter(pgn.outer_boundary().curves_begin()),
                h_iter(pgn.holes_begin()),
                iterating_boundary(true) {

        }

        iterator(const iterator &) = default;

        iterator &operator++() {
            // Increment curve pointer
            c_iter++;

            // If we're done with the boundary, start iterating holes
            if (iterating_boundary && c_iter == pgn->outer_boundary().curves_end()) {
                iterating_boundary = false;
                if (h_iter != pgn->holes_end()) {
                    c_iter = h_iter->curves_begin();
                }
            } else if (!iterating_boundary && c_iter == h_iter->curves_end()) {
                h_iter++;
                if (h_iter != pgn->holes_end()) {
                    c_iter = h_iter->curves_begin();
                }
            }

            if (h_iter == pgn->holes_end()) {
                pgn = nullptr;
            }

            return *this;
        }

        const iterator operator++(int) {
            iterator result(*this);
            ++(*this);
            return result;
        }

        constexpr bool operator!=(const iterator &that) const {
            return pgn != that.pgn;
        }

        Polygon_with_holes::General_polygon_2::Curve_const_iterator::value_type operator*() const {
            return *c_iter;
        }
    };

private:
    const Polygon_with_holes &pgn;

public:
    explicit Polygon_with_holes_curve_iterator(const Polygon_with_holes &pgn) : pgn(pgn) {}

    iterator begin() {
        return iterator(pgn);
    }

    iterator end() {
        return iterator();
    }

};

Line get_line(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q) {
    CORE::Expr Px = p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root());
    CORE::Expr Py = p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root());
    CORE::Expr Qx = q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root());
    CORE::Expr Qy = q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root());
    return Line(Point(Px, Py), Point(Qx, Qy));
}

Arrangement::X_monotone_curve_2 get_edge(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q) {
    Line l = get_line(p, q);
    return Arrangement::X_monotone_curve_2(l, p, q);
}

CORE::Expr get_distance(const Arrangement::Point_2 &p, const Arrangement::Point_2 &q) {
    CORE::Expr Px = p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root());
    CORE::Expr Py = p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root());
    CORE::Expr Qx = q.x().a0() + q.x().a1() * CGAL::sqrt(q.x().root());
    CORE::Expr Qy = q.y().a0() + q.y().a1() * CGAL::sqrt(q.y().root());
    auto diff_x = Px - Qx;
    auto diff_y = Py - Qy;
    return CGAL::sqrt(diff_x * diff_x + diff_y * diff_y);
}

CORE::Expr get_distance(const Arrangement::X_monotone_curve_2 &curve) {
    if (curve.is_linear()) {
        return get_distance(curve.source(), curve.target());
    }

    std::vector <std::pair<double, double>> polyline;
    curve.approximate(std::back_inserter(polyline), 10);
    CORE::Expr r(0);
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
    const Point &c = circle.center();
    CORE::Expr Cx = c.x();
    CORE::Expr Cy = c.y();
    CORE::Expr r = CGAL::sqrt(circle.squared_radius());
    CORE::Expr Px = p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root());
    CORE::Expr Py = p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root());

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

    s = Arrangement::Point_2(T1x, T1y);
    t = Arrangement::Point_2(T2x, T2y);
}

void get_tangent_point(const Arrangement::X_monotone_curve_2 &curve, const Arrangement::Point_2 &p,
                       std::vector <std::pair<Arrangement::Point_2, Arrangement::Point_2>> &points) {
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
                        std::vector <std::pair<Arrangement::Point_2, Arrangement::Point_2>> &points) {
    CGAL_assertion(curve1.is_circular());
    CGAL_assertion(curve2.is_circular());

    const Circle &circle1 = curve1.supporting_circle();
    const Circle &circle2 = curve2.supporting_circle();

    CORE::Expr r1 = CGAL::sqrt(circle1.squared_radius());
    CORE::Expr r2 = CGAL::sqrt(circle2.squared_radius());

    const Point &c1 = circle1.center();
    const Point &c2 = circle2.center();

    CORE::Expr C1x = c1.x();
    CORE::Expr C1y = c1.y();
    CORE::Expr C2x = c2.x();
    CORE::Expr C2y = c2.y();

    const Arrangement::Point_2 C1(C1x, C1y);
    const Arrangement::Point_2 C2(C2x, C2y);

    Arrangement::Point_2 T1, T2;
    Point p[8], q[8];

    if (r1 >= r2) {
        // Outer tangents
        const Circle circle3(circle1.center(), CGAL::square(r1 - r2));
        get_tangent_points_on_circle(circle3, C2, T1, T2);
        CORE::Expr T1x = T1.x().a0() + T1.x().a1() * CGAL::sqrt(T1.x().root());
        CORE::Expr T1y = T1.y().a0() + T1.y().a1() * CGAL::sqrt(T1.y().root());
        CORE::Expr T2x = T2.x().a0() + T2.x().a1() * CGAL::sqrt(T2.x().root());
        CORE::Expr T2y = T2.y().a0() + T2.y().a1() * CGAL::sqrt(T2.y().root());
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

void get_shortest_path(const Move &move,
                       const Polygon &f,
                       std::vector<const Configuration *> &robots) {
    Polygon_with_holes pgn(f);
    for (const Configuration *robot : robots) {
        const Point &p = robot->getPoint();
        if (p != move.first->getPoint() && p != move.second->getPoint() && check_inside(p, f)) {
            pgn.add_hole(D<2>(robot->getPoint()));
        }
    }

    Arrangement::Point_2 source(move.first->getPoint().x(), move.first->getPoint().y());
    Arrangement::Point_2 target(move.second->getPoint().x(), move.second->getPoint().y());

    Arrangement env;
    Arrangement::Face_handle uf = env.unbounded_face();
    Arrangement::Vertex_handle v = env.insert_in_face_interior(source, uf);
    Arrangement::Vertex_handle w = env.insert_in_face_interior(target, uf);
    edge_weight_map weight_map;

    std::vector <std::pair<Arrangement::Point_2, Arrangement::Point_2>> points;
    for (auto curve1 : Polygon_with_holes_curve_iterator(pgn)) {
        for (auto curve2 : Polygon_with_holes_curve_iterator(pgn)) {
            if (curve1.is_circular() && curve2.is_circular()) {
                get_tangent_points(curve1, curve2, points);
            } else if (curve1.is_circular()) {
                Arrangement::Point_2 ps[] = {curve2.source(), curve2.target()};
                for (const Arrangement::Point_2 &p : ps) {
                    get_tangent_point(curve1, p, points);
                }
            } else if (curve2.is_circular()) {
                Arrangement::Point_2 ps[] = {curve1.source(), curve1.target()};
                for (const Arrangement::Point_2 &p : ps) {
                    get_tangent_point(curve2, p, points);
                }
            } else {
                Arrangement::Point_2 ps[] = {curve1.source(),
                                             curve2.source(),
                                             curve1.target(),
                                             curve2.target()};
                for (int i = 0; i < 4; i++) {
                    Arrangement::Point_2 &p = ps[i];
                    for (int j = 0; j < i; j++) {
                        Arrangement::Point_2 &q = ps[j];
                        points.emplace_back(p, q);
                    }
                }
            }
        }
    }

    for (std::pair <Arrangement::Point_2, Arrangement::Point_2> &ps : points) {
        Arrangement::Point_2 &p = ps.first;
        Arrangement::Point_2 &q = ps.second;
        if (p == q) {
            continue;
        }
        Arrangement::X_monotone_curve_2 s = get_edge(p, q);

        bool valid = true;
        for (const Arrangement::X_monotone_curve_2 &curve : Polygon_with_holes_curve_iterator(pgn)) {
            if (!valid) {
                break;
            }

            std::vector <CGAL::Object> objects;
            curve.intersect(s, std::back_inserter(objects));

            for (const CGAL::Object &obj : objects) {
                Arrangement::X_monotone_curve_2::Intersection_point_2 i;
                if (CGAL::assign(i, obj)) {
                    Arrangement::Point_2 &r = i.first;
                    if (r != p && r != q && r != curve.source() && r != curve.target()) {
                        valid = false;
                        break;
                    }
                }
            }
        }

        if (valid) {
            CORE::Expr dist = get_distance(p, q);

            Arrangement::Halfedge_handle e = env.insert_in_face_interior(s, uf);
            put(weight_map, e, dist);

            for (const Arrangement::X_monotone_curve_2 &curve : Polygon_with_holes_curve_iterator(pgn)) {
                Arrangement::X_monotone_curve_2 l, r;
                if (curve.point_position(p) == CGAL::EQUAL) {
                    curve.split(p, l, r);

                    e = env.insert_in_face_interior(l, uf);
                    put(weight_map, e, get_distance(l));

                    e = env.insert_in_face_interior(r, uf);
                    put(weight_map, e, get_distance(r));
                }
                if (curve.point_position(q) == CGAL::EQUAL) {
                    curve.split(q, l, r);

                    e = env.insert_in_face_interior(l, uf);
                    put(weight_map, e, get_distance(l));

                    e = env.insert_in_face_interior(r, uf);
                    put(weight_map, e, get_distance(r));
                }
            }
        }
    }

    CGAL::Arr_vertex_index_map<Arrangement> index_map(env);
    boost::vector_property_map<CORE::Expr,
            CGAL::Arr_vertex_index_map<Arrangement >> dist_map(env.number_of_vertices(), index_map);
    boost::vector_property_map<Arrangement::Vertex_handle,
            CGAL::Arr_vertex_index_map<Arrangement >> pred_map(env.number_of_vertices(), index_map);
    boost::dijkstra_shortest_paths(env, v,
                                   boost::vertex_index_map(index_map).
                                           weight_map(weight_map).
                                           distance_map(dist_map).
                                           predecessor_map(pred_map));

    std::vector <Arrangement::Vertex_handle> path;
    Arrangement::Vertex_handle current = w;
    while (current != v) {
        path.push_back(current);
        current = pred_map[current];
    }
    path.push_back(current);

    for (auto p : path) {
        std::cerr << p->point() << ", ";
    }
    std::cerr << std::endl;

}

void simplify_moves(std::vector <Move> &moves) {
    std::vector <Move> newMoves;

    std::vector <Move> simplified;
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