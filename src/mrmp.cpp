#include "mrmp.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/approximated_offset_2.h>

#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

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

    for (const Configuration &u : U) {
        if (check_inside(u.getPoint(), F_i)) {
            MotionGraphVertexDescriptor v = boost::add_vertex(G_i);
            G_i[v].configuration = &u;
            G_i[v].hasPebble = u.isStart();
        }
    }

    General_polygon_set gps = remove_start_target_configs(F_i, U);
    std::vector<Polygon_with_holes> F_star;
    gps.polygons_with_holes(std::back_inserter(F_star));

    // Add edges between vertices in H_i
    for (const Polygon_with_holes &F_star_i : F_star) {
        std::vector<MotionGraphVertexDescriptor> B_i, H_i;
        const Polygon &boundary = F_star_i.outer_boundary();

        typename boost::graph_traits<MotionGraph>::vertex_iterator vi, v_end;
        for (boost::tie(vi, v_end) = boost::vertices(G_i); vi != v_end; ++vi) {
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

    std::queue<MotionGraphVertexDescriptor> queue;

    typename boost::graph_traits<MotionGraph>::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(graph); vi != vi_end; ++vi) {
        graph[*vi].visited = false;
        graph[*vi].predecessor = nullptr;
    }

    graph[start].visited = true;
    graph[start].predecessor = start;
    queue.push(start);

    while (!queue.empty()) {
        MotionGraphVertexDescriptor u = queue.front();
        queue.pop();

        typename boost::graph_traits<MotionGraph>::out_edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(u, graph); ei != ei_end; ++ei) {
            MotionGraphVertexDescriptor target = boost::target(*ei, graph);
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

void solve_motion_graph(const MotionGraph &G_i,
                        std::list<Move> &moves) {

    std::vector<MotionGraphEdgeDescriptor> spanning_tree;
    boost::kruskal_minimum_spanning_tree(G_i, std::back_inserter(spanning_tree));

    MotionGraph T_g(G_i);
    boost::remove_edge_if([&](const MotionGraphEdgeDescriptor &ed) {
        return std::find(spanning_tree.begin(), spanning_tree.end(), ed) == spanning_tree.end();
    }, T_g);

    while (boost::num_vertices(T_g)) {
        // Find a leave (preferable target)
        boost::graph_traits<MotionGraph>::vertex_iterator v_i, v_j, v_end;
        for (boost::tie(v_i, v_end) = boost::vertices(T_g); v_i != v_end; ++v_i) {
            if (boost::degree(*v_i, T_g) <= 1) {
                v_j = v_i;
                if (!T_g[*v_i].configuration->isStart()) {
                    break;
                }
            }
        }

        CGAL_assertion(v_i != v_end);

        const MotionGraphVertexDescriptor &v = *v_j;
        const MotionGraphVertex &n = T_g[v];

        // Find closest pebble w with a pebble
        MotionGraphVertexDescriptor w = find_shortest_path(T_g, *v_j, [&](MotionGraphVertexDescriptor vd) {
            if (!n.configuration->isStart()) {
                return T_g[vd].hasPebble;
            } else {
                return !T_g[vd].hasPebble;
            }
        });

        std::vector<MotionGraphVertexDescriptor> path;
        for (MotionGraphVertexDescriptor predecessor = w;
             predecessor != T_g[predecessor].predecessor;
             predecessor = T_g[predecessor].predecessor) {
            path.push_back(predecessor);
        }

        for (auto iter = path.begin(); iter != path.end(); ++iter) {
            const MotionGraphVertexDescriptor &vd = *iter;
            MotionGraphVertex &first = T_g[vd];
            if (vd != v) {
                // This is not the last vertex in our path
                auto iter_next = iter + 1;
                CGAL_assertion(iter_next != path.end());

                const MotionGraphVertexDescriptor &next = *iter_next;
                MotionGraphVertex &second = T_g[next];

                if (n.configuration->isStart()) {
                    CGAL_assertion(second.hasPebble);
                    CGAL_assertion(!first.hasPebble);
                    second.hasPebble = false;
                    first.hasPebble = true;
                    moves.emplace_back(second.configuration, first.configuration);
                } else {
                    CGAL_assertion(first.hasPebble);
                    CGAL_assertion(!second.hasPebble);
                    first.hasPebble = false;
                    second.hasPebble = true;
                    moves.emplace_back(first.configuration, second.configuration);
                }
            }
        }

        CGAL_assertion(n.hasPebble != n.configuration->isStart());

        // Remove vertex from T_g
        boost::clear_vertex(v, T_g);
        boost::remove_vertex(v, T_g);
    }
}


void get_shortest_path(const Move &move,
                       const Polygon &f,
                       std::vector<const Configuration *> &robots) {

    auto iter = robots.begin();
    for (; iter != robots.end(); ++iter) {
        if ((*iter)->getPoint() == move.first->getPoint()) {
            break;
        }
    }

    CGAL_assertion(iter != robots.end());

    const Point &source = move.first->getPoint();
    const Point &target = move.second->getPoint();
    // TODO: Get shortest path in f from source to target

    // Update robots
    robots.erase(iter);
    robots.push_back(move.second);
}