#include "interference_forest.h"

#include <boost/graph/kruskal_min_spanning_tree.hpp>

#include "edge_weight.h"

typedef boost::property_map<MotionGraph, boost::edge_weight_t>::type WeightMap;
typedef boost::property_map<MotionGraph, boost::vertex_index_t>::type VertexIndexMap;

static void create_spanning_tree(MotionGraph &G_i) {
    std::vector<MotionGraphEdgeDescriptor> spanning_tree;
    VertexIndexMap vim = boost::get(boost::vertex_index, G_i);
    WeightMap wm = boost::get(boost::edge_weight, G_i);
    boost::kruskal_minimum_spanning_tree(G_i, std::back_inserter(spanning_tree), boost::vertex_index_map(vim).weight_map(wm));

    boost::remove_edge_if([&](MotionGraphEdgeDescriptor e) {
        return (std::find(spanning_tree.begin(), spanning_tree.end(), e) == spanning_tree.end());
    }, G_i);
}

static MotionGraphVertexDescriptor get_leaf(MotionGraph &G_i) {
    boost::graph_traits<MotionGraph>::vertex_iterator v_i, v_end;
    MotionGraphVertexDescriptor vd = nullptr;
    for (boost::tie(v_i, v_end) = boost::vertices(G_i); v_i != v_end; ++v_i) {
        if (boost::degree(*v_i, G_i) <= 1) {
            switch (G_i[*v_i].color) {
                case PURPLE:
                    return *v_i;
                case RED:
                    if (vd != nullptr)
                        break;
                case BLUE:
                    vd = *v_i;
                    break;
            }
        }
    }
    return vd;
}

static void reset_motion_graph(MotionGraph &graph) {
    typename boost::graph_traits<MotionGraph>::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(graph);
         vi != vi_end;
         ++vi) {
        graph[*vi].value = 0;
        graph[*vi].visited = false;
        graph[*vi].predecessor = nullptr;
    }
}

static MotionGraphVertexDescriptor find_shortest_path(MotionGraph &graph,
                                               const MotionGraphVertexDescriptor &start,
                                               const std::function<bool(
                                                       MotionGraphVertexDescriptor)> &predicate) {

    if (predicate(start)) {
        return start;
    }

    std::queue<MotionGraphVertexDescriptor> queue;

    reset_motion_graph(graph);

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

bool pebble_game_process(MotionGraph &T_g, MotionGraphVertexDescriptor &v, std::vector<Move> &moves) {
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
            moves.emplace_back(*source.configuration, *target.configuration);
        } else {
            MotionGraphVertex &source = T_g[current];
            MotionGraphVertex &target = T_g[source.predecessor];

            CGAL_assertion(source.hasPebble);
            CGAL_assertion(!target.hasPebble);
            source.hasPebble = false;
            target.hasPebble = true;
            moves.emplace_back(*source.configuration, *target.configuration);
        }
    }

    return true;
}

void solve_motion_graph_pebble_game(MotionGraph &G_i, const Polygon &F_i, std::vector<Move> &motionSchedule) {
    create_spanning_tree(G_i);

    while (boost::num_vertices(G_i)) {
        MotionGraphVertexDescriptor vd = get_leaf(G_i);

        if (pebble_game_process(G_i, vd, motionSchedule)) {
            // Remove vertex from G_i
            boost::clear_vertex(vd, G_i);
            boost::remove_vertex(vd, G_i);
        }
    }
}

bool purple_tree_process(MotionGraph &T_g, MotionGraphVertexDescriptor &v, std::vector<Move> &moves) {
    MotionGraphVertex &n = T_g[v];

    if (n.configuration->isStart()) {
        // Purple leaf
        if (!n.hasPebble) {
            return true;
        }

        // Red leaf
        reset_motion_graph(T_g);

        std::queue<MotionGraphVertexDescriptor> q;
        n.value = 0;
        q.push(v);
        while (!q.empty()) {
            const MotionGraphVertexDescriptor &vd = q.front();
            q.pop();

            T_g[vd].visited = true;

            typename boost::graph_traits<MotionGraph>::out_edge_iterator ei, ei_end;
            for (boost::tie(ei, ei_end) = boost::out_edges(vd, T_g);
                 ei != ei_end;
                 ++ei) {

                const MotionGraphVertexDescriptor &next = (boost::source(*ei, T_g) != vd) ? boost::source(*ei, T_g) : boost::target(*ei, T_g);

                if (!T_g[next].visited) {
                    if (T_g[next].configuration->isStart()) {
                        T_g[next].value = T_g[vd].value + 1;
                    } else {
                        T_g[next].value = T_g[vd].value - 1;
                    }

                    if (T_g[next].value == 0) {
                        boost::remove_edge(*ei, T_g);
                        return false;
                    } else {
                        q.push(next);
                    }
                }
            }
        }

        CGAL_assertion(false);
        return false;
    } else {
        // Blue leaf
        if (!n.hasPebble) {
            // Find closest pebble
            MotionGraphVertexDescriptor w = find_shortest_path(T_g, v, [&](MotionGraphVertexDescriptor vd) {
                return T_g[vd].hasPebble;
            });

            MotionGraphVertex &source = T_g[w];
            MotionGraphVertex &target = T_g[v];

            CGAL_assertion(source.hasPebble);
            CGAL_assertion(!target.hasPebble);
            source.hasPebble = false;
            target.hasPebble = true;
            moves.emplace_back(*source.configuration, *target.configuration);
        }

        return true;
    }
}

void solve_motion_graph_purple_tree(MotionGraph &G_i, const Polygon &F_i, std::vector<Move> &motionSchedule) {
    create_spanning_tree(G_i);

    while (boost::num_vertices(G_i)) {
        MotionGraphVertexDescriptor vd = get_leaf(G_i);

        if (purple_tree_process(G_i, vd, motionSchedule)) {
            // Remove vertex from G_i
            boost::clear_vertex(vd, G_i);
            boost::remove_vertex(vd, G_i);
        }
    }
}

void solve_interference_forest(InterferenceForest &G,
                               std::vector<Move> &motionSchedule) {
    // Clear the motion schedule
    motionSchedule.clear();

    // Compute edge weights
    edge_weight(G);

    // Topologically ordered solving
    std::queue<InterferenceForestVertexDescriptor> Q;
    typename boost::graph_traits<InterferenceForest>::vertex_iterator v_i, v_end;
    for (boost::tie(v_i, v_end) = boost::vertices(G); v_i != v_end; ++v_i) {
        if (boost::in_degree(*v_i, G) == 0) {
            Q.push(*v_i);
        }
    }
    while (!Q.empty()) {
        InterferenceForestVertexDescriptor &vd = Q.front();
        Q.pop();

        InterferenceForestVertex &v = G[vd];
        MotionGraph &G_i = v.motionGraph;
        const Polygon &F_i = *v.freeSpaceComponent;

        if constexpr (solveMotionGraphFcn == PEBBLE_GAME) {
            solve_motion_graph_pebble_game(G_i, F_i, motionSchedule);
        } else if constexpr (solveMotionGraphFcn == PURPLE_TREE) {
            solve_motion_graph_purple_tree(G_i, F_i, motionSchedule);
        }

        typename boost::graph_traits<InterferenceForest>::out_edge_iterator e_i, e_end;
        for (boost::tie(e_i, e_end) = boost::out_edges(vd, G); e_i != e_end; ++e_i) {
            CGAL_assertion(boost::source(*e_i, G) == vd);
            Q.push(boost::target(*e_i, G));
        }

    }
}
