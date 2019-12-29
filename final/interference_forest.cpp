#include "interference_forest.h"

#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/filtered_graph.hpp>

#include "edge_weight.h"

typedef boost::property_map<MotionGraph, boost::edge_weight_t>::type WeightMap;
typedef boost::property_map<MotionGraph, boost::vertex_index_t>::type VertexIndexMap;
typedef boost::property_map<MotionGraph, boost::edge_index_t>::type EdgeIndexMap;

static MotionGraphVertexDescriptor get_leaf(MotionGraph &G_i) {
    boost::graph_traits<MotionGraph>::vertex_iterator v_i, v_end;
    MotionGraphVertexDescriptor vd = nullptr;
    for (boost::tie(v_i, v_end) = boost::vertices(G_i); v_i != v_end; ++v_i) {
        if (boost::degree(*v_i, G_i) <= 1) {
            switch (G_i[*v_i].color) {
                case PURPLE:
                    return *v_i;
                case RED:
                    if (vd == nullptr)
                        vd = *v_i;
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

        typename boost::graph_traits<MotionGraph>::out_edge_iterator e_i, e_end;
        for (boost::tie(e_i, e_end) = boost::out_edges(u, graph); e_i != e_end; ++e_i) {
            const MotionGraphVertexDescriptor &target = boost::target(*e_i, graph);
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

    for (MotionGraphVertexDescriptor current = w; current != v; current = T_g[current].predecessor) {
        if (n.configuration->isStart()) {
            MotionGraphVertex &target = T_g[current];
            MotionGraphVertex &source = T_g[target.predecessor];

            CGAL_assertion(source.hasPebble);
            CGAL_assertion(!target.hasPebble);
            source.hasPebble = false;
            target.hasPebble = true;
            Move m(*source.configuration, *target.configuration);
            moves.push_back(m);
        } else {
            MotionGraphVertex &source = T_g[current];
            MotionGraphVertex &target = T_g[source.predecessor];

            CGAL_assertion(source.hasPebble);
            CGAL_assertion(!target.hasPebble);
            source.hasPebble = false;
            target.hasPebble = true;
            Move m(*source.configuration, *target.configuration);
            moves.push_back(m);
        }
    }

    return true;
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

        // Get all leaves
        std::queue<MotionGraphVertexDescriptor> q;
        boost::graph_traits<MotionGraph>::vertex_iterator vi, vi_end;
        for (boost::tie(vi, vi_end) = boost::vertices(T_g); vi != vi_end; ++vi) {
            const MotionGraphVertexDescriptor &vd = *vi;
            if (boost::out_degree(vd, T_g) == 1) {
                // Leaf
                q.push(vd);
            }
        }

        while (!q.empty()) {
            const MotionGraphVertexDescriptor &vd = q.front();
            q.pop();

            // Count non-visited neighbours
            typename boost::graph_traits<MotionGraph>::out_edge_iterator ei_begin, ei, ei_end;
            boost::tie(ei_begin, ei_end) = boost::out_edges(vd, T_g);
            int nvn = std::count_if(ei_begin, ei_end, [&](MotionGraphEdgeDescriptor ed) {
                CGAL_assertion(boost::source(ed, T_g) == vd);
                MotionGraphVertexDescriptor next = boost::target(ed, T_g);
                return !T_g[next].visited;
            });

            // If we have 0 or 1 non-visited neighbours, compute value
            if (nvn == 0 || nvn == 1) {
                T_g[vd].visited = true;

                if (T_g[vd].configuration->isStart() && T_g[vd].hasPebble) {
                    // Red
                    T_g[vd].value = 1;
                } else if (T_g[vd].configuration->isStart() || T_g[vd].hasPebble) {
                    // Purple
                    T_g[vd].value = 0;
                } else {
                    // Blue
                    T_g[vd].value = -1;
                }

                for (boost::tie(ei, ei_end) = boost::out_edges(vd, T_g); ei != ei_end; ++ei) {
                    CGAL_assertion(boost::source(*ei, T_g) == vd);
                    const MotionGraphVertexDescriptor next = boost::target(*ei, T_g);
                    if (T_g[next].visited) {
                        T_g[vd].value += T_g[next].value;
                    } else {
                        // The parent
                        T_g[vd].predecessor = next;
                        q.push(next);
                    }
                }

                if (T_g[vd].value == 0) {
                    // Root, should not be the only tree with value 0 (though it will be the last one)
                    CGAL_assertion(T_g[vd].predecessor != nullptr);
                    boost::remove_edge(vd, T_g[vd].predecessor, T_g);
                    return false;
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
            Move m(*source.configuration, *target.configuration);
            moves.push_back(m);
        }

        return true;
    }
}

void solve_motion_graph(MotionGraph &G_i, std::vector<Move> &motionSchedule, solve_motion_graph_function s) {
    std::vector<MotionGraphEdgeDescriptor> spanning_tree;
    VertexIndexMap vim = boost::get(boost::vertex_index, G_i);
    WeightMap wm = boost::get(boost::edge_weight, G_i);
    boost::kruskal_minimum_spanning_tree(G_i, std::back_inserter(spanning_tree),
                                         boost::vertex_index_map(vim).weight_map(wm));

    MotionGraphEdgeIterator ei, ei_end, next;
    boost::tie(ei, ei_end) = boost::edges(G_i);
    for (next = ei; ei != ei_end; ei = next) {
        ++next;
        if (std::find(spanning_tree.begin(), spanning_tree.end(), *ei) == spanning_tree.end())
            boost::remove_edge(*ei, G_i);
    }

    while (boost::num_vertices(G_i)) {
        MotionGraphVertexDescriptor vd = get_leaf(G_i);

        bool remove;
        if (s == PEBBLE_GAME) {
            remove = pebble_game_process(G_i, vd, motionSchedule);
        } else if (s == PURPLE_TREE) {
            remove = purple_tree_process(G_i, vd, motionSchedule);
        }

        if (remove) {
            // Remove vertex from G_i
            boost::clear_vertex(vd, G_i);
            boost::remove_vertex(vd, G_i);
        }
    }
}
