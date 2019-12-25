#include "edge_weight.h"

#include <CGAL/squared_distance_2.h>

typedef boost::property_map<MotionGraph, boost::edge_weight_t>::type WeightMap;

static void edge_weight_constant(MotionGraph &G_i, WeightMap &w) {
    MotionGraphEdgeIterator e_i, e_end;
    for (boost::tie(e_i, e_end) = boost::edges(G_i); e_i != e_end; ++e_i) {
        boost::put(w, *e_i, Kernel::FT(1));
    }
}

static void edge_weight_euclidean(MotionGraph &G_i, WeightMap &w) {
    MotionGraphEdgeIterator e_i, e_end;
    for (boost::tie(e_i, e_end) = boost::edges(G_i); e_i != e_end; ++e_i) {
        const MotionGraphVertex &source = G_i[boost::source(*e_i, G_i)];
        const MotionGraphVertex &target = G_i[boost::target(*e_i, G_i)];
        const Point &p = source.configuration->getPoint();
        const Point &q = target.configuration->getPoint();

        boost::put(w, *e_i, CGAL::sqrt(CGAL::squared_distance(p, q)));
    }
}

static void edge_weight_euclidean_squared(MotionGraph &G_i, WeightMap &w) {
    MotionGraphEdgeIterator e_i, e_end;
    for (boost::tie(e_i, e_end) = boost::edges(G_i); e_i != e_end; ++e_i) {
        const MotionGraphVertex &source = G_i[boost::source(*e_i, G_i)];
        const MotionGraphVertex &target = G_i[boost::target(*e_i, G_i)];
        const Point &p = source.configuration->getPoint();
        const Point &q = target.configuration->getPoint();

        boost::put(w, *e_i, CGAL::squared_distance(p, q));
    }
}

template <typename T>
constexpr int sign(T val) {
    return ((T(0) < val) - (val < T(0)));
}

static void edge_weight_geodesic(MotionGraph &G_i, WeightMap &w, const Polygon &F_i) {
    struct polygon_point_property_t {
        typedef boost::vertex_property_tag kind;
    };

    typedef const Polygon::X_monotone_curve_2::Point_2 PolygonPoint;
    typedef boost::property<polygon_point_property_t, PolygonPoint> VertexProp;
    typedef boost::property<boost::edge_weight_t, double> EdgeProp;
    typedef boost::undirected_graph<VertexProp, EdgeProp> IntervisibilityGraph;
    typedef boost::graph_traits<IntervisibilityGraph>::vertex_iterator IGVertexIterator;
    typedef boost::graph_traits<IntervisibilityGraph>::vertex_descriptor IGVertexDescriptor;
    typedef boost::property_map<IntervisibilityGraph, polygon_point_property_t>::type PolygonPointMap;

    IntervisibilityGraph intervisibilityGraph;
    for (auto iter = F_i.curves_begin(); iter != F_i.curves_end(); ++iter) {
        IGVertexIterator v_begin, v_i, v_end;
        IGVertexDescriptor v_source, v_target;
        boost::tie(v_begin, v_end) = boost::vertices(intervisibilityGraph);

        // Add source if needed
        v_i = std::find_if(v_begin, v_end, [&](IGVertexDescriptor &vd) {
            PolygonPointMap ppm = boost::get(polygon_point_property_t(), intervisibilityGraph);
            return ppm[vd].equals(iter->source());
        });
        if (v_i == v_end) {
            v_source = intervisibilityGraph.add_vertex(VertexProp(iter->source()));
        } else {
            v_source = *v_i;
        }

        // Add target if needed
        v_i = std::find_if(v_begin, v_end, [&](IGVertexDescriptor &vd) {
            PolygonPointMap ppm = boost::get(polygon_point_property_t(), intervisibilityGraph);
            return ppm[vd].equals(iter->target());
        });
        if (v_i == v_end) {
            v_target = intervisibilityGraph.add_vertex(VertexProp(iter->target()));
        } else {
            v_target = *v_i;
        }

        // Add edge
        if (iter->is_circular()) {
            PolygonPoint &p = iter->left();
            double px = CGAL::to_double(p.x());
            double py = CGAL::to_double(p.y());

            PolygonPoint &q = iter->right();
            double qx = CGAL::to_double(q.x());
            double qy = CGAL::to_double(q.y());

            const Polygon::X_monotone_curve_2::Circle_2 &circle = iter->supporting_circle();
            double radius = CGAL::to_double(CGAL::sqrt(circle.squared_radius()));
            const Kernel::Point_2 &c = circle.center();
            double cx = CGAL::to_double(c.x());
            double cy = CGAL::to_double(c.y());

            double p_theta = atan2(py - cy, px - cx);
            double q_theta = atan2(qy - cy, qx - cx);

            double distance;
            if (p_theta == 0) {
                distance = radius * abs(q_theta);
            } else if (q_theta == 0) {
                distance = radius * abs(p_theta);
            } else {
                CGAL_assertion_msg(sign(p_theta) == sign(q_theta), "Curve not X monotone");
                distance = radius * abs(p_theta - q_theta);
            }

            intervisibilityGraph.add_edge(v_source, v_target, EdgeProp(distance));
        } else {
            PolygonPoint &p = iter->left();
            double px = CGAL::to_double(p.x());
            double py = CGAL::to_double(p.y());

            PolygonPoint &q = iter->right();
            double qx = CGAL::to_double(q.x());
            double qy = CGAL::to_double(q.y());

            double dx = px - qx;
            double dy = py - qy;

            double distance = sqrt((dx * dx) + (dy * dy));
            intervisibilityGraph.add_edge(v_source, v_target, EdgeProp(distance));
        }
    }

    MotionGraphEdgeIterator e_i, e_end;
    for (boost::tie(e_i, e_end) = boost::edges(G_i); e_i != e_end; ++e_i) {
        MotionGraphVertexDescriptor source = boost::source(*e_i, G_i);
        MotionGraphVertexDescriptor target = boost::target(*e_i, G_i);

        // TODO: shortest_path from source to target inside F_i
    }
}

void edge_weight(MotionGraph &G_i, const Polygon &F_i) {
    WeightMap w = boost::get(boost::edge_weight, G_i);

    // Set edge weight
    if constexpr (edgeWeightFcn == CONSTANT) {
        edge_weight_constant(G_i, w);
    } else if constexpr (edgeWeightFcn == EUCLIDEAN) {
        edge_weight_euclidean(G_i, w);
    } else if constexpr (edgeWeightFcn == EUCLIDEAN_SQUARED) {
        edge_weight_euclidean_squared(G_i, w);
    } else if constexpr (edgeWeightFcn == GEODESIC) {
        edge_weight_geodesic(G_i, w, F_i);
    }
}

void edge_weight(InterferenceForest &G) {
    InterferenceForestVertexIterator v_i, v_end;
    for (boost::tie(v_i, v_end) = boost::vertices(G); v_i != v_end; ++v_i) {
        MotionGraph &G_i = G[*v_i].motionGraph;
        const Polygon &F_i = *G[*v_i].freeSpaceComponent;

        edge_weight(G_i, F_i);
    }
}