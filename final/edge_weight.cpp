#include "edge_weight.h"

#include <CGAL/squared_distance_2.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

#include <CGAL/boost/graph/graph_traits_Constrained_Delaunay_triangulation_2.h>
#include <CGAL/boost/graph/dijkstra_shortest_paths.h>

#include <boost/graph/filtered_graph.hpp>
#include <boost/function_output_iterator.hpp>

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

template<typename T>
constexpr int sign(T val) {
    return ((T(0) < val) - (val < T(0)));
}

static void edge_weight_geodesic(MotionGraph &G_i, WeightMap &w, const Polygon &F_i) {
    std::vector<Point> polyline;
    F_i.approximate(boost::make_function_output_iterator([&polyline](const std::pair<double, double> &p) {
        polyline.emplace_back(p.first, p.second);
    }), 10); // Complexity increases with constant factor


    struct FaceInfo {
        FaceInfo() = default;

        bool visited;
        bool in_domain;
    };

    typedef Kernel Gt;
    typedef CGAL::Triangulation_vertex_base_with_id_2<Gt> Tvb;
    typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, Gt> Tfb;
    typedef CGAL::Constrained_triangulation_face_base_2<Gt, Tfb> Ctfb;
    typedef CGAL::Triangulation_data_structure_2<Tvb, Ctfb> Tds;
    typedef CGAL::Exact_predicates_tag ITag;
    typedef CGAL::Constrained_Delaunay_triangulation_2<Gt, Tds, ITag> CDT;

    // Create Constrained Delaunay Triangulation
    CDT cdt;

    // Insert the constraints (polyline)
    cdt.insert_constraint(polyline.begin(), polyline.end(), true);

    // Insert the query points
    MotionGraphVertexIterator v_i, v_end;
    for (boost::tie(v_i, v_end) = boost::vertices(G_i); v_i != v_end; ++v_i) {
        const Point &point = G_i[*v_i].configuration->getPoint();
        cdt.insert(point);
    }

    // Compute whether faces are inside our outside of the polygon
    for (CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
        FaceInfo &info = it->info();
        info.in_domain = false;
        info.visited = false;
    }

    CDT::Face_handle fh = cdt.infinite_face();
    fh->info().visited = true;
    std::queue<CDT::Face_handle> Q;
    Q.push(fh->neighbor(0));
    while (!Q.empty()) {
        fh = Q.front();
        Q.pop();

        FaceInfo &info = fh->info();
        info.visited = true;

        for (int i = 0; i < 3; i++) {
            CDT::Face_handle neighbor = fh->neighbor(i);
            FaceInfo &neighborInfo = neighbor->info();
            if (!neighborInfo.visited) {
                Q.push(neighbor);
            } else if (fh->is_constrained(i)) {
                info.in_domain = !neighborInfo.in_domain;
            }
        }
    }

    struct Inside_polygon {
        Inside_polygon() = default;

        bool operator()(const CDT::Edge &e) const {
            CDT::Face_handle fh = e.first;
            if (fh->info().in_domain) {
                return true;
            }
            return fh->is_constrained(e.second);
        }
    };

    Inside_polygon filter;
    typedef boost::filtered_graph<CDT, Inside_polygon> Polygon_triangulation;
    Polygon_triangulation triangulation(cdt, filter);

    typedef boost::graph_traits<Polygon_triangulation>::vertex_descriptor vertex_descriptor;
    typedef boost::graph_traits<Polygon_triangulation>::vertex_iterator vertex_iterator;

    typedef std::map<vertex_descriptor, int> VertexIndexMap;
    typedef boost::associative_property_map<VertexIndexMap> VertexIdPropertyMap;
    VertexIndexMap vertex_id_map;
    VertexIdPropertyMap vertex_index_pmap(vertex_id_map);

    // Associate indices to vertices
    int index = 0;
    vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(triangulation); vi != vi_end; ++vi) {
        vertex_descriptor vd = *vi;
        vertex_id_map[vd] = index++;
    }

    std::vector<vertex_descriptor> predecessor(boost::num_vertices(triangulation));
    boost::iterator_property_map<std::vector<vertex_descriptor>::iterator, VertexIdPropertyMap> predecessor_pmap(
            predecessor.begin(), vertex_index_pmap);

    std::vector<Kernel::FT> distance(boost::num_vertices(triangulation));
    boost::iterator_property_map<std::vector<Kernel::FT>::iterator, VertexIdPropertyMap> distance_pmap(distance.begin(),
                                                                                                       vertex_index_pmap);

    MotionGraphEdgeIterator e_i, e_end;
    for (boost::tie(e_i, e_end) = boost::edges(G_i); e_i != e_end; ++e_i) {
        const Point &source = G_i[boost::source(*e_i, G_i)].configuration->getPoint();
        const Point &target = G_i[boost::target(*e_i, G_i)].configuration->getPoint();

        boost::tie(vi, vi_end) = boost::vertices(triangulation);
        vertex_descriptor vd_source = *std::find_if(vi, vi_end, [&](vertex_descriptor vd) {
            return source == vd->point();
        });

        boost::tie(vi, vi_end) = boost::vertices(triangulation);
        vertex_descriptor vd_target = *std::find_if(vi, vi_end, [&](vertex_descriptor vd) {
            return target == vd->point();
        });

        boost::dijkstra_shortest_paths(triangulation, vd_source,
                                       boost::vertex_index_map(vertex_index_pmap)
                                               .predecessor_map(predecessor_pmap)
                                               .distance_map(distance_pmap));
        boost::put(w, *e_i, CGAL::to_double(distance[vertex_id_map[vd_target]]));
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