#include "benchmark.h"

#include <iostream>

#include <mrmp.h>
#include <InterferenceForest.h>

#include <CGAL/IO/File_writer_wavefront.h>
#include <CGAL/Inverse_index.h>

#include <boost/graph/graphviz.hpp>

#include "timeit.h"

void print(const std::string &filename, const Workspace &W) {
    std::ofstream stream(filename);
    CGAL::File_writer_wavefront writer{};
    writer.write_header(stream, W.size(), W.size(), 1);

    typedef Workspace::Vertex_const_iterator    VCI;
    for (VCI iter = W.vertices_begin(); iter != W.vertices_end(); ++iter) {
        writer.write_vertex(CGAL::to_double(iter->x()), CGAL::to_double(iter->y()), 0.0);
    }

    typedef CGAL::Inverse_index<VCI>            Index;
    Index index(W.vertices_begin(), W.vertices_end());
    writer.write_facet_header();
    writer.write_facet_begin(W.size());
    for (VCI iter = W.vertices_begin(); iter != W.vertices_end(); ++iter) {
        writer.write_facet_vertex_index(index[iter]);
    }
    writer.write_facet_end();
    writer.write_footer();

    stream.close();
}

void print(const std::string &filename, const FreeSpace &F) {
    std::ofstream stream(filename);
    for (const Polygon &F_i : F) {
        stream << F_i << std::endl;
    }
    stream.close();
}

void print(const std::string &filename, const MotionGraph &G_i) {
    std::ofstream stream(filename);
    boost::write_graphviz(stream, G_i);
    stream.close();
}

void print(const std::string &filename, const InterferenceForest &G) {
    std::ofstream stream(filename);
    boost::write_graphviz(stream, G);
    stream.close();
}

void benchmark(const Workspace &W, const ConfigurationSet &U, const std::string &name) {
    print(name + "_W.obj", W);

    // Generate Free space components
    timeit t;

    timeit t1;
    FreeSpace F;
    generate_free_space(W, F);
    std::cout << "Generate free space: " << t1.elapsed().count() << " s" << std::endl;
    print(name + "_F.obj", F);

    t1 = timeit();
    InterferenceForest G;
    int index = 0;
    for (const Polygon &F_i : F) {
        timeit t2;
        // Generate motion graph
        InterferenceForestVertexDescriptor vd = G.add_vertex();
        InterferenceForestVertex &v = G[vd];
        MotionGraph &G_i = v.motionGraph;
        v.freeSpaceComponent = &F_i;
        v.index = index++;
        v.visited = false;

        generate_motion_graph(F_i, U, G_i);
        std::cout << "Generate motion graph: " << t2.elapsed().count() << " s" << std::endl;
        print(name + "_G" + std::to_string(v.index) + ".gv", G_i);
    }

    // DIF edges
    typename boost::graph_traits<InterferenceForest>::vertex_iterator v_begin, vii, vij, v_end;
    boost::tie(v_begin, v_end) = boost::vertices(G);

    for (vii = v_begin; vii != v_end; vii++) {
        const InterferenceForestVertexDescriptor &vdi = *vii;
        const InterferenceForestVertex &vi = G[vdi];
        const Polygon &G_i = *vi.freeSpaceComponent;

        for (vij = v_begin; vij != vii; vij++) {
            const InterferenceForestVertexDescriptor &vdj = *vij;
            const InterferenceForestVertex &vj = G[vdj];
            const Polygon &G_j = *vj.freeSpaceComponent;

            for (const Configuration &c : U) {
                if (check_inside(c.getPoint(), G_i) && do_intersect(D<2>(c.getPoint()), G_j)) {
                    if (c.isStart()) {
                        G.add_edge(vdi, vdj);
                    } else {
                        G.add_edge(vdj, vdi);
                    }
                }
                if (check_inside(c.getPoint(), G_j) && do_intersect(D<2>(c.getPoint()), G_i)) {
                    if (c.isStart()) {
                        G.add_edge(vdj, vdi);
                    } else {
                        G.add_edge(vdi, vdj);
                    }
                }
            }
        }
    }
    std::cout << "Generate interference forest: " << t1.elapsed().count() << " s" << std::endl;
    print(name + "_G.gv", G);

    t1 = timeit();
    /// Topological sorting (Kahn's algorithm)
    // L <- Empty list that will contain the sorted elements
    std::list<InterferenceForestVertexDescriptor> L;
    // S <- Set of all nodes with no incoming edge
    std::set<InterferenceForestVertexDescriptor> S;

    typename boost::graph_traits<InterferenceForest>::vertex_iterator vi;
    for (boost::tie(vi, v_end) = boost::vertices(G); vi != v_end; ++vi) {
        InterferenceForestVertexDescriptor u = *vi;
        InterferenceForestVertex &v = G[u];
        v.visited = false;
        if (boost::in_degree(u, G) == 0) {
            S.insert(u);
        }
    }

    while (!S.empty()) {
        // Remove a node n from S
        auto iter = S.begin();
        const InterferenceForestVertexDescriptor n = *iter;
        G[n].visited = true;
        S.erase(iter);

        // Add n to tail of L
        L.push_back(n);


        // For each node m with an edge e from n to m
        typename boost::graph_traits<InterferenceForest>::out_edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(n, G); ei != ei_end; ++ei) {
            CGAL_assertion(boost::source(*ei, G) == n);
            const InterferenceForestVertexDescriptor m = boost::target(*ei, G);

            // If m has no other incoming edges
            typename boost::graph_traits<InterferenceForest>::in_edge_iterator ej, ej_end;
            for (boost::tie(ej, ej_end) = boost::in_edges(m, G); ej != ej_end; ++ej) {
                CGAL_assertion(boost::target(*ej, G) == m);
                const InterferenceForestVertexDescriptor l = boost::source(*ej, G);
                if (!G[l].visited) {
                    break;
                }
            }
            if (ej == ej_end) {
                S.insert(m);
            }
        }
    }

    // If G has a cycle, L will not contain every vertex in G
    CGAL_assertion(boost::num_vertices(G) == L.size());

    std::vector<Move> moves;
    for (const InterferenceForestVertexDescriptor &n : L) {
        timeit t2;
        InterferenceForestVertex &v = G[n];
        MotionGraph &G_i = v.motionGraph;
        solve_motion_graph(G_i, moves);
        std::cout << "Solving motion graph: " << t2.elapsed().count() << " s" << std::endl;
    }
    std::cout << "Solve interference forest: " << t1.elapsed().count() << " s" << std::endl;

    std::cout << "Total time: " << t.elapsed().count() << " s" << std::endl;
}