#include "benchmark.h"

#include <iostream>

#include <mrmp.h>
#include <InterferenceForest.h>

#include <boost/graph/graphviz.hpp>

#include "timeit.h"

void print(const std::string &filename, const Workspace &W) {
    std::ofstream stream(filename);
    stream << W;
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
        print(name + "_G" + std::to_string(v.index) + ".obj", G_i);
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
    print(name + "_G.obj", G);

    std::cout << "Total time: " << t.elapsed().count() << " s" << std::endl;
}