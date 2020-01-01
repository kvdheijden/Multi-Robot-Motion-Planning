#include <iostream>
#include <vector>
#include <regex>

#include <boost/filesystem.hpp>

#include "cgal_types.h"

#include "Configuration.h"
#include "free_space.h"
#include "motion_graph.h"
#include "interference_forest.h"
#include "edge_weight.h"
#include "timeit.h"

//#define SAVE_INTERMEDIATES

static bool do_intersect(const Polygon::X_monotone_curve_2 &curve1, const Polygon::X_monotone_curve_2 &curve2) {
    std::list<CGAL::Object> objects;
    curve1.intersect(curve2, std::back_inserter(objects));
    return !objects.empty();
}

static bool do_intersect(const Polygon &pgn1, const Polygon::X_monotone_curve_2 &curve) {
    for (auto iter = pgn1.curves_begin(); iter != pgn1.curves_end(); ++iter) {
        if (do_intersect(*iter, curve)) {
            return true;
        }
    }
    return false;
}

static bool do_intersect(const Polygon &pgn1, const Polygon &pgn2) {
    for (auto iter = pgn2.curves_begin(); iter != pgn2.curves_end(); ++iter) {
        if (do_intersect(pgn1, *iter)) {
            return true;
        }
    }
    return false;
}

void benchmark(std::ostream &stream,
               const Workspace &W,
               const std::vector<Configuration> &S,
               const std::vector<Configuration> &T,
               const boost::filesystem::path &path,
               edge_weight_fcn e,
               solve_motion_graph_function solver) {
    timeit timer;

    FreeSpace F;
    do {
        timeit t1;
        generate_free_space(W, F);
        std::chrono::duration<double> time = t1.elapsed();
        std::cerr << "[TIMEIT] Generate Free Space: " << time.count() << " s." << std::endl;
        stream << time.count() << ","; // genF
    } while (false);

    InterferenceForest G;
    do {
        timeit t1;
        stream << "\"";
        int i = 0;
        for (const Polygon &F_i : F) {
            InterferenceForestVertexDescriptor vd = boost::add_vertex(G);
            G[vd].freeSpaceComponent = &F_i;
            MotionGraph &G_i = G[vd].motionGraph;
            G[vd].index = i++;

            timeit t2;
            generate_motion_graph(F_i, S, T, G_i);
            std::chrono::duration<double> time = t2.elapsed();
            std::cerr << "[TIMEIT] Generate Motion graph: " << time.count() << " s." << std::endl;
            std::cerr << "[COMPLEXITY] |V| = " << boost::num_vertices(G_i) << ", |E| = " << boost::num_edges(G_i) << std::endl;
            stream << "(" << time.count() << "/" << boost::num_vertices(G_i) << "/" << boost::num_edges(G_i) << "),";
        }
        stream << "\",";

        typename boost::graph_traits<InterferenceForest>::vertex_iterator v_begin, vii, vij, v_end;
        boost::tie(v_begin, v_end) = boost::vertices(G);
        for (boost::tie(vii, v_end) = boost::vertices(G); vii != v_end; vii++) {
            const InterferenceForestVertexDescriptor &vdi = *vii;
            const InterferenceForestVertex &vi = G[vdi];
            const Polygon &G_i = *vi.freeSpaceComponent;

            for (vij = v_begin; vij != vii; vij++) {
                const InterferenceForestVertexDescriptor &vdj = *vij;
                const InterferenceForestVertex &vj = G[vdj];
                const Polygon &G_j = *vj.freeSpaceComponent;

                for (const Configuration &s : S) {
                    if (check_inside(s.getPoint(), G_i) && do_intersect(D<2>(s.getPoint()), G_j)) {
                        G.add_edge(vdi, vdj);
                    }
                    if (check_inside(s.getPoint(), G_j) && do_intersect(D<2>(s.getPoint()), G_i)) {
                        G.add_edge(vdj, vdi);
                    }
                }

                for (const Configuration &t : T) {
                    if (check_inside(t.getPoint(), G_i) && do_intersect(D<2>(t.getPoint()), G_j)) {
                        G.add_edge(vdj, vdi);
                    }
                    if (check_inside(t.getPoint(), G_j) && do_intersect(D<2>(t.getPoint()), G_i)) {
                        G.add_edge(vdi, vdj);
                    }
                }
            }
        }

        std::chrono::duration<double> time = t1.elapsed();
        std::cerr << "[TIMEIT] Generate Interference Forest: " << time.count() << " s." << std::endl;
        std::cerr << "[COMPLEXITY] |V| = " << boost::num_vertices(G) << ", |E| = " << boost::num_edges(G) << std::endl;
        stream << "(" << time.count() << "/" << boost::num_vertices(G) << "/" << boost::num_edges(G) << "),";
    } while (false);

#ifdef SAVE_INTERMEDIATES
    InterferenceForestVertexIterator vj, vj_end;
    for (boost::tie(vj, vj_end) = boost::vertices(G); vj != vj_end; ++vj) {
        {
            InterferenceForestVertexDescriptor &vd = *vj;
            boost::filesystem::ofstream f(path / ("F_" + std::to_string(G[vd].index) + ".txt"));
            boost::filesystem::ofstream g(path / ("G_" + std::to_string(G[vd].index) + ".dot"));

            const Polygon &F_i = *G[vd].freeSpaceComponent;
            const MotionGraph &G_i = G[vd].motionGraph;

            f << F_i;
            g << "graph G_" << std::to_string(G[vd].index) << " {" << std::endl;
            boost::graph_traits<MotionGraph>::vertex_iterator vi, vi_end;
            for (boost::tie(vi, vi_end) = boost::vertices(G_i); vi != vi_end; ++vi) {
                const Configuration &c = *G_i[*vi].configuration;
                g << "\t" << c.to_string() << "[color=\"" << (c.isStart() ? "green" : "purple") << "\"];" << std::endl;
            }
            boost::graph_traits<MotionGraph>::edge_iterator ei, ei_end;
            for (boost::tie(ei, ei_end) = boost::edges(G_i); ei != ei_end; ++ei) {
                const Configuration &source = *G_i[boost::source(*ei, G_i)].configuration;
                const Configuration &target = *G_i[boost::target(*ei, G_i)].configuration;
                g << "\t" << source.to_string() << " -- " << target.to_string() << ";" << std::endl;
            }
            g << "}" << std::endl;
        }

        {
            boost::filesystem::ofstream g(path / ("G.dot"));
            g << "graph G {" << std::endl;
            boost::graph_traits<InterferenceForest>::vertex_iterator vi, vi_end;
            for (boost::tie(vi, vi_end) = boost::vertices(G); vi != vi_end; ++vi) {
                const InterferenceForestVertex &v = G[*vi];
                g << "\t" << v.to_string() << "[color=\"black\"];" << std::endl;
            }
            boost::graph_traits<InterferenceForest>::edge_iterator ei, ei_end;
            for (boost::tie(ei, ei_end) = boost::edges(G); ei != ei_end; ++ei) {
                const InterferenceForestVertex &source = G[boost::source(*ei, G)];
                const InterferenceForestVertex &target = G[boost::target(*ei, G)];
                g << "\t" << source.to_string() << " -- " << target.to_string() << ";" << std::endl;
            }
            g << "}" << std::endl;
        }
    }
#else
    (void) path;
#endif

    std::vector<Move> motionSchedule;
    do {
        timeit t1;

        // Compute edge weights
        edge_weight(G, e);
        std::chrono::duration<double> time = t1.elapsed();
        std::cerr << "[TIMEIT] Generate Edge weights: " << time.count() << " s." << std::endl;
        stream << time.count() << ",";
    } while (false);

    do {
        timeit t1;
        stream << "\"";
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

            timeit t2;
            solve_motion_graph(G_i, motionSchedule, solver);
            std::chrono::duration<double> time = t2.elapsed();
            std::cerr << "[TIMEIT] Solve Motion graph: " << time.count() << " s." << std::endl;
            stream << time.count() << ",";

            typename boost::graph_traits<InterferenceForest>::out_edge_iterator e_i, e_end;
            for (boost::tie(e_i, e_end) = boost::out_edges(vd, G); e_i != e_end; ++e_i) {
                CGAL_assertion(boost::source(*e_i, G) == vd);
                Q.push(boost::target(*e_i, G));
            }
        }
        stream << "\",";

        std::chrono::duration<double> time = t1.elapsed();
        std::cerr << "[TIMEIT] Solve Interference Forest: " << time.count() << " s." << std::endl;
        stream << time.count() << ",";
    } while (false);

    std::chrono::duration<double> time = timer.elapsed();
    std::cerr << "[TIMEIT] Total: " << time.count() << " s." << std::endl;
    stream << time.count() << ",";

    std::cerr << "Motion schedule: {" << std::endl;
    stream << "\"{";
    for (const Move &m : motionSchedule) {
        stream << "(" << m.first.get().to_string() << ", " << m.second.get().to_string() << "),";
        std::cerr << "\t(" << m.first.get().to_string() << ", " << m.second.get().to_string() << ")," << std::endl;
    }
    stream << "}\"";
    std::cerr << "}" << std::endl;
}

void run(const boost::filesystem::path &path, edge_weight_fcn e, solve_motion_graph_function s) {
    const std::regex r("n([0-9]+)_m([0-9]+)");
    std::smatch match;

    std::vector<boost::filesystem::path> v;
    for (const boost::filesystem::directory_entry &entry : boost::filesystem::directory_iterator(path))
        if (boost::filesystem::is_directory(entry.path()))
            v.push_back(entry.path());

    std::sort(v.begin(), v.end(), [&](boost::filesystem::path &a, boost::filesystem::path &b) {
        int na, nb, ma, mb;

        if (!std::regex_match(a.filename().string(), match, r))
            return true;

        na = std::stoi(match[1].str());
        ma = std::stoi(match[2].str());

        if (!std::regex_match(b.filename().string(), match, r))
            return false;

        nb = std::stoi(match[1].str());
        mb = std::stoi(match[2].str());

        return (na == nb) ? (ma < mb) : (na < nb);
    });

    std::string bench("benchmark_");
    switch (s) {
        case PEBBLE_GAME:
            bench += "pebble_";
            break;
        case PURPLE_TREE:
            bench += "purple_";
            break;
        default:
            exit(1);
    }
    switch (e) {
        case CONSTANT:
            bench += "constant";
            break;
        case EUCLIDEAN:
            bench += "euclidean";
            break;
        case EUCLIDEAN_SQUARED:
            bench += "squared_euclidean";
            break;
        case GEODESIC:
            bench += "geodesic";
            break;
        default:
            exit(1);
    }
    bench += ".csv";

    boost::filesystem::ofstream stream(path / bench);
    stream << "n,m,generateFreeSpace(seconds),generateMotionGraph(seconds/|V|/|E|),generateInterferenceForest(seconds/|V|/|E|),computeEdgeWeight(seconds),solveMotionGraph(seconds),solveInterferenceForest(seconds),Total(seconds),result" << std::endl;
    for (auto &&x : v) {
        const boost::filesystem::path w = x / "w.txt";
        if (!boost::filesystem::is_regular_file(w)) {
            std::cerr << "No workspace file found in " << x.filename() << std::endl;
            continue;
        }

        if (!std::regex_match(x.filename().string(), match, r)) {
            std::cerr << "Failed to determine n/m in " << x.filename() << std::endl;
            continue;
        }
        std::size_t n = std::stoi(match[1].str());
        std::size_t m = std::stoi(match[2].str());

        std::cerr << "Starting benchmarking of " << x.filename() << std::endl;

        Workspace W;
        boost::filesystem::ifstream(w) >> W;

        Point p;
        std::vector<Configuration> S, T;
        for (std::size_t i = 0; i < m; i++) {
            const boost::filesystem::path s_i = x / ("s_" + std::to_string(i) + ".txt");
            if (!boost::filesystem::is_regular_file(s_i)) {
                std::cerr << "Failed to open " << s_i.filename() << " in " << x.filename() << std::endl;
                break;
            }
            const boost::filesystem::path t_i= x / ("t_" + std::to_string(i) + ".txt");
            if (!boost::filesystem::is_regular_file(t_i)) {
                std::cerr << "Failed to open " << t_i.filename() << " in " << x.filename() << std::endl;
                break;
            }

            boost::filesystem::ifstream(s_i) >> p;
            S.emplace_back(std::move(p), true, i);

            boost::filesystem::ifstream(t_i) >> p;
            T.emplace_back(std::move(p), false, i);
        }

        if (W.size() != n) {
            std::cerr << "Invalid workspace size for " << x.filename() << std::endl;
            continue;
        }
        if (S.size() != m) {
            std::cerr << "Invalid starting configuration size for " << x.filename() << std::endl;
            continue;
        }
        if (T.size() != m) {
            std::cerr << "Invalid target configuration size for " << x.filename() << std::endl;
            continue;
        }

        stream << n << "," << m << ",";
        benchmark(stream, W, S, T, x, e, s);
        stream << std::endl;
        std::cerr << "Done benchmarking of " << x.filename() << std::endl << std::endl;
    }
}

int main(int argc, char *argv[]) {
    (void) argc;
    (void) argv;

    constexpr edge_weight_fcn E[] = {
            CONSTANT,
            EUCLIDEAN,
            EUCLIDEAN_SQUARED,
            GEODESIC
    };
    constexpr solve_motion_graph_function S[] = {
            PEBBLE_GAME,
            PURPLE_TREE
    };

    for (edge_weight_fcn e : E) {
        for (solve_motion_graph_function s : S) {
            run(boost::filesystem::path("/home/koen/Documents/datasets/random"), e, s);
            run(boost::filesystem::path("/home/koen/Documents/datasets/grid"), e, s);
            run(boost::filesystem::path("/home/koen/Documents/datasets/zigzag"), e, s);
            for (const boost::filesystem::directory_entry &entry : boost::filesystem::directory_iterator(boost::filesystem::path("/home/koen/Documents/datasets/corridor"))) {
                const boost::filesystem::path &p = entry.path();

                if (p.filename_is_dot_dot() || p.filename_is_dot())
                    continue;

                if (boost::filesystem::is_directory(p)) {
                    run(p, e, s);
                }
            }
            run(boost::filesystem::path("/home/koen/Documents/datasets/comb"), e, s);
        }
    }



    return EXIT_SUCCESS;
}
