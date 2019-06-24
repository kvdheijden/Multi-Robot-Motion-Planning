#include "MainWindow.h"

#include <QMessageBox>
#include <CGAL/assertions.h>

#include <boost/graph/graph_utility.hpp>

#include "mrmp.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), Ui::MainWindow() {
    setupUi(this);

    // Setup GraphicsItems
    wg = new WorkspaceGraphicsItem(workspace);
    QObject::connect(
            this, SIGNAL(changed()),
            wg, SLOT(modelChanged())
    );
    scene.addItem(wg);

    cg = new ConfigurationSetGraphicsItem(configurations);
    QObject::connect(
            this, SIGNAL(changed()),
            cg, SLOT(modelChanged())
    );
    scene.addItem(cg);

    fsg = new FreeSpaceGraphicsItem(free_space);
    QObject::connect(
            this, SIGNAL(changed()),
            fsg, SLOT(modelChanged())
    );
    scene.addItem(fsg);

    // Setup GraphicsViewInputs
    wi = new CGAL::Qt::GraphicsViewPolylineInput<Kernel>(this, &scene);
    QObject::connect(
            wi, SIGNAL(generate(CGAL::Object)),
            this, SLOT(processInputWorkspace(CGAL::Object))
    );

    si = new CGAL::Qt::GraphicsViewPointInput<Kernel>(this);
    QObject::connect(
            si, SIGNAL(generate(CGAL::Object)),
            this, SLOT(processInputStartConfigs(CGAL::Object))
    );

    ti = new CGAL::Qt::GraphicsViewPointInput<Kernel>(this);
    QObject::connect(
            ti, SIGNAL(generate(CGAL::Object)),
            this, SLOT(processInputTargetConfigs(CGAL::Object))
    );

    iGroup = new QActionGroup(this);
    iGroup->addAction(this->actionInsertNone);
    iGroup->addAction(this->actionInsertWorkspace);
    iGroup->addAction(this->actionInsertStartConfigs);
    iGroup->addAction(this->actionInsertTargetConfigs);

    this->actionInsertNone->setChecked(true);

//    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
//    scene.setSceneRect(0, 0, 50, 50);

    this->graphicsView->setScene(&scene);
    this->graphicsView->setMouseTracking(true);
    this->graphicsView->matrix().scale(1, -1);

    QRectF bbox(0, 0, 50, 50);
    this->graphicsView->setSceneRect(bbox);
    this->graphicsView->fitInView(bbox, Qt::KeepAspectRatio);
}

MainWindow::~MainWindow() {
    delete wi;
    delete si;
    delete ti;

    delete fsg;
    delete cg;
    delete wg;

    delete iGroup;
}

void MainWindow::processInputWorkspace(CGAL::Object object) {
    std::list<Point> polyline;
    if (CGAL::assign(polyline, object)) {
        if (polyline.size() <= 2) {
            return;
        }

        // Clear all objects and the scene
        clear_objects();

        // Make sure this is a valid polyline
        CGAL_assertion(polyline.front() == polyline.back());
        polyline.pop_front();

        // Create the workspace
        Workspace W(polyline.begin(), polyline.end());

        // Make sure the workspace is simple
        if (!W.is_simple()) {
            // Error
            QMessageBox::critical(nullptr, "Invalid Polygon", "The workspace polygon must be simple");
            return;
        }

        // Copy to the workspace variable
        this->workspace = W;

        // Generate the free space
        generate_free_space(workspace, free_space);
    }
    emit(changed());
}

void MainWindow::processInputStartConfigs(CGAL::Object object) {
    Point point;
    if (CGAL::assign(point, object)) {

        // Determine if point is inside free space
        bool inside = false;
        for (const Polygon &F : free_space) {
            if (check_inside(point, F)) {
                inside = true;
                break;
            }
        }

        if (!inside) {
            // Error
            QMessageBox::critical(nullptr, "Invalid Point", "Start/Target configurations must be inside the workspace");
            return;
        }

        configurations.addSourceConfiguration(point);
    }
    emit(changed());
}

void MainWindow::processInputTargetConfigs(CGAL::Object object) {
    Point point;
    if (CGAL::assign(point, object)) {

        // Determine if point is inside free space
        bool inside = false;
        for (const Polygon &F : free_space) {
            if (check_inside(point, F)) {
                inside = true;
                break;
            }
        }

        if (!inside) {
            // Error
            QMessageBox::critical(nullptr, "Invalid Point", "Start/Target configurations must be inside the workspace");
            return;
        }

        configurations.addTargetConfiguration(point);
    }
    emit(changed());
}

void MainWindow::on_actionNew_triggered() {
    clear_objects();
    emit(changed());
}

void MainWindow::on_actionOpen_triggered() {

}

void MainWindow::on_actionImport_triggered() {

}

void MainWindow::on_actionExport_triggered() {

}

void MainWindow::on_actionExit_triggered() {
    this->close();
}

void MainWindow::on_actionUndo_triggered() {

}

void MainWindow::on_actionRedo_triggered() {

}

void MainWindow::on_actionInsertNone_toggled(bool checked) {

}

void MainWindow::on_actionInsertWorkspace_toggled(bool checked) {
    if (checked) {
        scene.installEventFilter(wi);
    } else {
        scene.removeEventFilter(wi);
    }
}

void MainWindow::on_actionInsertStartConfigs_toggled(bool checked) {
    if (checked) {
        scene.installEventFilter(si);
    } else {
        scene.removeEventFilter(si);
    }
}

void MainWindow::on_actionInsertTargetConfigs_toggled(bool checked) {
    if (checked) {
        scene.installEventFilter(ti);
    } else {
        scene.removeEventFilter(ti);
    }
}

void MainWindow::on_actionGenerateMotionGraph_triggered() {
    G.clear();
    int index = 0;
    for (const Polygon &F_i : free_space) {
        // Create a Interference forest vertex
        InterferenceForestVertexDescriptor i = G.add_vertex();
        InterferenceForestVertex &v = G[i];
        v.freeSpaceComponent = &F_i;
        v.visited = false;
        v.index = index++;

        // Retrieve the motion graph from it
        MotionGraph &G_i = v.motionGraph;

        // Generate the motion graph
        generate_motion_graph(F_i, configurations, G_i);

        // Print
        std::cerr << std::string("G_") + std::to_string(v.index) << std::endl;
        boost::print_graph(G_i, [&](MotionGraphVertexDescriptor vd) {
            const Configuration *c = G_i[vd].configuration;
            return (c->isStart() ? "s" : "t") + std::to_string(c->getIndex());
        }, std::cerr);
        std::cerr << std::endl;
    }

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

            for (const Configuration &c : configurations) {
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

    // Print
    std::cerr << "Interference Forest: " << std::endl;
    boost::print_graph(G, [&](InterferenceForestVertexDescriptor vd) {
        return "G_" + std::to_string(G[vd].index);
    }, std::cerr);
    std::cerr << std::endl;
}

void MainWindow::on_actionSolve_triggered() {
    /// Topological sorting (Kahn's algorithm)
    // L <- Empty list that will contain the sorted elements
    std::list<InterferenceForestVertexDescriptor> L;
    // S <- Set of all nodes with no incoming edge
    std::set<InterferenceForestVertexDescriptor> S;

    typename boost::graph_traits<InterferenceForest>::vertex_iterator vi, v_end;
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
        InterferenceForestVertex &v = G[n];
        MotionGraph &G_i = v.motionGraph;
        solve_motion_graph(G_i, moves);
    }

    simplify_moves(moves);

    for (const Move &m : moves) {
        std::cerr << "Move robot at "
                  << (m.first.isStart() ? "s" : "t") << m.first.getIndex()
                  << " towards "
                  << (m.second.isStart() ? "s" : "t") << m.second.getIndex()
                  << std::endl;
    }

    std::vector<std::reference_wrapper<const Configuration>> robots;
    for (const Configuration &configuration : configurations) {
        if (configuration.isStart()) {
            robots.emplace_back(configuration);
        }
    }

    for (const Move &move : moves) {
        for (const Polygon &f : free_space) {
            if (check_inside(move.first.getPoint(), f)) {
                get_shortest_path(move, f, robots);
            }
        }
    }
}

void MainWindow::on_actionRecenter_triggered() {
    QRectF bbox = wg->boundingRect();
    this->graphicsView->setSceneRect(bbox);
    this->graphicsView->fitInView(bbox, Qt::KeepAspectRatio);
}

void MainWindow::clear_objects() {
    // Clear data structures
    G.clear();
    free_space.clear();
    workspace.clear();
    configurations.clear();
}