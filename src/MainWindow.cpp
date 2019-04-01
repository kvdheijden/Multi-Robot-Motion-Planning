#include "MainWindow.h"

#include <QMessageBox>
#include <CGAL/assertions.h>

#include "mrmp.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), Ui::MainWindow()
{
    setupUi(this);

    // Setup GraphicsItems
    wg = new WorkspaceGraphicsItem(workspace);
    QObject::connect(
            this, SIGNAL(changed()),
            wg, SLOT(modelChanged())
    );
    wg->setEdgesPen(QPen(Qt::black, .1));
    wg->setVerticesPen(QPen(Qt::black, .1));
    scene.addItem(wg);

    sg = new ConfigurationGraphicsItem(startConfigs);
    QObject::connect(
            this, SIGNAL(changed()),
            sg, SLOT(modelChanged())
    );
    sg->setDiscPen(QPen(Qt::green, .1));
    sg->setDiscBrush(QBrush(Qt::green));
    scene.addItem(sg);

    tg = new ConfigurationGraphicsItem(targetConfigs);
    QObject::connect(
            this, SIGNAL(changed()),
            tg, SLOT(modelChanged())
    );
    tg->setDiscPen(QPen(Qt::magenta, .1));
    tg->setDiscBrush(QBrush(Qt::magenta));
    scene.addItem(tg);

    fsg = new FreeSpaceGraphicsItem(free_space);
    QObject::connect(
            this, SIGNAL(changed()),
            fsg, SLOT(modelChanged())
    );
    fsg->setCurvesPen(QPen(Qt::blue, .1));
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

    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    scene.setSceneRect(-10, -10, 10, 10);

    this->graphicsView->setScene(&scene);
    this->graphicsView->setMouseTracking(true);
    this->graphicsView->matrix().scale(1, -1);

    QRectF bbox(-25, -25, 50, 50);
    this->graphicsView->setSceneRect(bbox);
    this->graphicsView->fitInView(bbox, Qt::KeepAspectRatio);
}

MainWindow::~MainWindow()
{
    delete wi;
    delete si;
    delete ti;

    delete fsg;
    delete wg;
    delete sg;
    delete tg;

    delete iGroup;
}

void MainWindow::processInputWorkspace(CGAL::Object object)
{
    std::list<Input_point> points;
    if (CGAL::assign(points, object)) {
        if (points.size() <= 2) {
            return;
        }

        clear_objects();

        CGAL_assertion(points.front() == points.back());
        points.pop_front();
        Input_polygon p(points.begin(), points.end());
        if (!p.is_simple()) {
            // Error
            QMessageBox messageBox;
            messageBox.critical(nullptr, "Invalid Polygon", "The workspace polygon must be simple");
            messageBox.setFixedSize(500, 200);
            return;
        }

        // Copy to the workspace variable
        workspace = p;
    }
    emit(changed());
}

void MainWindow::processInputStartConfigs(CGAL::Object object)
{
    Input_point point;
    if (CGAL::assign(point, object)) {
        if (!check_inside(point, workspace)) {
            // Error
            QMessageBox messageBox;
            messageBox.critical(nullptr, "Invalid Point", "Start/Target configurations must be inside the workspace");
            messageBox.setFixedSize(500, 200);
            return;
        }
        startConfigs.push_back(point);
    }
    emit(changed());
}

void MainWindow::processInputTargetConfigs(CGAL::Object object)
{
    Input_point point;
    if (CGAL::assign(point, object)) {
        if (!check_inside(point, workspace)) {
            // Error
            QMessageBox messageBox;
            messageBox.critical(nullptr, "Invalid Point", "Start/Target configurations must be inside the workspace");
            messageBox.setFixedSize(500, 200);
            return;
        }
        targetConfigs.push_back(point);
    }
    emit(changed());
}

void MainWindow::on_actionNew_triggered()
{
    clear_objects();
    emit(changed());
}

void MainWindow::on_actionOpen_triggered()
{

}

void MainWindow::on_actionImport_triggered()
{

}

void MainWindow::on_actionExport_triggered()
{

}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

void MainWindow::on_actionUndo_triggered()
{

}

void MainWindow::on_actionRedo_triggered()
{

}

void MainWindow::on_actionInsertNone_toggled(bool checked)
{

}

void MainWindow::on_actionInsertWorkspace_toggled(bool checked)
{
    if (checked) {
        scene.installEventFilter(wi);
    } else {
        scene.removeEventFilter(wi);
    }
}

void MainWindow::on_actionInsertStartConfigs_toggled(bool checked)
{
    if (checked) {
        scene.installEventFilter(si);
    } else {
        scene.removeEventFilter(si);
    }
}

void MainWindow::on_actionInsertTargetConfigs_toggled(bool checked)
{
    if (checked) {
        scene.installEventFilter(ti);
    } else {
        scene.removeEventFilter(ti);
    }
}

void MainWindow::on_actionGenerateFreeSpace_triggered()
{
    free_space.clear();
    std::vector<Polygon> F;
    generate_free_space(workspace, F);
    for (const Polygon& f : F) {
        General_polygon_set gps = remove_start_target_configs(f, startConfigs, targetConfigs);
        free_space.push_back(std::make_pair(f, gps));
    }
    emit(changed());
}

void MainWindow::on_actionGenerateMotionGraph_triggered()
{

}

void MainWindow::on_actionRecenter_triggered()
{
    QRectF bbox = wg->boundingRect();
    this->graphicsView->setSceneRect(bbox);
    this->graphicsView->fitInView(bbox, Qt::KeepAspectRatio);
}

void MainWindow::clear_UI()
{
    scene.clear();
}

void MainWindow::clear_objects()
{
    // Clear data structures
    free_space.clear();
    workspace.clear();
    startConfigs.clear();
    targetConfigs.clear();
}