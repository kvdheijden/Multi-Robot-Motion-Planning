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
    scene.addItem(wg);

    sg = new ConfigurationGraphicsItem(startConfigs);
    QObject::connect(
            this, SIGNAL(changed()),
            sg, SLOT(modelChanged())
    );
    sg->setDiscPen(QPen(Qt::green, 1.));
    scene.addItem(sg);

    tg = new ConfigurationGraphicsItem(targetConfigs);
    QObject::connect(
            this, SIGNAL(changed()),
            tg, SLOT(modelChanged())
    );
    tg->setDiscPen(QPen(Qt::magenta, 1.));
    scene.addItem(tg);

    fsg = new FreeSpaceGraphicsItem(free_space);
    QObject::connect(
            this, SIGNAL(changed()),
            fsg, SLOT(modelChanged())
    );
    fsg->setCurvesPen(QPen(Qt::blue, 1.));
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

    QObject::connect(this->actionExit, SIGNAL(triggered()), this, SLOT(close()));

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
}

MainWindow::~MainWindow()
{
    delete wi;
    delete si;
    delete ti;

    delete wg;
    delete sg;
    delete tg;
    delete fsg;

    delete iGroup;
}

void MainWindow::processInputWorkspace(CGAL::Object object)
{
    std::list<Point> points;
    if (CGAL::assign(points, object)) {
        CGAL_assertion(points.front() == points.back());
        points.pop_front();
        Polygon p(points.begin(), points.end());
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
    Point point;
    if (CGAL::assign(point, object)) {
        startConfigs.push_back(point);
    }
    emit(changed());
}

void MainWindow::processInputTargetConfigs(CGAL::Object object)
{
    Point point;
    if (CGAL::assign(point, object)) {
        targetConfigs.push_back(point);
    }
    emit(changed());
}

void MainWindow::on_actionNew_triggered()
{
    // Clear data structures
    workspace.clear();
    startConfigs.clear();
    targetConfigs.clear();
    free_space.clear();

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

    std::vector<Inset_polygon> F;
    generate_free_space(workspace, F);
    remove_start_target_configs(F, startConfigs, targetConfigs, free_space);
    emit(changed());
}

void MainWindow::on_actionRecenter_triggered()
{
    QRectF bbox = wg->boundingRect();
    this->graphicsView->setSceneRect(bbox);
    this->graphicsView->fitInView(bbox, Qt::KeepAspectRatio);
}
