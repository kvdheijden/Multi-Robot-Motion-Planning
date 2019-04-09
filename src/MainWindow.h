#ifndef MULTI_ROBOT_MOTION_PLANNING_MAINWINDOW_H
#define MULTI_ROBOT_MOTION_PLANNING_MAINWINDOW_H

#include "ui_MainWindow.h"
#include <QMainWindow>
#include <QGraphicsScene>

#include <CGAL/Object.h>

#include <CGAL/Qt/GraphicsViewPolylineInput.h>
#include <CGAL/Qt/GraphicsViewPointInput.h>

#include "cgal_types.h"
#include "WorkspaceGraphicsItem.h"
#include "ConfigurationGraphicsItem.h"
#include "FreeSpaceGraphicsItem.h"

#include "VertexProperty.h"

#include <boost/graph/undirected_graph.hpp>

class MainWindow : public QMainWindow, private Ui::MainWindow {
Q_OBJECT

private:
    QGraphicsScene scene;
    QActionGroup *iGroup;

    Input_polygon workspace;
    WorkspaceGraphicsItem *wg;
    CGAL::Qt::GraphicsViewPolylineInput<Kernel> *wi;

    std::vector<Point> startConfigs, targetConfigs;
    ConfigurationGraphicsItem *sg, *tg;
    CGAL::Qt::GraphicsViewPointInput<Kernel> *si, *ti;

    std::vector<std::pair<Polygon, General_polygon_set>> free_space;
    FreeSpaceGraphicsItem *fsg;

    std::vector<boost::undirected_graph<VertexProperty>> G;

    void clear_UI();
    void clear_objects();

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

public slots:
    void processInputWorkspace(CGAL::Object object);
    void processInputStartConfigs(CGAL::Object object);
    void processInputTargetConfigs(CGAL::Object object);

    void on_actionNew_triggered();
    void on_actionOpen_triggered();
    void on_actionImport_triggered();
    void on_actionExport_triggered();
    void on_actionExit_triggered();

    void on_actionUndo_triggered();
    void on_actionRedo_triggered();
    void on_actionInsertNone_toggled(bool checked);
    void on_actionInsertWorkspace_toggled(bool checked);
    void on_actionInsertStartConfigs_toggled(bool checked);
    void on_actionInsertTargetConfigs_toggled(bool checked);

    void on_actionGenerateFreeSpace_triggered();
    void on_actionGenerateMotionGraph_triggered();

    void on_actionRecenter_triggered();

signals:
    void changed();
};


#endif //MULTI_ROBOT_MOTION_PLANNING_MAINWINDOW_H
