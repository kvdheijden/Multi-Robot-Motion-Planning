#include <iostream>
#include <boost/filesystem.hpp>
#include <QApplication>
#include <QGraphicsScene>
#include <QObject>

#include <CGAL/Qt/CircularArcGraphicsItem.h>

#include "MainWindow.h"

static void snap_xdg_fix()
{
    using namespace boost;
    filesystem::path xdg_runtime_dir(std::getenv("XDG_RUNTIME_DIR"));
    if (!filesystem::exists(xdg_runtime_dir)) {
        std::cerr << "XDG_RUNTIME_DIR (" << xdg_runtime_dir << ") did not exist. Creating it." << std::endl;
        filesystem::create_directories(xdg_runtime_dir);
    }
}

int main(int argc, char *argv[])
{
    snap_xdg_fix();

    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.show();
    return QApplication::exec();
}