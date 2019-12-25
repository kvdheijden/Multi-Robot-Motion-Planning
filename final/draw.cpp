#include <iostream>
#include <vector>
#include <regex>

#include <boost/filesystem.hpp>

#include <QString>
#include <QGuiApplication>
#include <QImage>
#include <QPainter>

#include <CGAL/Qt/PainterOstream.h>

#include "cgal_types.h"

constexpr int RADIUS = 5000;

void paint_workspace(QPainter &painter, const Workspace &W) {
    CGAL::Qt::PainterOstream<Kernel> painterOstream(&painter);
    painter.setPen(QPen(Qt::black, 5));
    for (auto iter = W.vertices_begin(); iter != W.vertices_end(); ++iter) {
        painterOstream << *iter;
    }
    for (auto iter = W.edges_begin(); iter != W.edges_end(); ++iter) {
        painterOstream << *iter;
    }
}

void paint_configurations(QPainter &painter, const std::vector<Point> &P, const QColor &color) {
    CGAL::Qt::PainterOstream<Kernel> painterOstream(&painter);
    painter.setPen(QPen(color, 1));
    painter.setBrush(QBrush(color));
    for (const Point &p : P) {
        painterOstream << Circle(p, Kernel::FT(1));
    }
}

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);
    const std::regex r("n([0-9]+)_m([0-9]+)");
    std::smatch match;

    const boost::filesystem::path path("/home/koen/Documents/datasets_r" + std::to_string(RADIUS));

    std::vector<boost::filesystem::path> v;
    for (const boost::filesystem::directory_entry &entry : boost::filesystem::directory_iterator(path))
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

        std::less<> less;
        return (na == nb) ? less(ma, mb) : less(na, nb);
    });


    for (auto &&x : v) {
        std::cout << "Starting drawing of " << x.filename() << std::endl;

        const boost::filesystem::path w = x / "w.txt";
        if (!boost::filesystem::is_regular_file(w)) {
            std::cout << "No workspace file found in " << x.filename() << std::endl;
            continue;
        }

        if (!std::regex_match(x.filename().string(), match, r)) {
            std::cout << "Failed to determine n/m in " << x.filename() << std::endl;
            continue;
        }
        int n = std::stoi(match[1].str());
        int m = std::stoi(match[2].str());

        Workspace W;
        boost::filesystem::ifstream(w) >> W;

        Point p;
        std::vector<Point> S, T;
        for (int i = 0; i < m; i++) {
            const boost::filesystem::path s = x / ("s_" + std::to_string(i) + ".txt");
            if (!boost::filesystem::is_regular_file(s)) {
                std::cout << "Failed to open " << s.filename() << " in " << x.filename() << std::endl;
                break;
            }
            const boost::filesystem::path t = x / ("t_" + std::to_string(i) + ".txt");
            if (!boost::filesystem::is_regular_file(t)) {
                std::cout << "Failed to open " << t.filename() << " in " << x.filename() << std::endl;
                break;
            }

            boost::filesystem::ifstream(s) >> p;
            S.push_back(p);

            boost::filesystem::ifstream(t) >> p;
            T.push_back(p);
        }

        if (W.size() != n) {
            std::cout << "Invalid workspace size for " << x.filename() << std::endl;
            continue;
        }
        if (S.size() != m) {
            std::cout << "Invalid starting configuration size for " << x.filename() << std::endl;
            continue;
        }
        if (T.size() != m) {
            std::cout << "Invalid target configuration size for " << x.filename() << std::endl;
            continue;
        }

        const boost::filesystem::path img = x / "img.png";

        QImage image(2 * RADIUS, 2 * RADIUS, QImage::Format_RGB32);
        image.fill(Qt::white);

        QPainter painter;
        painter.begin(&image);
        painter.translate(QPoint(RADIUS, RADIUS));
        paint_workspace(painter, W);
        paint_configurations(painter, S, Qt::green);
        paint_configurations(painter, T, Qt::magenta);
        painter.end();

        image.save(QString(img.c_str()), "PNG");

        std::cout << "Done drawing of " << x.filename() << std::endl << std::endl;
    }
}
