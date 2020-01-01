#include <iostream>
#include <boost/filesystem.hpp>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>

#include "free_space.h"
#include "motion_graph.h"

typedef CGAL::Creator_uniform_2<double, Point> Creator;
typedef CGAL::Random_points_in_square_2<Point, Creator> Point_generator;

bool valid_point_in_polygon(Point &p, const Polygon &F_i, const std::vector<Point> &S,
                            const std::vector<Point> &T) {
    const CGAL::Bbox_2 bbox = F_i.bbox();
    double xmin = bbox.xmin();
    double ymin = bbox.ymin();
    double xmax = bbox.xmax();
    double ymax = bbox.ymax();
    double width = xmax - xmin;
    double height = ymax - ymin;

    CGAL::Aff_transformation_2<Kernel> translate(CGAL::TRANSLATION, Vector((xmin + xmax) / 2., (ymin + ymax) / 2.));
    Point_generator pg(std::max(width, height) / 2.);

    int failsafe = 10000;

    bool correct = false;
    do {
        std::copy_n(pg++, 1, &p);

        if (check_inside(p, F_i)) {
            correct = true;

            for (const Point &s : S) {
                if (CGAL::squared_distance(p, s) < Kernel::FT(16)) {
                    correct = false;
                    break;
                }
            }
            for (const Point &t : T) {
                if (CGAL::squared_distance(p, t) < Kernel::FT(16)) {
                    correct = false;
                    break;
                }
            }
        }
    } while (!correct && (--failsafe));

    return failsafe;
}

int main(int argc, char *argv[]) {
    (void) argc;
    (void) argv;

    constexpr int R = 5000;
    constexpr int N[] = {1000}; // # of workspace points
    constexpr int M[] = {1, 2, 3, 5, 8, 10, 20, 30, 50, 80, 100}; // # of robots

    const boost::filesystem::path base("/home/koen/Documents/datasets/random");

    for (int n : N) {
        Point_generator pg(static_cast<double>(R));

        Workspace W;
        FreeSpace F;

        std::cout << "Generating workspace with " << n << " points." << std::endl;
//        CGAL::random_polygon_2(n, std::back_inserter(W), pg);
        boost::filesystem::ifstream("/home/koen/Documents/datasets_r5000/n1000_m50/w.txt") >> W;
        generate_free_space(W, F);

        for (int m : M) {
            const boost::filesystem::path path = base / ("n" + std::to_string(n) + "_m" + std::to_string(m));
            boost::filesystem::create_directories(path);

            boost::filesystem::ofstream(path / "w.txt") << W;
            boost::filesystem::ofstream(path / "F_0.txt") << F[0];

            std::vector<Point> S, T;
            for (int i = 0; i < m; i++) {
                Point p;

                while (!valid_point_in_polygon(p, F[0], S, T));
                boost::filesystem::ofstream(path / ("s_" + std::to_string(i) + ".txt")) << p;
                S.emplace_back(p);

                while (!valid_point_in_polygon(p, F[0], S, T));
                boost::filesystem::ofstream(path / ("t_" + std::to_string(i) + ".txt")) << p;
                T.emplace_back(p);
            }
        }
    }
    return 0;
}