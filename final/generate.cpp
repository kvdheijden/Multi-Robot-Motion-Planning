#include <iostream>

#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/Aff_transformation_2.h>

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

bool valid_point_in_freespace(Point &p, const FreeSpace &F, const std::vector<Point> &S,
                              const std::vector<Point> &T) {

    int i = std::rand() % F.size();
    const Polygon &F_i = F[i];

    return valid_point_in_polygon(p, F_i, S, T);
}

int main(int argc, char *argv[]) {
    constexpr int R[] = {5000};
    constexpr int N[] = {300};
    constexpr int M[] = {5};

    for (int r : R) {
        for (int n : N) {
            for (int m : M) {
                std::string dirname = "/home/koen/Documents/datasets_r" + std::to_string(r) + "/n" + std::to_string(n) + "_m" + std::to_string(m);

                std::ofstream file(dirname + "/w.txt");
                if (!file) {
                    std::cout << "Failed to open workspace file." << std::endl;
                    exit(1);
                }

                std::cout << "N = " << n << " | M = " << m << std::endl;
                Point_generator pg(static_cast<double>(r));

                Workspace W;
                std::cout << "Generating workspace with " << n << " points." << std::endl;
                CGAL::random_polygon_2(n, std::back_inserter(W), pg);
                file << W;
                file.close();

                FreeSpace F;
                generate_free_space(W, F);
                int i = 0;
                std::for_each(F.begin(), F.end(), [&](const Polygon &F_i) {
                    std::ofstream file(dirname + "/F_" + std::to_string(i++) + ".txt");
                    if (!file) {
                        std::cout << "Failed to open free space file." << std::endl;
                        exit(1);
                    }
                    file << F_i;
                    file.close();
                });

                std::cout << "Generating " << m << " robot start configurations." << std::endl;
                std::vector<Point> S, T;
                while (S.size() < m) {
                    Point p;
                    if (!valid_point_in_freespace(p, F, S, T)) {
                        file << "Errored out" << std::endl;
                        std::cout << "Errored out" << std::endl;
                        goto next;
                    }
                    file = std::ofstream(dirname + "/s_" + std::to_string(S.size()) + ".txt");
                    if (!file) {
                        std::cout << "Failed to open starting configuration file." << std::endl;
                        exit(1);
                    }
                    file << p;
                    S.emplace_back(p);
                    file.close();
                }

                std::cout << "Generating " << m << " robot target configurations." << std::endl;
                for (const Polygon &F_i : F) {
                    for (const Point &s : S) {
                        if (check_inside(s, F_i)) {
                            Point p;
                            if (!valid_point_in_polygon(p, F_i, S, T)) {
                                file << "Errored out" << std::endl;
                                std::cout << "Errored out" << std::endl;
                                goto next;
                            }

                            file = std::ofstream(dirname + "/t_" + std::to_string(T.size()) + ".txt");
                            if (!file) {
                                std::cout << "Failed to open starting configuration file." << std::endl;
                                exit(1);
                            }
                            file << p;
                            T.emplace_back(p);
                            file.close();
                        }
                    }
                }

                next:;
                std::cout << std::endl;
            }
        }
    }
    return 0;
}
