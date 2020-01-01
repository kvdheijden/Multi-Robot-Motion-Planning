#include <iostream>
#include <boost/filesystem.hpp>

#include "free_space.h"

int main(int argc, char *argv[]) {
    (void) argc;
    (void) argv;

    constexpr int M[] = {1, 2, 3, 5, 8, 10, 20, 30, 50, 80, 100}; // # of comb teeth, n = 4 * m
    const boost::filesystem::path base("/home/koen/Documents/datasets/comb");

    double e = 0.1;
    double l = 20.0;
    double h = 6.0;

    for (int m : M) {
        int n = (2 * m) + (2 * (m - 1)) + 2;
        const boost::filesystem::path path = base / ("n" + std::to_string(n) + "_m" + std::to_string(m));
        boost::filesystem::create_directories(path);

        double x = 0, y = 0;

        std::vector<Point> points;
        points.emplace_back(x, y);
        y += l;
        for (int i = 0; i < m; i++) {
            points.emplace_back(x, y);

            x += h / 2;
            y -= h / 2;
            Point s(x, y);
            Point t(x, h / 2);
            boost::filesystem::ofstream(path / ("s_" + std::to_string(i) + ".txt")) << s;
            boost::filesystem::ofstream(path / ("t_" + std::to_string(i) + ".txt")) << t;

            x += h / 2;
            y += h / 2;
            points.emplace_back(x, y);

            if (i < m - 1) {
                y -= (l - h);
                points.emplace_back(x, y);
                x += e;
                points.emplace_back(x, y);
                y += (l - h);
            } else {
                y = 0;
                points.emplace_back(x, y);
            }
        }

        Workspace W(points.begin(), points.end());
        boost::filesystem::ofstream(path / "w.txt") << W;

//        FreeSpace F;
//        generate_free_space(W, F);
//        if (F.size() != 1) {
//            std::cout << "Free space mismatch!" << std::endl;
//            return 1;
//        }
//        boost::filesystem::ofstream(path / "F_0.txt") << F[0];
    }

    return 0;
}
