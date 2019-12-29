#include <iostream>
#include <boost/filesystem.hpp>

#include "free_space.h"

int main(int argc, char *argv[]) {
    (void) argc;
    (void) argv;

    const std::vector<int> M = {
        2, 5, 2, 4, 3
    };

    constexpr bool splits[] = {false, true};

    constexpr double separation = 5.0;
    constexpr double h = 1.5;

    for (bool split : splits) {
        const boost::filesystem::path base("/home/koen/Documents/datasets/corridor");
        const boost::filesystem::path base2 = base / ((split) ? "multi" : "single");

        std::vector<Point> points, S, T;

        double x = separation / 2;
        double y = h;
        for (int m : M) {
            for (int i = 0; i < m; i++) {
                S.emplace_back(x, y);
                x += separation;
            }
            for (int i = 0; i < m; i++) {
                T.emplace_back(x, y);
                x += separation;
            }
        }

        x -= separation / 2.;
        y += h;

        double peak = (split) ? 0.6 : 0.4;
        points.emplace_back(0, 0);
        double x1 = separation / 2.;
        for (std::size_t i = 1; i < M.size(); i++) {
            int m = M[i - 1];

            x1 += (2 * m * separation);
            double x2 = x1 - separation + std::sqrt(1 * 1 - (h - 0.6) * (h - 0.6));
            points.emplace_back(x2, 0);
            points.emplace_back(x2, peak);
            points.emplace_back(x2 + 1, 0);
        }
        points.emplace_back(x, 0);
        int size = static_cast<int>(points.size());
        for (int i = size - 1; i >= 0; i--) {
            points.emplace_back(points[i].x(), y - points[i].y());
        }

        const boost::filesystem::path path =
                base2 / ("n" + std::to_string(points.size()) + "_m" + std::to_string(S.size()));
        boost::filesystem::create_directories(path);
        boost::filesystem::ofstream(path / "w.txt") << Workspace(points.begin(), points.end());
        for (std::size_t i = 0; i < S.size(); i++) {
            boost::filesystem::ofstream(path / ("s_" + std::to_string(i) + ".txt")) << S[i];
            boost::filesystem::ofstream(path / ("t_" + std::to_string(i) + ".txt")) << T[i];
        }
    }
    return 0;
}
