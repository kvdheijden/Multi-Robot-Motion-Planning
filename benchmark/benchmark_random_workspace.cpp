//#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/approximated_offset_2.h>

#include <boost/timer.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel         Kernel;
typedef CGAL::Point_2<Kernel>                                       Point;
typedef CGAL::Polygon_2<Kernel>                                     Polygon;
typedef CGAL::Creator_uniform_2<int, Point>                         Creator;
typedef CGAL::Random_points_in_disc_2<Point, Creator>               Point_generator;
typedef CGAL::Gps_circle_segment_traits_2<Kernel>                   Gps_traits;

int main(int argc, char *argv[]) {
    constexpr double RADIUS = 100;
    constexpr int M = 500;
    const Point_generator pg(RADIUS);
    for (int m = 3; m < M; m++) {
        Polygon W;
        CGAL::random_polygon_2(m, std::back_inserter(W), pg);

        double eps = 0.1; //std::numeric_limits<double>::epsilon();
        Kernel::FT r(1 - eps);
        std::list<Gps_traits::Polygon_2> F;

        boost::timer timer;
        CGAL::approximated_inset_2(W, r, eps, std::back_inserter(F));
        double seconds = timer.elapsed();
        std::cout << "M = " << m << std::endl;
        std::cout << "Free space generation: " << seconds << "s" << std::endl;
        std::cout << "|F|: " << F.size() << std::endl;
        std::cout << std::endl;
    }
}