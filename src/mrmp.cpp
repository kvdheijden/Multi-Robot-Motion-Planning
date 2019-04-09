#include "mrmp.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/approximated_offset_2.h>

template<int n>
static Polygon D(const Point &p) {
    Circle circle(p, Kernel::FT(n * n));

    Traits traits;
    Traits::Make_x_monotone_2 make_x_monotone = traits.make_x_monotone_2_object();

    Traits::Curve_2 curve(circle);
    std::vector<CGAL::Object> objects;
    make_x_monotone(curve, std::back_inserter(objects));
    CGAL_assertion(objects.size() == 2);

    Traits::X_monotone_curve_2 arc;
    Polygon result;
    for (const CGAL::Object &obj : objects) {
        CGAL::assign(arc, obj);
        result.push_back(arc);
    }

    return result;
}

bool check_inside(const Point &point, const Input_polygon &polygon) {
    CGAL::Bounded_side bounded_side = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), point,
                                                           Kernel());
    return (bounded_side != CGAL::ON_UNBOUNDED_SIDE);
}

bool check_inside(const Point &point, const Polygon &polygon) {
    const General_polygon_set gps(polygon);
    const Arrangement &arr = gps.arrangement();

    typedef CGAL::Arr_naive_point_location<Arrangement> Point_location;
    typedef Point_location::Point_2 Point_to_locate;
    typedef Point_location::Result Point_location_result;
    typedef Point_location_result::Type Point_location_result_type;

    Point_location pl(arr);
    const Point_to_locate ptl(point.x(), point.y());

    Point_location_result_type location = pl.locate(ptl);

    typedef Point_location_result::Vertex_const_handle Vertex_const_handle;
    typedef Point_location_result::Halfedge_const_handle Halfedge_const_handle;
    typedef Point_location_result::Face_const_handle Face_const_handle;

    const Vertex_const_handle *v;
    const Halfedge_const_handle *e;
    const Face_const_handle *f;

    if ((f = boost::get < Face_const_handle > (&location))) {
        return (*f)->has_outer_ccb();
    }
    return static_cast<bool>(boost::get < Halfedge_const_handle > (&location) ||
                             boost::get < Vertex_const_handle > (&location));
}


void generate_free_space(const Input_polygon &W, std::vector<Polygon> &F) {
    Kernel::FT r(1);
    double eps = 0.001; //std::numeric_limits<double>::epsilon();

    F.clear();
    CGAL::approximated_inset_2(W, r, eps, std::back_inserter(F));

    for (Polygon &f : F) {
        if (f.orientation() != CGAL::Orientation::COUNTERCLOCKWISE) {
            f.reverse_orientation();
        }
        CGAL_assertion(f.orientation() == CGAL::Orientation::COUNTERCLOCKWISE);
    }
}


General_polygon_set remove_start_target_configs(const Polygon &F,
                                                const std::vector<Point> &S,
                                                const std::vector<Point> &T) {
    General_polygon_set gps(F);

    int s_i = 0;
    for (const Point &s : S) {
        if (check_inside(s, F)) {
            gps.difference(D<2>(s));
            s_i++;
        }
    }

    int t_i = 0;
    for (const Point &t : T) {
        if (check_inside(t, F)) {
            gps.difference(D<2>(t));
            t_i++;
        }
    }

    // Ensure every free space part has equal no. of start and target configurations
    CGAL_assertion(s_i == t_i);

    return gps;
}

static bool do_intersect(const Polygon &pgn1, const Polygon &pgn2) {
    for (auto iter1 = pgn1.curves_begin(); iter1 != pgn1.curves_end(); ++iter1) {
        for (auto iter2 = pgn2.curves_begin(); iter2 != pgn2.curves_end(); ++iter2) {
            std::list<CGAL::Object> objects;
            auto result = iter1->intersect(*iter2, std::back_inserter(objects));

            if (!objects.empty()) {
                return true;
            }
        }
    }
    return false;
}

static bool do_intersect(const Polygon &pgn1, const Polygon::X_monotone_curve_2 &curve) {
    for (auto iter1 = pgn1.curves_begin(); iter1 != pgn1.curves_end(); ++iter1) {
        std::list<CGAL::Object> objects;
        auto result = iter1->intersect(curve, std::back_inserter(objects));

        if (!objects.empty()) {
            return true;
        }
    }
    return false;
}

class compare_along_segment {
public:
    explicit compare_along_segment(const Polygon::X_monotone_curve_2 &curve) : curve(curve), source(curve.source()) {}

    bool operator()(const Point &p, const Point &q) {
        Polygon cdP = D<2>(p);
        Polygon cdQ = D<2>(q);

        std::vector<Polygon::Point_2> intersectP;
        for (auto iter = cdP.curves_begin(); iter != cdP.curves_end(); ++iter) {
            std::vector<CGAL::Object> objects;
            iter->intersect(curve, std::back_inserter(objects));
            for (const CGAL::Object& object : objects) {
                std::pair<Polygon::Point_2, unsigned int> p_tmp;
//                Polygon::X_monotone_curve_2 c_tmp;

                if (CGAL::assign(p_tmp, object)) {
                    intersectP.push_back(p_tmp.first);
//                } else if (CGAL::assign(c_tmp, object)) {
//                    intersectP.push_back(c_tmp.source());
                } else {
                    std::cerr << "Invalid object" << std::endl;
                }
            }
        }
        CGAL_assertion(!intersectP.empty());

        std::vector<Polygon::Point_2> intersectQ;
        for (auto iter = cdQ.curves_begin(); iter != cdQ.curves_end(); ++iter) {
            std::vector<CGAL::Object> objects;
            iter->intersect(curve, std::back_inserter(objects));
            for (const CGAL::Object& object : objects) {
                std::pair<Polygon::Point_2, unsigned int> p_tmp;
//                Polygon::X_monotone_curve_2 c_tmp;

                if (CGAL::assign(p_tmp, object)) {
                    intersectQ.push_back(p_tmp.first);
//                } else if (CGAL::assign(c_tmp, object)) {
//                    intersectQ.push_back(c_tmp.source());
                } else {
                    std::cerr << "Invalid object" << std::endl;
                }
            }
        }
        CGAL_assertion(!intersectQ.empty());

        const Polygon::Point_2& pp = intersectP[0];
        const Polygon::Point_2& qq = intersectQ[0];

        if (curve.is_circular()) {
            return orientation(source, pp, qq) == CGAL::Orientation::RIGHT_TURN;
        }
        return squared_distance(source, pp) < squared_distance(source, qq);
    }

private:
    const Polygon::X_monotone_curve_2& curve;
    const Polygon::Point_2& source;

    static inline Point get_point(const Polygon::Point_2& p)
    {
        CORE::Expr x = p.x().a0() + p.x().a1() * CGAL::sqrt(p.x().root());
        CORE::Expr y = p.y().a0() + p.y().a1() * CGAL::sqrt(p.y().root());
        return Point(x, y);
    }

    static inline CGAL::Orientation orientation(const Polygon::Point_2& p, const Polygon::Point_2& q, const Polygon::Point_2& r)
    {
        Point pp = get_point(p);
        Point qq = get_point(q);
        Point rr = get_point(r);
        return CGAL::orientation(pp, qq, rr);
    }

    static inline Kernel::FT squared_distance(const Polygon::Point_2& p, const Polygon::Point_2& q)
    {
        Point pp = get_point(p);
        Point qq = get_point(q);
        return CGAL::squared_distance(pp, qq);
    }
};

void generate_motion_graph(const Polygon &F_i,
                           const std::vector<Polygon_with_holes> &F_star,
                           const std::vector<Point> &S,
                           const std::vector<Point> &T,
                           boost::undirected_graph<VertexProperty> &G_i) {
    typedef boost::undirected_graph<VertexProperty>::vertex_descriptor Vdesc;

    CGAL_precondition(S.size() == T.size());

    std::vector<Vdesc> source_descriptors;
    std::vector<Vdesc> target_descriptors;

    G_i.clear();
    for (int i = 0; i < S.size(); i++) {
        source_descriptors.push_back(G_i.add_vertex(VertexProperty(true, i)));
        target_descriptors.push_back(G_i.add_vertex(VertexProperty(false, i)));
    }

//    const Polygon::Point_2 start = F_i.curves_begin()->source();
//    Polygon::Point_2 curr = start;
//    Vdesc *u = nullptr;
//    Vdesc *first = nullptr;
//
//    do {
//        // Retrieve the next curve
//        Polygon::Curve_const_iterator iter;
//        for (iter = F_i.curves_begin(); iter != F_i.curves_end(); ++iter) {
//            if (curr == iter->source()) {
//                break;
//            }
//        }
//        CGAL_assertion(iter != F_i.curves_end());
//
//        // Find all points in the boundary of F_i
//        std::vector<Point> B_i;
//        for (int i = 0; i < S.size(); i++) {
//            if (check_inside(S[i], F_i) && do_intersect(D<2>(S[i]), *iter)) {
//                B_i.emplace_back(S[i]);
//            }
//            if (check_inside(T[i], F_i) && do_intersect(D<2>(T[i]), *iter)) {
//                B_i.emplace_back(T[i]);
//            }
//        }
//
//        // Order the points by intersection along the curve
//        std::sort(B_i.begin(), B_i.end(), compare_along_segment(*iter));
//
//        // Add edges between vertices in G_i
//        for (const Point &p : B_i) {
//            Vdesc *v = nullptr;
//            for (int i = 0; i < S.size(); i++) {
//                if (p == S[i]) {
//                    v = &source_descriptors[i];
//                } else if (p == T[i]) {
//                    v = &target_descriptors[i];
//                }
//            }
//            CGAL_assertion(v != nullptr);
//
//            if (first == nullptr) {
//                first = v;
//            }
//
//            if (v != nullptr && u != nullptr && u != v) {
//                G_i.add_edge(*u, *v);
//            }
//
//            u = v;
//        }
//
//        curr = iter->target();
//    } while (curr != start);
//
//    // Add the edge completing the boundary
//    CGAL_assertion((first == nullptr) == (u == nullptr));
//    if (first != nullptr && u != nullptr)  {
//        G_i.add_edge(*first, *u);
//    }

    // Add edges between vertices in H_i
    for (const Polygon_with_holes& F_star_i : F_star) {
        std::vector<Vdesc> B_i, H_i;

        for (int i = 0; i < S.size(); i++) {
            if (check_inside(S[i], F_i)) {
                const Polygon& boundary = F_star_i.outer_boundary();
                if (check_inside(S[i], boundary)) {
                    H_i.push_back(source_descriptors[i]);
                } else if (do_intersect(D<2>(S[i]), boundary)) {
                    B_i.push_back(source_descriptors[i]);
                }
                if (check_inside(T[i], boundary)) {
                    H_i.push_back(target_descriptors[i]);
                } else if (do_intersect(D<2>(T[i]), boundary)) {
                    B_i.push_back(target_descriptors[i]);
                }
            }
        }

        for (const Vdesc& b : B_i) {
            for (const Vdesc& h : H_i) {
                G_i.add_edge(b, h);
            }
        }

        for (int i = 0; i < B_i.size(); i++) {
            for (int j = 0; j < i; j++) {
                G_i.add_edge(B_i[i], B_i[j]);
            }
        }
        for (int i = 0; i < H_i.size(); i++) {
            for (int j = 0; j < i; j++) {
                G_i.add_edge(H_i[i], H_i[j]);
            }
        }
    }
}