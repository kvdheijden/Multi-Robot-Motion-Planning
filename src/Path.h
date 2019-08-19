#ifndef MULTI_ROBOT_MOTION_PLANNING_PATH_H
#define MULTI_ROBOT_MOTION_PLANNING_PATH_H

struct Path {
    using Point = std::pair<double, double>;

    struct Element {
        const bool is_circular;
        const Point next;
        const Point helper;
    };

    std::list<Element> elements;
};

#endif //MULTI_ROBOT_MOTION_PLANNING_PATH_H
