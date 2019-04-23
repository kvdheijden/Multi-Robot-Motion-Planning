#ifndef MULTI_ROBOT_MOTION_PLANNING_CONFIGURATION_H
#define MULTI_ROBOT_MOTION_PLANNING_CONFIGURATION_H

#include "cgal_types.h"

class Configuration {

private:
    const Point p;
    const bool is_source{};
    const int index{};

public:
    Configuration(Point &&p, bool is_source, int index);
    Configuration(const Configuration& other);

    ~Configuration();

    bool operator==(const Configuration& other) const;
    bool operator!=(const Configuration& other) const;

    const Point &getPoint() const;

    const bool &isStart() const;

    const int &getIndex() const;

    CGAL::Bbox_2 bbox() const;
};

#endif //MULTI_ROBOT_MOTION_PLANNING_CONFIGURATION_H
