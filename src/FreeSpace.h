#ifndef MULTI_ROBOT_MOTION_PLANNING_FREESPACE_H
#define MULTI_ROBOT_MOTION_PLANNING_FREESPACE_H

#include <vector>

#include "cgal_types.h"

class FreeSpace {
public:
    typedef std::vector<Polygon>::iterator                    iterator;
    typedef std::vector<Polygon>::const_iterator              const_iterator;

private:
    std::vector<Polygon> F;

public:
    FreeSpace();
    ~FreeSpace();

    iterator begin();
    const_iterator begin() const;

    iterator end();
    const_iterator end() const;

    void clear();

    std::vector<Polygon>& container();
};


#endif //MULTI_ROBOT_MOTION_PLANNING_FREESPACE_H
