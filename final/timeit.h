#ifndef MULTI_ROBOT_MOTION_PLANNING_TIMEIT_H
#define MULTI_ROBOT_MOTION_PLANNING_TIMEIT_H

#include <chrono>

class timeit {
private:
    using clock = std::chrono::high_resolution_clock;
    clock::time_point m_start;
public:
    timeit() : m_start(clock::now()) {

    }

    std::chrono::duration<double> elapsed() const {
        return clock::now() - m_start;
    }
};


#endif //MULTI_ROBOT_MOTION_PLANNING_TIMEIT_H
