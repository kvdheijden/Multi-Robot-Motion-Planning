//
// Created by koen on 12-4-19.
//

#ifndef MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONSET_H
#define MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONSET_H

#include "Configuration.h"

class ConfigurationSet {

private:
    std::vector<Configuration> configurations;
    int next_source_index;
    int next_target_index;

public:
    ConfigurationSet();
    ~ConfigurationSet();

    void addSourceConfiguration(Point &p);
    void addTargetConfiguration(Point &p);

    typedef std::vector<Configuration>::iterator        iterator;
    typedef std::vector<Configuration>::const_iterator  const_iterator;

    iterator begin();
    const_iterator begin() const;

    iterator end();
    const_iterator end() const;

    void clear();
    size_t size() const;
};


#endif //MULTI_ROBOT_MOTION_PLANNING_CONFIGURATIONSET_H
