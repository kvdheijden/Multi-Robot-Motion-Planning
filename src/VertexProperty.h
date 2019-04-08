//
// Created by koen on 8-4-19.
//

#ifndef MULTI_ROBOT_MOTION_PLANNING_VERTEXPROPERTY_H
#define MULTI_ROBOT_MOTION_PLANNING_VERTEXPROPERTY_H


class VertexProperty {
public:
    bool has_pebble;
    const bool is_start_config;
    const int id;

    VertexProperty(bool is_start_config, int id) : is_start_config(is_start_config), id(id), has_pebble(is_start_config)
    {

    }
};


#endif //MULTI_ROBOT_MOTION_PLANNING_VERTEXPROPERTY_H
