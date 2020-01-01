#ifndef MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H
#define MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H

#include "InterferenceForest.h"

enum solve_motion_graph_function {
    PEBBLE_GAME,
    PURPLE_TREE
};

bool pebble_game_process(MotionGraph &T_g,
                         MotionGraphVertexDescriptor &v,
                         std::vector<Move> &moves);

bool purple_tree_process(MotionGraph &T_g,
                         MotionGraphVertexDescriptor &v,
                         std::vector<Move> &moves);

void solve_motion_graph(MotionGraph &G_i, std::vector<Move> &motionSchedule, solve_motion_graph_function s);

#endif //MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H
