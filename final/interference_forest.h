#ifndef MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H
#define MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H

#include "InterferenceForest.h"

#define MOTION_GRAPH_SOLVE_PURPLE_TREE

constexpr enum solve_motion_graph_function {
    PEBBLE_GAME,
    PURPLE_TREE
} solveMotionGraphFcn =
#if defined(MOTION_GRAPH_SOLVE_PURPLE_TREE)
        PURPLE_TREE
#else
        PEBBLE_GAME
#endif
;

bool pebble_game_process(MotionGraph &T_g,
                         MotionGraphVertexDescriptor &v,
                         std::vector<Move> &moves);

bool purple_tree_process(MotionGraph &T_g,
                         MotionGraphVertexDescriptor &v,
                         std::vector<Move> &moves);

void solve_motion_graph(MotionGraph &G_i, std::vector<Move> &motionSchedule);

#endif //MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H
