#ifndef MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H
#define MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H

#include "InterferenceForest.h"

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

void solve_motion_graph_pebble_game(MotionGraph &G_i,
                                    const Polygon &F_i,
                                    std::vector<Move> &motionSchedule);

bool purple_tree_process(MotionGraph &T_g,
                         MotionGraphVertexDescriptor &v,
                         std::vector<Move> &moves);

void solve_motion_graph_purple_tree(MotionGraph &G_i,
                                    const Polygon &F_i,
                                    std::vector<Move> &motionSchedule);

void solve_interference_forest(InterferenceForest &G,
                               std::vector<Move> motionSchedule);

#endif //MULTI_ROBOT_MOTION_PLANNING_INTERFERENCE_FOREST_H
