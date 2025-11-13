//
// Created by refactoring utility.h 2024
// Mathematical utility functions
//
#pragma once

#include <vector>
#include <algorithm>

namespace Slam3refParam {
    // Global node index generation
    constexpr int kSessionStartIdxOffset = 1000000;

    int genGlobalNodeIdx(const int& _session_idx, const int& _node_offset);
    int ungenGlobalNodeIdx(const int& _session_idx, const int& _idx_in_graph);
    int genAnchorNodeIdx(const int& _session_idx);

    // Vector utilities
    std::vector<std::pair<double, int>> sortVecWithIdx(const std::vector<double>& arr);
    std::vector<int> linspace(int a, int b, int N);
    void collect_digits(std::vector<int>& digits, int num);

    // Distance calculation
    float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2);
}
