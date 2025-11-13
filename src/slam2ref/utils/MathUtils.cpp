//
// Created by refactoring utility.cpp 2024
// Mathematical utility functions implementation
//

#include "slam2ref/utils/MathUtils.h"
#include <gtsam/geometry/Pose3.h>
#include <cmath>

namespace Slam3refParam {

    int genGlobalNodeIdx(const int& _session_idx, const int& _node_offset)
    {
        return _node_offset + 1 + _session_idx * kSessionStartIdxOffset;
    }

    int ungenGlobalNodeIdx(const int& _session_idx, const int& _idx_in_graph)
    {
        return (_idx_in_graph - 1) / (_session_idx * kSessionStartIdxOffset);
    }

    int genAnchorNodeIdx(const int& _session_idx)
    {
        return _session_idx * kSessionStartIdxOffset;
    }

    std::vector<std::pair<double, int>> sortVecWithIdx(const std::vector<double>& arr)
    {
        std::vector<std::pair<double, int>> vp;
        vp.reserve(arr.size());

        for (size_t i = 0; i < arr.size(); ++i) {
            vp.push_back(std::make_pair(arr[i], static_cast<int>(i)));
        }

        std::sort(vp.begin(), vp.end());
        return vp;
    }

    std::vector<int> linspace(int a, int b, int N)
    {
        std::vector<int> result;
        result.reserve(N);

        double step = static_cast<double>(b - a) / (N - 1);
        for (int i = 0; i < N; ++i) {
            result.push_back(static_cast<int>(a + i * step));
        }
        return result;
    }

    void collect_digits(std::vector<int>& digits, int num)
    {
        if (num > 9) {
            collect_digits(digits, num / 10);
        }
        digits.push_back(num % 10);
    }

    float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2)
    {
        gtsam::Point3 translation1 = p1.translation();
        gtsam::Point3 translation2 = p2.translation();

        double dx = translation1.x() - translation2.x();
        double dy = translation1.y() - translation2.y();
        double dz = translation1.z() - translation2.z();

        return static_cast<float>(std::sqrt(dx*dx + dy*dy + dz*dz));
    }
}
