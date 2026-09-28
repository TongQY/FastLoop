#ifndef SEARCH_AND_FUSE_PLAN_H
#define SEARCH_AND_FUSE_PLAN_H

#include <cstddef>

namespace ORB_SLAM3 {
class KeyFrame;
class MapPoint;
}

struct SearchAndFuseObservation
{
    ORB_SLAM3::KeyFrame* keyframe = nullptr;
    ORB_SLAM3::MapPoint* map_point = nullptr;
    std::size_t feature_index = 0;
};

#endif  // SEARCH_AND_FUSE_PLAN_H
