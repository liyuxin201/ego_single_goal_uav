#ifndef ONBOARD_DETECTOR_LIDAR_CLUSTER_FILTER_H
#define ONBOARD_DETECTOR_LIDAR_CLUSTER_FILTER_H

#include <deque>
#include <vector>

#include <Eigen/Eigen>

#include <onboard_detector/utils.h>

namespace onboardDetector {

bool isBBoxWithinLimit(const box3D& box, const Eigen::Vector3d& maxSize);

bool shouldUseLidarTrackMotionFallback(
    const std::deque<box3D>& history,
    int frameGap,
    double frameDt,
    double trackVelThresh,
    bool targetSized);

void filterOversizedLidarClusters(
    std::vector<box3D>& boxes,
    std::vector<std::vector<Eigen::Vector3d>>& pcClusters,
    std::vector<Eigen::Vector3d>& pcClusterCenters,
    std::vector<Eigen::Vector3d>& pcClusterStds,
    const Eigen::Vector3d& maxSize);

}  // namespace onboardDetector

#endif
