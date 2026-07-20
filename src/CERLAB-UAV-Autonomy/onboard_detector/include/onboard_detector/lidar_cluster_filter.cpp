#include <onboard_detector/lidar_cluster_filter.h>

#include <algorithm>
#include <cmath>

namespace onboardDetector {

bool isBBoxWithinLimit(const box3D& box, const Eigen::Vector3d& maxSize) {
    return std::abs(box.x_width) <= std::abs(maxSize(0)) &&
           std::abs(box.y_width) <= std::abs(maxSize(1)) &&
           std::abs(box.z_width) <= std::abs(maxSize(2));
}

bool shouldUseLidarTrackMotionFallback(
    const std::deque<box3D>& history,
    int frameGap,
    double frameDt,
    double trackVelThresh,
    bool targetSized) {
    if (!targetSized || frameGap <= 0 || frameDt <= 1e-6 || static_cast<int>(history.size()) <= frameGap) {
        return false;
    }

    const box3D& current = history[0];
    const box3D& previous = history[frameGap];
    double filterSpeed = std::hypot(current.Vx, current.Vy);
    if (!std::isfinite(filterSpeed)) {
        filterSpeed = 0.0;
    }

    double boxSpeed = std::hypot(
        (current.x - previous.x) / frameDt,
        (current.y - previous.y) / frameDt);
    if (!std::isfinite(boxSpeed)) {
        boxSpeed = 0.0;
    }

    int recentFastMotionFrames = 0;
    const int motionHistoryWindow = std::min(4, static_cast<int>(history.size()));
    for (int i = 0; i < motionHistoryWindow; ++i) {
        const double histSpeed = std::hypot(history[i].Vx, history[i].Vy);
        if (std::isfinite(histSpeed) && histSpeed >= trackVelThresh) {
            ++recentFastMotionFrames;
        }
    }

    const bool stableTrackedMotion =
        motionHistoryWindow >= 2 &&
        recentFastMotionFrames >= 2 &&
        filterSpeed >= trackVelThresh &&
        boxSpeed >= 0.8 * trackVelThresh;
    const bool trackedVelocityMotion =
        filterSpeed >= trackVelThresh &&
        (boxSpeed >= 0.8 * trackVelThresh || recentFastMotionFrames >= 2);

    return stableTrackedMotion || trackedVelocityMotion;
}

void filterOversizedLidarClusters(
    std::vector<box3D>& boxes,
    std::vector<std::vector<Eigen::Vector3d>>& pcClusters,
    std::vector<Eigen::Vector3d>& pcClusterCenters,
    std::vector<Eigen::Vector3d>& pcClusterStds,
    const Eigen::Vector3d& maxSize) {
    std::vector<box3D> keptBoxes;
    std::vector<std::vector<Eigen::Vector3d>> keptClusters;
    std::vector<Eigen::Vector3d> keptCenters;
    std::vector<Eigen::Vector3d> keptStds;

    keptBoxes.reserve(boxes.size());
    keptClusters.reserve(pcClusters.size());
    keptCenters.reserve(pcClusterCenters.size());
    keptStds.reserve(pcClusterStds.size());

    for (size_t i = 0; i < boxes.size(); ++i) {
        if (!isBBoxWithinLimit(boxes[i], maxSize)) {
            continue;
        }

        keptBoxes.push_back(boxes[i]);
        if (i < pcClusters.size()) {
            keptClusters.push_back(pcClusters[i]);
        }
        if (i < pcClusterCenters.size()) {
            keptCenters.push_back(pcClusterCenters[i]);
        }
        if (i < pcClusterStds.size()) {
            keptStds.push_back(pcClusterStds[i]);
        }
    }

    boxes.swap(keptBoxes);
    pcClusters.swap(keptClusters);
    pcClusterCenters.swap(keptCenters);
    pcClusterStds.swap(keptStds);
}

}  // namespace onboardDetector
