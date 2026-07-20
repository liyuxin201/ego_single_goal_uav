#include <gtest/gtest.h>

#include <deque>

#include <onboard_detector/lidar_cluster_filter.h>

namespace {

onboardDetector::box3D makeTrackedBox(
    float x,
    float y,
    float x_width,
    float y_width,
    float z_width,
    float vx,
    float vy) {
    onboardDetector::box3D box;
    box.x = x;
    box.y = y;
    box.x_width = x_width;
    box.y_width = y_width;
    box.z_width = z_width;
    box.Vx = vx;
    box.Vy = vy;
    return box;
}

}  // namespace

TEST(DynamicDetectorLidarClassificationTest, UsesTrackMotionFallbackForFastConsistentTrack) {
    std::deque<onboardDetector::box3D> history;
    history.push_front(makeTrackedBox(2.40f, 0.00f, 0.50f, 0.70f, 1.10f, 0.20f, -2.00f));
    history.push_back(makeTrackedBox(2.38f, 0.20f, 0.50f, 0.70f, 1.10f, 0.18f, -1.95f));
    history.push_back(makeTrackedBox(2.35f, 0.41f, 0.50f, 0.70f, 1.10f, 0.19f, -2.10f));

    EXPECT_TRUE(onboardDetector::shouldUseLidarTrackMotionFallback(
        history,
        1,
        0.1,
        0.1,
        true));
}

TEST(DynamicDetectorLidarClassificationTest, RejectsSlowTrackWithoutPointSupport) {
    std::deque<onboardDetector::box3D> history;
    history.push_front(makeTrackedBox(3.20f, 4.30f, 0.25f, 1.15f, 2.35f, 0.01f, 0.02f));
    history.push_back(makeTrackedBox(3.20f, 4.30f, 0.25f, 1.15f, 2.35f, 0.01f, 0.02f));

    EXPECT_FALSE(onboardDetector::shouldUseLidarTrackMotionFallback(
        history,
        1,
        0.1,
        0.1,
        true));
}

TEST(DynamicDetectorLidarClassificationTest, RejectsOversizedTrackEvenWhenVelocityIsHigh) {
    std::deque<onboardDetector::box3D> history;
    history.push_front(makeTrackedBox(1.80f, 0.30f, 3.50f, 1.10f, 1.40f, 0.30f, -1.20f));
    history.push_back(makeTrackedBox(1.75f, 0.42f, 3.50f, 1.10f, 1.40f, 0.28f, -1.10f));
    history.push_back(makeTrackedBox(1.70f, 0.54f, 3.50f, 1.10f, 1.40f, 0.31f, -1.25f));

    EXPECT_FALSE(onboardDetector::shouldUseLidarTrackMotionFallback(
        history,
        1,
        0.1,
        0.1,
        false));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
