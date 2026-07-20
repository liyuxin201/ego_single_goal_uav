#include <gtest/gtest.h>

#include <Eigen/Eigen>

#include <onboard_detector/lidar_cluster_filter.h>

namespace {

onboardDetector::box3D makeBox(
    float x_width,
    float y_width,
    float z_width) {
    onboardDetector::box3D box;
    box.x_width = x_width;
    box.y_width = y_width;
    box.z_width = z_width;
    return box;
}

}  // namespace

TEST(LidarClusterFilterTest, RemovesClustersLargerThanConfiguredLimit) {
    std::vector<onboardDetector::box3D> boxes{
        makeBox(2.5f, 2.0f, 1.8f),
        makeBox(3.2f, 2.0f, 1.8f),
        makeBox(1.0f, 3.1f, 1.8f),
        makeBox(1.0f, 2.0f, 3.4f),
        makeBox(3.0f, 3.0f, 3.0f),
    };
    std::vector<std::vector<Eigen::Vector3d>> clusters{
        {Eigen::Vector3d(0.0, 0.0, 0.0)},
        {Eigen::Vector3d(1.0, 0.0, 0.0)},
        {Eigen::Vector3d(2.0, 0.0, 0.0)},
        {Eigen::Vector3d(3.0, 0.0, 0.0)},
        {Eigen::Vector3d(4.0, 0.0, 0.0)},
    };
    std::vector<Eigen::Vector3d> centers{
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(2.0, 0.0, 0.0),
        Eigen::Vector3d(3.0, 0.0, 0.0),
        Eigen::Vector3d(4.0, 0.0, 0.0),
    };
    std::vector<Eigen::Vector3d> stds{
        Eigen::Vector3d(0.1, 0.1, 0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1),
    };

    onboardDetector::filterOversizedLidarClusters(
        boxes,
        clusters,
        centers,
        stds,
        Eigen::Vector3d(3.0, 3.0, 3.0));

    ASSERT_EQ(boxes.size(), 2u);
    ASSERT_EQ(clusters.size(), 2u);
    ASSERT_EQ(centers.size(), 2u);
    ASSERT_EQ(stds.size(), 2u);
    EXPECT_FLOAT_EQ(boxes[0].x_width, 2.5f);
    EXPECT_FLOAT_EQ(boxes[1].x_width, 3.0f);
    EXPECT_DOUBLE_EQ(clusters[0][0](0), 0.0);
    EXPECT_DOUBLE_EQ(clusters[1][0](0), 4.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
