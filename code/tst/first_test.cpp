
#include <iostream>
#include <goc/goc.h>
#include <gtest/gtest.h>

#include "preprocess/preprocess_travel_times.h"

using namespace networks2019;

TEST(FirstTest, Dummy) {
    /*
     * 3 nodes.
     * From A you can go to B, and from B to C.
     * It's expected that 'travel_times' has an arc from A to C.
     */
    // clusters
    // cluster_speeds
    // speed_zones
    // distances

    nlohmann::json j;
    j["digraph"]["vertex_count"] = 3;
    j["digraph"]["arcs"] = { {0, 1, 0}, {0, 0, 1}, {0, 0, 0} };
    j["distances"] = { {0.0, 10.0, 0.0}, {0.0, 0.0, 10.0}, {0.0, 0.0, 0.0} };

    j["horizon"] = { 0, 1000 };
    j["speed_zones"] = { {0, 1000} };

    j["cluster_count"] = 1;
    j["clusters"] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
    j["cluster_speeds"] = { {1} };
    std::cout << j << "\n";

    preprocess_travel_times(j);

    ASSERT_EQ(1, 1);
}

TEST(FirstTest, SecondDummy) {
    ASSERT_EQ(1, 0);
    ASSERT_TRUE(false);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
