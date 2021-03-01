
#include <iostream>
#include <goc/goc.h>
#include <gtest/gtest.h>

#include "preprocess/preprocess_travel_times.h"

using namespace networks2019;
using namespace goc;

TEST(FirstTest, Dummy) {
    /*
     * 3 nodes.
     * From A you can go to B in 10, and from B to C in 10.
     * It's expected that 'travel_times' has an arc from A to C in 20.
     */

    nlohmann::json j;
    j["digraph"]["vertex_count"] = 3;
    j["digraph"]["arcs"] = { {0, 1, 0}, {0, 0, 1}, {0, 0, 0} };
    j["distances"] = { {0.0, 10.0, 0.0}, {0.0, 0.0, 10.0}, {0.0, 0.0, 0.0} };

    j["horizon"] = { 0, 1000 };
    j["speed_zones"] = { {0, 1000} };

    j["cluster_count"] = 1;
    j["clusters"] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
    j["cluster_speeds"] = { {1} };

    preprocess_travel_times(j);

    goc::Matrix<goc::PWLFunction> m = j["travel_times"];
    ASSERT_FALSE(m[0][2].Empty());
    ASSERT_TRUE(m[2][0].Empty());

    goc::PWLFunction expected = goc::PWLFunction::ConstantFunction(20, goc::Interval(0, 1000-20));
    ASSERT_EQ(expected, m[0][2]);
}

TEST(FirstTest, Dummy2) {
    /*
     * 3 nodes.
     * From A you can go to B in 10, and from B to C in 10.
     * It's expected that 'travel_times' has an arc from A to C in 20.
     */

    nlohmann::json digraph;
    digraph["vertex_count"] = 3;
    digraph["arcs"] = { {0, 1, 0}, {0, 0, 1}, {0, 0, 0} };

    double horizon_end = 1000;
    Interval horizon(0, horizon_end);

    Matrix<PWLFunction> travel_times(3, 3);
    travel_times[0][1] = PWLFunction::ConstantFunction(10, Interval(0, horizon_end - 10));
    travel_times[0][1] = travel_times[0][1] + PWLFunction::IdentityFunction(travel_times[0][1].Domain());

    travel_times[1][2] = PWLFunction::ConstantFunction(10, Interval(0, horizon_end - 10));
    travel_times[1][2] = travel_times[1][2] + PWLFunction::IdentityFunction(travel_times[1][2].Domain());

    Matrix<PWLFunction> res = quickest_paths(digraph, travel_times, horizon);

    goc::PWLFunction expected = goc::PWLFunction::ConstantFunction(20, goc::Interval(0, horizon_end - 20));
    ASSERT_EQ(expected, res[0][2]);

}

TEST(FirstTest, Dummy3) {
    /*
     * 3 nodes.
     * From A you can go to B in 10, and from B to C in 10.
     * It's expected that 'travel_times' has an arc from A to C in 20.
     */

    nlohmann::json digraph;
    digraph["vertex_count"] = 3;
    digraph["arcs"] = { {0, 1, 0}, {0, 0, 1}, {0, 0, 0} };

    double horizon_end = 100;
    Interval horizon(0, horizon_end);

    Matrix<PWLFunction> travel_times(3, 3);
    travel_times[0][1].AddPiece(LinearFunction(Point2D(0, 10), Point2D(10, 20)));
    travel_times[0][1].AddPiece(LinearFunction(Point2D(10, 20), Point2D(horizon_end-20, 20)));
    travel_times[0][1] = travel_times[0][1] + PWLFunction::IdentityFunction(travel_times[0][1].Domain());

    travel_times[1][2].AddPiece(LinearFunction(Point2D(0, 10), Point2D(20, 5)));
    travel_times[1][2].AddPiece(LinearFunction(Point2D(20, 5), Point2D(horizon_end-5, 5)));
    travel_times[1][2] = travel_times[1][2] + PWLFunction::IdentityFunction(travel_times[1][2].Domain());

    Matrix<PWLFunction> res = quickest_paths(digraph, travel_times, horizon);

    goc::PWLFunction expected;
    expected.AddPiece(LinearFunction(Point2D(0, 10+7.5), Point2D(5, 15+5)));
    expected.AddPiece(LinearFunction(Point2D(5, 15+5), Point2D(10, 20+5)));
    expected.AddPiece(LinearFunction(Point2D(10, 20+5), Point2D(horizon_end - (20+5), 20+5)));
    ASSERT_EQ(expected, res[0][2]);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
