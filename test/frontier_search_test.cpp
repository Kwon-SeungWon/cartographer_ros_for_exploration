#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ros2_frontier_based_explore/frontier_search.hpp"

#include <gtest/gtest.h>


using namespace ros2_frontier_based_explore;

TEST(FrontierSearchTest, EasyTest)
{
    FrontierSearch();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}