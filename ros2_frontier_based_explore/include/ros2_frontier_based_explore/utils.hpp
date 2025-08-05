#pragma once

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace ros2_frontier_based_explore
{
    std::vector<unsigned int> nhood4(unsigned int idx,
                                     const nav2_costmap_2d::Costmap2D &costmap);

    std::vector<unsigned int> nhood8(unsigned int idx,
                                     const nav2_costmap_2d::Costmap2D &costmap);

    bool nearestCell(unsigned int &result, unsigned int start, unsigned char val,
                     const nav2_costmap_2d::Costmap2D &costmap);

} // namespace ros2_frontier_based_explore