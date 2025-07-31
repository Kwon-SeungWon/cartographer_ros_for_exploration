#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "ros2_frontier_based_explore/explore.hpp"

using std::placeholders::_1;

namespace ros2_frontier_based_explore
{
  Explore::Explore() : Node("explore")
  {
    // read params
    this->declare_parameter("planner_frequency", 1.0);
    this->declare_parameter("progress_timeout", 30);
    this->declare_parameter("visualize", false);
    this->declare_parameter("potential_scale", 1e-3);
    this->declare_parameter("orientation_scale", 0.0);
    this->declare_parameter("gain_scale", 1.0);
    this->declare_parameter("min_frontier_size", 0.5);
    this->declare_parameter("distance_threshold", 2.0);

    this->get_parameter("planner_frequency", planner_frequency_);
    this->get_parameter("visualize", visualize_);
    this->get_parameter("potential_scale", potential_scale_);
    this->get_parameter("orientation_scale", orientation_scale_);
    this->get_parameter("gain_scale", gain_scale_);
    this->get_parameter("min_frontier_size", min_frontier_size_);
    this->get_parameter("distance_threshold", distance_threshold_);
    if (this->get_parameter("progress_timeout", timeout_sec)) {
        progress_timeout_ = rclcpp::Duration::from_seconds(timeout_sec);
    } else {
        progress_timeout_ = rclcpp::Duration::from_seconds(30); // 기본값 설정
    }

    check_finish_ = 0;
    state = false;
    // init transform server
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // init frontier_exploration
    search_ = FrontierSearch(costmap_.getCostmap(),
                             potential_scale_, gain_scale_,
                             min_frontier_size_);
                             
    last_progress_ = this->now();
    prev_distance_ = std::numeric_limits<double>::infinity();
    total_distance_ = 0.0;
    
    // RCLCPP_ERROR(this->get_logger(), "potential_scale_: %f", potential_scale_);
    // timed callback to make plan
    exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / planner_frequency_)),
      std::bind(&Explore::makePlan, this));

    calculating_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(200.0)), // 100ms
      std::bind(&Explore::distanceCalculator, this));

    // map subscribers
    // ros2 has no waitForMessage yet
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&Explore::mapReceived, this, _1));

    costmap_updates_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        "map_updates", 10, std::bind(&Explore::mapUpdateReceived, this, _1));

    // visualization publisher
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 1);
    finish_pub_ = this->create_publisher<std_msgs::msg::Empty>("explore_finish", 1);

    // action client
    nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "navigate_to_pose");

    // Exploration Resume Subscriber
    resume_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "resume_exploration", 10, std::bind(&Explore::resumeCallback, this, std::placeholders::_1));
  
    exploration_active_ = true;

    last_pose_.position.x = 0.0;
    last_pose_.position.y = 0.0;
    last_pose_.position.z = 0.0;

    // 원점 복귀 상태 플래그 추가
    return_to_origin_ = false;
  }

  void Explore::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Got new map");
    costmap_.updateFullMap(msg,state);
  }

  void Explore::mapUpdateReceived(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Got new partial map");
    costmap_.updatePartialMap(msg);
  }

  void Explore::makePlan()
  {
    if (return_to_origin_) {
      // 이미 원점 복귀 중이면 아무것도 하지 않음
      return;
    }
    if (!exploration_active_) {
      RCLCPP_INFO(this->get_logger(), "Exploration paused.");
      return;
    }
    // RCLCPP_INFO(this->get_logger(), "Making Plan");

    // get current pose
    // geometry_msgs::msg::Pose curr_pose = getRobotPose();
    geometry_msgs::msg::TransformStamped map_to_base_link;
    try
    {
      map_to_base_link = tf_buffer_->lookupTransform(
          "map", "base_link", tf2::TimePointZero);
      // RCLCPP_INFO(this->get_logger(), "Got pose");
    }
    catch (tf2::TransformException &ex)
    {
      // RCLCPP_INFO(this->get_logger(), "No pose");
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = map_to_base_link.transform.translation.x;
    pose.position.y = map_to_base_link.transform.translation.y;
    pose.position.z = map_to_base_link.transform.translation.z;
    
    frontiers = search_.searchFrom(pose.position);

    // RCLCPP_ERROR(this->get_logger(), "Frontiers size: %d", frontiers.size() );
    // stop system when frontiers are empty
    if (frontiers.empty())
    {
      RCLCPP_INFO(this->get_logger(), "frontiers empty");
      check_finish_ ++;
      std_msgs::msg::Empty msgs;
      // 원점 복귀 함수 호출
      goToOrigin();
      return_to_origin_ = true;
      if(check_finish_ > 2)
      {
        if(!state)
          finish_pub_->publish(msgs);
        else
          check_finish_ = 0;
        state = false;
      }
      return;
    }

    // publish
    if (visualize_)
    {
      visualizeFrontiers(frontiers);
    }

    // try to find non blacklisted position to go to
    auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
                                     [this](const Frontier &f) {
                                       return goalOnBlacklist(f.centroid);
                                     });
    if (frontier == frontiers.end())
    {
      // stop();
      std_msgs::msg::Empty msgs;
      finish_pub_->publish(msgs);
      return;
    }


    target_position = frontier->centroid;


    std::lock_guard<std::mutex> lck(mutex_);

    // time out if we are not making any progress
    bool same_goal = prev_goal_ == target_position;
    prev_goal_ = target_position;
    if (!same_goal || prev_distance_ > frontier->min_distance)
    {
      // we have different goal or we made some progress
      last_progress_ = this->now();
      prev_distance_ = frontier->min_distance;
    }
    // black list if we've made no progress for a long time
    if (this->now() - last_progress_ > progress_timeout_)
    {
      frontier_blacklist_.push_back(target_position);
      RCLCPP_INFO(this->get_logger(), "Adding current goal to black list");
      // makePlan();
      return;
    }

    // we don't need to do anything if we still pursuing the same goal
    if (same_goal)
    {
      RCLCPP_INFO(this->get_logger(), "same goal casle");
      return;
    }

    // send goal to nav2
    ClientT::Goal client_goal;
    client_goal.pose.pose.position = target_position;
    client_goal.pose.pose.orientation.w = 1.;
    client_goal.pose.header.frame_id = "map";
    client_goal.pose.header.stamp = this->now();

    auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&Explore::goalResponseCallback, this, std::placeholders::_1);
    future_goal_handle_ =
        nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
  }

  void Explore::goalResponseCallback(
      // std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future)
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future)
  {
    RCLCPP_INFO(this->get_logger(), "Reached Goal");
    // makePlan();
    
  }   

  void Explore::reachgoalResponseCallback(
      // std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future)
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future)
  {
    RCLCPP_INFO(this->get_logger(), "Reached init Goal");
    std_msgs::msg::Empty msgs;
    finish_pub_->publish(msgs);
    return_to_origin_ = false; // 복귀 완료 시 플래그 해제
  }  

  bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point &goal)
  {

    constexpr static size_t tolerance = 100;
    nav2_costmap_2d::Costmap2D *costmap2d = costmap_.getCostmap();

    // check if a goal is on the blacklist for goals that we're pursuing
    for (auto &frontier_goal : frontier_blacklist_)
    {
      double x_diff = fabs(goal.x - frontier_goal.x);
      double y_diff = fabs(goal.y - frontier_goal.y);

      if (x_diff < tolerance * costmap2d->getResolution() &&
          y_diff < tolerance * costmap2d->getResolution())
        return true;
    }
    return false;
  }

  void Explore::visualizeFrontiers(const std::vector<Frontier> &frontiers)
  {
    static size_t prev_marker_count = 0;

    visualization_msgs::msg::MarkerArray markers_msg;

    // recycle m object when adding to marker array
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.ns = "frontier";
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(
        0, 1); // uniform distribution between 0 and 1

    size_t id = 0;
    // add centroid of wavefronts to marker array
    for (auto &wavefront : frontiers)
    {
      m.color.r = dis(gen);
      m.color.g = dis(gen);
      m.color.b = dis(gen);
      for (auto &pt : wavefront.points)
      {
        m.id = id++;
        m.pose.position.x = pt.x;
        m.pose.position.y = pt.y;
        m.pose.position.z = 0.0;
        markers_msg.markers.push_back(m);
      }
    }
    size_t curr_marker_count = markers_msg.markers.size();

    // delete prev. markers
    m.action = visualization_msgs::msg::Marker::DELETE;
    for (; id < prev_marker_count; ++id)
    {
      m.id = int(id);
      markers_msg.markers.push_back(m);
    }

    prev_marker_count = curr_marker_count;
    marker_array_publisher_->publish(markers_msg);
  }

  bool Explore::checkFrontier(geometry_msgs::msg::Point goal, std::vector<ros2_frontier_based_explore::Frontier> frontiers)
  {
    double th = 1.0;
      return true;
    for(auto frontier : frontiers)
    {
      for(auto point : frontier.points)
      {
        double dist = std::hypot(goal.x - point.x, goal.y - point.y);
        if(dist < 1.0)
          return false;
      }
    }
    return true;
  }

  void Explore::distanceCalculator()
  {
    if (return_to_origin_) {
        // 원점 복귀 중에는 블랙리스트 추가 등 아무것도 하지 않음
        return;
    }
      geometry_msgs::msg::TransformStamped map_to_base_link;
      try {
          map_to_base_link = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", ex.what());
          return;
      }

      geometry_msgs::msg::Pose pose;
      pose.position.x = map_to_base_link.transform.translation.x;
      pose.position.y = map_to_base_link.transform.translation.y;
      pose.position.z = map_to_base_link.transform.translation.z;

      double dx = pose.position.x - last_pose_.position.x;
      double dy = pose.position.y - last_pose_.position.y;
      double delta_distance = std::sqrt(dx * dx + dy * dy);

      if (delta_distance > 0.01) {  
          total_distance_ += delta_distance;
          last_pose_ = pose;
      }

      if (delta_distance <= 0.1 && (this->now() - last_progress_) > progress_timeout_){
        frontier_blacklist_.push_back(target_position);
        RCLCPP_INFO(this->get_logger(), "Adding current goal to black list");
      }

      if (total_distance_ >= distance_threshold_) {
          RCLCPP_INFO(this->get_logger(), "Robot traveled %.2fm, pausing exploration.", total_distance_);
          stop();
      }
  }

  
  void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      RCLCPP_INFO(this->get_logger(), "Received resume command. Resuming exploration.");
      total_distance_ = 0;
      exploration_active_ = true;
      exploring_timer_->reset();
    }
  }

  void Explore::stop()
  {
    exploration_active_ = false;
    nav_to_pose_client_->async_cancel_all_goals();
    exploring_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Exploration stopped.");
  }

  // 원점 복귀 함수 정의
  void Explore::goToOrigin()
  {
    ClientT::Goal origin_goal;
    origin_goal.pose.pose.position.x = 0.0;
    origin_goal.pose.pose.position.y = 0.0;
    origin_goal.pose.pose.position.z = 0.0;
    origin_goal.pose.pose.orientation.w = 1.0;
    origin_goal.pose.header.frame_id = "map";
    origin_goal.pose.header.stamp = this->now();
    auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Explore::reachgoalResponseCallback, this, std::placeholders::_1);
    nav_to_pose_client_->async_send_goal(origin_goal, send_goal_options);
  }
} // namespace ros2_frontier_based_explore
