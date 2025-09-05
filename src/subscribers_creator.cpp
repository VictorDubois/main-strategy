#include "core.h"

void Core::create_subscribers()
{

    std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_lidar_sub_func
      = std::bind(&Core::updateLidar, this, std::placeholders::_1, true);
    std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_lidar_behind_sub_func
      = std::bind(&Core::updateLidar, this, std::placeholders::_1, false);

    m_lidar_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "obstacle_pose_stamped", 5, l_lidar_sub_func); //, l_sub_options);
    m_lidar_behind_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "obstacle_behind_pose_stamped", 5, l_lidar_behind_sub_func); //, l_sub_options);
    m_tirette_sub = this->create_subscription<std_msgs::msg::Bool>(
      "tirette",
      5,
      std::bind(&Core::updateTirette, this, std::placeholders::_1)); //, l_sub_options);
    m_odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 5, std::bind(&Core::updateOdom, this, std::placeholders::_1)); //, l_sub_options);
    m_odometry_slash_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 5, std::bind(&Core::updateOdom, this, std::placeholders::_1)); //, l_sub_options);
    m_strat_movement_sub = this->create_subscription<krabi_msgs::msg::StratMovement>(
      "strat_movement",
      5,
      std::bind(&Core::updateStratMovement, this, std::placeholders::_1)); //, l_sub_options);
}

void Core::create_aruco_subscribers()
{
#ifdef USE_ARCUO
    if (!m_is_blue)
    {
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_6_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 6);
        m_arucos_sub[6] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r6", 5, l_arucos_6_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_7_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 7);
        m_arucos_sub[7] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r7", 5, l_arucos_7_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_8_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 8);
        m_arucos_sub[8] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r8", 5, l_arucos_8_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_9_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 9);
        m_arucos_sub[9] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r9", 5, l_arucos_9_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_10_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 10);
        m_arucos_sub[10] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r10", 5, l_arucos_10_func); //, l_sub_options);
    }
    else
    {
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_1_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 1);
        m_arucos_sub[1] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r1", 5, l_arucos_1_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_2_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 2);
        m_arucos_sub[2] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r2", 5, l_arucos_2_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_3_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 3);
        m_arucos_sub[3] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r3", 5, l_arucos_3_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_4_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 4);
        m_arucos_sub[4] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r4", 5, l_arucos_4_func); //, l_sub_options);
        std::function<void(std::shared_ptr<geometry_msgs::msg::PoseStamped>)> l_arucos_5_func
          = std::bind(&Core::updateAruco, this, std::placeholders::_1, 5);
        m_arucos_sub[5] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/pose_robots/r5", 5, l_arucos_5_func); //, l_sub_options);
    }
#endif // USE_ARCUO
}