/**
 * map_odom_publisher
 *
 * Publie la transformation map -> odom (TF dynamique, 20 Hz) et permet de la corriger
 * via le service `correct_pose` (recalage bordure).
 *
 * Pourquoi corriger map -> odom et pas odom -> base_link ?
 *   REP 105 : le repère odom doit rester continu (pas de saut). C'est l'odométrie STM32
 *   qui le publie. Les corrections absolues (recalage, lidar...) se font en déplaçant
 *   odom dans map.
 *
 * Calcul de la correction :
 *   map_T_base = map_T_odom * odom_T_base
 *   On veut map_T_base_corrigée, à odom_T_base inchangé, donc :
 *   map_T_odom_nouveau = map_T_base_corrigée * inverse(odom_T_base)
 */

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <krabi_msgs/srv/correct_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace
{
// Ramène un angle dans ]-pi, pi]
double normalizeAngle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

tf2::Transform makeTransform(double x, double y, double theta)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    return tf2::Transform(q, tf2::Vector3(x, y, 0));
}
} // namespace

class MapOdomPublisher : public rclcpp::Node
{
public:
    MapOdomPublisher()
      : Node("map_odom_publisher")
    {
        // Mêmes noms que ceux passés par krabi_launch.py aux autres nœuds.
        const double x = this->declare_parameter("init_pose/x", 0.0);
        const double y = this->declare_parameter("init_pose/y", 0.0);
        const double theta = this->declare_parameter("init_pose/theta", 0.0);
        m_map_frame = this->declare_parameter("map_frame", std::string("map"));
        m_odom_frame = this->declare_parameter("odom_frame", std::string("odom"));
        m_base_frame = this->declare_parameter("base_frame", std::string("base_link"));

        // Au démarrage odom coïncide avec la pose initiale du robot : même valeur que
        // l'ancien static_transform_publisher.
        m_map_T_odom = makeTransform(x, y, theta);
        RCLCPP_INFO(this->get_logger(),
                    "Initial map->odom: x=%.3f y=%.3f theta=%.3f rad",
                    x,
                    y,
                    theta);

        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
        m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer et service dans le groupe par défaut (mutuellement exclusif) : ils ne
        // tournent jamais en même temps. Le mutex protège quand même l'état partagé.
        m_timer = this->create_wall_timer(50ms, [this]() { publishMapOdom(); });

        m_correct_pose_srv = this->create_service<krabi_msgs::srv::CorrectPose>(
          "correct_pose",
          [this](const std::shared_ptr<krabi_msgs::srv::CorrectPose::Request> req,
                 std::shared_ptr<krabi_msgs::srv::CorrectPose::Response> res)
          { correctPose(req, res); });
    }

private:
    void publishMapOdom()
    {
        tf2::Transform map_T_odom;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            map_T_odom = m_map_T_odom;
        }

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = m_map_frame;
        msg.child_frame_id = m_odom_frame;
        msg.transform = tf2::toMsg(map_T_odom);
        m_tf_broadcaster->sendTransform(msg);
    }

    void correctPose(const std::shared_ptr<krabi_msgs::srv::CorrectPose::Request> req,
                     std::shared_ptr<krabi_msgs::srv::CorrectPose::Response> res)
    {
        // 1. Pose actuelle du robot dans odom (dernière valeur disponible)
        tf2::Transform odom_T_base;
        try
        {
            const auto tf_msg
              = m_tf_buffer->lookupTransform(m_odom_frame, m_base_frame, tf2::TimePointZero);
            tf2::fromMsg(tf_msg.transform, odom_T_base);
        }
        catch (const tf2::TransformException& ex)
        {
            res->success = false;
            res->message = std::string("odom->base_link lookup failed: ") + ex.what();
            RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
            return;
        }

        {
        std::lock_guard<std::mutex> lock(m_mutex);

        // 2. Pose actuelle dans map
        const tf2::Transform map_T_base = m_map_T_odom * odom_T_base;
        const double old_x = map_T_base.getOrigin().x();
        const double old_y = map_T_base.getOrigin().y();
        const double old_theta = tf2::getYaw(map_T_base.getRotation());

        // 3. On ne remplace que ce qu'on connaît (ex : contre un mur x, on garde y)
        const double new_x = req->set_x ? req->x : old_x;
        const double new_y = req->set_y ? req->y : old_y;
        const double new_theta = req->set_theta ? normalizeAngle(req->theta) : old_theta;

        // 4. Nouveau map->odom tel que map_T_odom * odom_T_base = pose corrigée
        m_map_T_odom = makeTransform(new_x, new_y, new_theta) * odom_T_base.inverse();

        // 5. Log
        RCLCPP_INFO(this->get_logger(),
                    "correct_pose: old (%.3f, %.3f, %.3f) -> new (%.3f, %.3f, %.3f), "
                    "delta (dx=%.3f m, dy=%.3f m, dtheta=%.3f rad)",
                    old_x,
                    old_y,
                    old_theta,
                    new_x,
                    new_y,
                    new_theta,
                    new_x - old_x,
                    new_y - old_y,
                    normalizeAngle(new_theta - old_theta));

        } // fin de la section protégée par le mutex

        // 6. Publication immédiate, sans attendre le prochain tick du timer
        // (publishMapOdom reprend le mutex lui-même).
        publishMapOdom();

        res->success = true;
        res->message = "map->odom updated";
    }

    std::string m_map_frame, m_odom_frame, m_base_frame;

    std::mutex m_mutex;
    tf2::Transform m_map_T_odom; // protégé par m_mutex

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Service<krabi_msgs::srv::CorrectPose>::SharedPtr m_correct_pose_srv;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
