#ifndef RACECAR_PLUGIN_HPP_
#define RACECAR_PLUGIN_HPP_
 
#include "rclcpp/rclcpp.hpp"

#include <gazebo_ros/node.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
 
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
 namespace gazebo {

    class RacecarPlugin : public gazebo::ModelPlugin
    {
        public:
        RacecarPlugin();
        ~RacecarPlugin() override;
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf) override;
        void update();

        private:
        gazebo::event::ConnectionPtr _update_connection;
        std::shared_ptr<rclcpp::Node> _rosnode;
        gazebo::physics::ModelPtr _model;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_odom;
        gazebo::physics::JointPtr _left_wheel_joint;
        gazebo::physics::JointPtr _right_wheel_joint;
        gazebo::physics::JointPtr _left_steering_hinge_joint;
        gazebo::physics::JointPtr _right_steering_hinge_joint;
        const float _eval_rate=0.01;
        geometry_msgs::msg::Point _pose;
        void publishOdom();
        std::vector<double> ToQuaternion(std::vector<double> &euler);
    };

 
 #endif  // EUFS_PLUGINS_GAZEBO_RACE_CAR_MODEL_INCLUDE_GAZEBO_RACE_CAR_MODEL_GAZEBO_ROS_RACE_CAR_HPP_
}  // namespace gazebo_plugins 