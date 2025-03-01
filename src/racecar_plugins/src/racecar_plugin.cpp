#include <memory>
#include <racecar_plugins/racecar_plugin.hpp>

#define GREEN "\033[1;32m"
#define RESET "\033[0m"

namespace gazebo
{
    RacecarPlugin::RacecarPlugin(){
        std::cout<<GREEN<<"Initialized Racecar Plugin"<<RESET<<std::endl;
    }
    RacecarPlugin::~RacecarPlugin(){
        _update_connection.reset();
        std::cout<<"destroy"<<std::endl;
    }
    void RacecarPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        _rosnode = gazebo_ros::Node::Get(sdf);
        
        _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RacecarPlugin::update, this));
        
        _model = model;

        _pub_odom =
            _rosnode->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
    }
    void RacecarPlugin::update() {
        
        publishOdom();
        // _model->SetWorldPose(ignition::math::Pose3d(x+0.0001, y-0.0001, z, 0.0, 0.0, 0.0));
        
    }
    void RacecarPlugin::publishOdom(){
        ignition::math::Pose3d _offset = _model->WorldPose();
        
        nav_msgs::msg::Odometry _odom;
        _odom.pose.pose.position.x = _offset.Pos()[0];
        _odom.pose.pose.position.y = _offset.Pos()[1];
        _odom.pose.pose.position.z = _offset.Pos()[2];
        
        std::vector<double> euler = {0,0,_offset.Rot().Yaw()}; 
        std::vector<double> quat = ToQuaternion(euler);
        _odom.pose.pose.orientation.x = quat[0];
        _odom.pose.pose.orientation.y = quat[1];
        _odom.pose.pose.orientation.z = quat[2];
        _odom.pose.pose.orientation.w = quat[3];

        ignition::math::Vector3 _linear_vel = _model->WorldLinearVel();
        _odom.twist.twist.linear.x = _linear_vel[0];
        _odom.twist.twist.linear.y = _linear_vel[1];
        _odom.twist.twist.linear.z = _linear_vel[2];

        ignition::math::Vector3 _angular_vel = _model->WorldAngularVel();
        _odom.twist.twist.angular.x = _angular_vel[0];
        _odom.twist.twist.angular.y = _angular_vel[1];
        _odom.twist.twist.angular.z = _angular_vel[2];

        _pub_odom->publish(_odom);
    }
    std::vector<double> RacecarPlugin::ToQuaternion(std::vector<double> &euler) {
        // Abbreviations for the various angular functions
        double cy = cos(euler[0] * 0.5);
        double sy = sin(euler[0] * 0.5);
        double cp = cos(euler[1] * 0.5);
        double sp = sin(euler[1] * 0.5);
        double cr = cos(euler[2] * 0.5);
        double sr = sin(euler[2] * 0.5);
      
        std::vector<double> q;
        q.reserve(4);
        q[0] = cy * cp * sr - sy * sp * cr;  // x
        q[1] = sy * cp * sr + cy * sp * cr;  // y
        q[2] = sy * cp * cr - cy * sp * sr;  // z
        q[3] = cy * cp * cr + sy * sp * sr;  // w
      
        return q;
      }
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(RacecarPlugin)
}
