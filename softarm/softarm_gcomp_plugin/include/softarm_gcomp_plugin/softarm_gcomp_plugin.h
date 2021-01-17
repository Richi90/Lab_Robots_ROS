#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo{
  
  class softarmGcompPlugin : public ModelPlugin {
  
  public:
    // Constructor
    softarmGcompPlugin() : ModelPlugin(){}

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate(const common::UpdateInfo & /*_info*/);

    void getRef2_callback(const std_msgs::Float64& val);

    void getRef3_callback(const std_msgs::Float64& val);

    void ComputeCompTau(double q2, double q3);

  private:

    // Command name to create a specific subscriber
    std::string pos_sub2_name, pos_sub3_name;
    std::string tau2_pub_name, tau3_pub_name;

    // Namespace retrieved from the urdf
    std::string ns_name;
    std::string j2_name, j3_name;

    // Pointer to the model 
    physics::ModelPtr model;

    // Pointer to output shaft joint 
    physics::JointPtr joint;

    // Pointer to the update event connection 
    event::ConnectionPtr updateConnection;    

    // Define publisher and subscriber
    ros::Publisher pub_comp2, pub_comp3;    // for compensation torque on joint 2 & 3
    ros::Subscriber sub_tau2, sub_tau3;     // for joint position reference
    ros::NodeHandle n;
    
    // Position messages
    std_msgs::Float64 joint2_pos, joint3_pos;

    // Messages to publish compensation torque
    std_msgs::Float64 tau_comp2, tau_comp3;

    // Spring stiffnesses
    double K_spring2, K_spring3;

  };

  GZ_REGISTER_MODEL_PLUGIN(softarmGcompPlugin)
}