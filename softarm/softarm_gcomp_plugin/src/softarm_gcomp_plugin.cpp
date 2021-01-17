#include <softarm_gcomp_plugin/softarm_gcomp_plugin.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/JointState.h>
#include <boost/bind.hpp>
#include <math.h>
#include <string>
#include <ros/ros.h>

using namespace gazebo;
using namespace std;

/*


*/

void softarmGcompPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "softhands_Plugin");

  // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Retrieve namespace from urdf tags
  this->ns_name =_sdf->GetElement("namespace")->Get<string>();
  this->j2_name =_sdf->GetElement("joint_2")->Get<string>();
  this->j3_name =_sdf->GetElement("joint_3")->Get<string>();
  this->K_spring2 =_sdf->GetElement("k_2")->Get<double>();
  this->K_spring3 =_sdf->GetElement("k_3")->Get<double>();

  // Everything-is-fine message
  std::string ok_msg = "Gravity compensation plugin on joint " + j2_name + " and " + j3_name + " started!";
  ROS_WARN_STREAM(ok_msg);

  // Compose the string for the joint states subscriber
  pos_sub2_name = ns_name + "/" + j2_name +  "/state";
  pos_sub3_name = ns_name + "/" + j3_name +  "/state";

  // Compose strings for the gravity compensation torque publishers
  tau2_pub_name = ns_name + "/" + j2_name + "/ext_tau";
  tau3_pub_name = ns_name + "/" + j3_name + "/ext_tau";

  // Subscribers and Publishers for the joint states and command
  sub_tau2 = n.subscribe(pos_sub2_name, 10, &softarmGcompPlugin::getRef2_callback, this);
  sub_tau3 = n.subscribe(pos_sub3_name, 10, &softarmGcompPlugin::getRef3_callback, this);
  pub_comp2 = n.advertise<std_msgs::Float64>(tau2_pub_name, 500);
  pub_comp3 = n.advertise<std_msgs::Float64>(tau3_pub_name, 500);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&softarmGcompPlugin::OnUpdate, this, _1));  

}

// Subscriber callbacks to joint positions
void softarmGcompPlugin::getRef2_callback(const std_msgs::Float64& val)
{
    this->joint2_pos = val;
}
void softarmGcompPlugin::getRef3_callback(const std_msgs::Float64& val)
{
    this->joint3_pos = val;
}

// Gravity compensation torque function
void softarmGcompPlugin::ComputeCompTau(double q2, double q3)
{
  // Retrieve spring stiffnesses
  double k2 = this->K_spring2;
  double k3 = this->K_spring3;

  // Ancoraggio molle
  double d2 = 0.031;
  double d3 = 0.0244;
  // Precarichi molle
  double xp2 = -0.0111;
  double xp3 = -0.009;
  // Dist. uscita cavo-centro camma
  double c2 = 0.03615;
  double c3 = 0.0305;
  // Offset angolo
  double gamma = 13.6*M_PI/180;

  // Compute compensation torque from formulas
  this->tau_comp2.data = -( (0.5*c3*d3*k3*sin(gamma + q2 + q3 + acos(d3/c3) + 1.6)*(0.5*xp3 + 0.5*pow((c3*c3 + d3*d3 - 2.0*c3*d3*cos(gamma + q2 + q3 + acos(d3/c3) + 1.6)),(1/2))))/pow((c3*c3 + d3*d3 - 2.0*c3*d3*cos(gamma + q2 + q3 + acos(d3/c3) + 1.6)),(1/2)) + (0.5*c2*d2*k2*sin(q2 + acos(d2/c2))*(0.5*xp2 + 0.5*pow((c2*c2 + d2*d2 - 2.0*c2*d2*cos(q2 + acos(d2/c2))),(1/2))))/pow((c2*c2 + d2*d2 - 2.0*c2*d2*cos(q2 + acos(d2/c2))),(1/2)) ); 
  this->tau_comp3.data = -( (0.5*c3*d3*k3*sin(gamma + q2 + q3 + acos(d3/c3) + 1.6)*(0.5*xp3 + 0.5*pow((c3*c3 + d3*d3 - 2.0*c3*d3*cos(gamma + q2 + q3 + acos(d3/c3) + 1.6)),(1/2))))/pow((c3*c3 + d3*d3 - 2.0*c3*d3*cos(gamma + q2 + q3 + acos(d3/c3) + 1.6)),(1/2)) );

}

// Main update function
void softarmGcompPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{

  // Evaluate the gravity torques from formulas
  softarmGcompPlugin::ComputeCompTau(this->joint2_pos.data, this->joint3_pos.data);

  // Publish compensation torques
  pub_comp2.publish(this->tau_comp2);
  pub_comp3.publish(this->tau_comp3);
}