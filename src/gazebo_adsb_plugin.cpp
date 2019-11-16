#include "gazebo_adsb_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboAdsbPlugin::GazeboAdsbPlugin()
    : ModelPlugin()
{
}

GazeboAdsbPlugin::~GazeboAdsbPlugin() {
  updateConnection_->~Connection();
}


void GazeboAdsbPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_adsb_plugin] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_adsb_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_adsb_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  getSdfParam<double>(_sdf, "FullWeight", full_weight_, full_weight_);
  getSdfParam<double>(_sdf, "SprayRate", spray_rate_, spray_rate_);
  getSdfParam<double>(_sdf, "EmptyWeight", empty_weight_, empty_weight_);

  #if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  #else
  last_time_ = world_->GetSimTime();
  #endif

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboAdsbPlugin::OnUpdate, this, _1));

  gravity_W_ = world_->Gravity();

}

// This gets called by the world update start event.
void GazeboAdsbPlugin::OnUpdate(const common::UpdateInfo& _info) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time  = world_->SimTime();
#else
  common::Time current_time  = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  
  //TODO: This time should not be double, but measured only when spraying
  double t = current_time.Double();

  //TODO: Consider sloshing inside the tank?
  double current_mass = std::max(full_weight_ - spray_rate_ * t, empty_weight_);
  physics::InertialPtr inertia_ptr = link_->GetInertial();

  inertia_ptr->gazebo::physics::Inertial::SetMass(current_mass);
  inertia_ptr->gazebo::physics::Inertial::SetCoG(ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  inertia_ptr->gazebo::physics::Inertial::SetIXX(0.1);
  inertia_ptr->gazebo::physics::Inertial::SetIYY(0.2);
  inertia_ptr->gazebo::physics::Inertial::SetIZZ(0.3);
  inertia_ptr->gazebo::physics::Inertial::SetIXY(0.0);
  inertia_ptr->gazebo::physics::Inertial::SetIXZ(0.0);
  inertia_ptr->gazebo::physics::Inertial::SetIYZ(0.0);
 
  //TODO: Check if this is really needed
  link_->gazebo::physics::Link::UpdateMass();
}
GZ_REGISTER_MODEL_PLUGIN(GazeboAdsbPlugin);
}