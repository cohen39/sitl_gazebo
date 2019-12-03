/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_LIFT_DRAG_PLUGIN_HH_
#define _GAZEBO_LIFT_DRAG_PLUGIN_HH_

#include <string>
#include <vector>
#include <ctime>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE LiftDragPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: LiftDragPlugin();

    /// \brief Destructor.
    public: ~LiftDragPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    // coefficient of lift
    protected: double cl;

    // coefficient of drag
    protected: double cd;

    // pitch moment coefficient
    protected: double cm;

    //  lift coefficient for angle of attack
    protected: double cla;

    // lift coefficient at 0 angle of attack
    protected: double cl0;

    // lift coefficient for alerion deflection
    protected: double clda;

    // lift coefficient for elevator deflection
    protected: double clde;

    // drag coefficient for cl^2
    protected: double cd_a;

    // drag coefficient for cl
    protected: double cd_b;

    // drag coefficient at cl = 0
    protected: double cd_c;

    // pitch moment coefficient for angle of attack
    protected: double cma;

    // pitch moment coefficient at 0 angle of attack
    protected: double cm0;

    // pitch moment coefficient for aileron deflection
    protected: double cmda;

    // Pitch moment coeffecient for elevator deflection
    protected: double cmde;

    // Roll moment coefficient for difference in aileron deflection
    protected: double crda;

    // Yaw moment coefficient for rudder deflection
    protected: double cydr;

    // angle of left aileron deflection (positive increases lift)
    protected: double dal;

    // angle of right aileron deflection (positive increases lift)
    protected: double dar;

    // angle of elevator deflection (positive increases lift)
    protected: double de;

    // angle of rudder deflection (positive is unkown...)
    protected: double dr;

    /// \brief angle of attach when airfoil stalls
    protected: double alphaStall;

    /// \brief Cl-alpha rate after stall
    protected: double claStall;

    /// \brief Cd-alpha rate after stall
    protected: double cdaStall;

    /// \brief Cm-alpha rate after stall
    protected: double cmaStall;

    /// \brief: \TODO: make a stall velocity curve
    protected: double velocityStall;

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    protected: double rho;

    /// \brief if the shape is aerodynamically radially symmetric about
    /// the forward direction. Defaults to false for wing shapes.
    /// If set to true, the upward direction is determined by the
    /// angle of attack.
    protected: bool radialSymmetry;

    /// \brief effective planeform surface area
    protected: double area;

    /// \brief angle of sweep
    protected: double sweep;

    /// \brief initial angle of attack
    protected: double alpha0;

    /// \brief angle of attack
    protected: double alpha;

    /// \brief center of pressure in link local coordinates
    protected: ignition::math::Vector3d cp;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight.
    protected: ignition::math::Vector3d forward;

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector. Inflow velocity orthogonal to forward and upward vectors
    /// is considered flow in the wing sweep direction.
    protected: ignition::math::Vector3d upward;

    /// \brief Smoothed velocity
    protected: ignition::math::Vector3d velSmooth;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief Pointer to a joint that actuates a control surface for
    /// this lifting body
    protected: std::string lAileronJointName;

    protected: std::string rAileronJointName;

    protected: std::string elevatorJointName;

    protected: std::string rudderJointName;

    protected: physics::JointPtr lAileronJoint;

    protected: physics::JointPtr rAileronJoint;

    protected: physics::JointPtr elevatorJoint;

    protected: physics::JointPtr rudderJoint;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    protected: long double lastTime;
  };
}
#endif
