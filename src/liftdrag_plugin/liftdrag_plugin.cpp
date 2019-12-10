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

#include <algorithm>
#include <string>
#include <ctime>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_plugin/liftdrag_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)

/////////////////////////////////////////////////
LiftDragPlugin::LiftDragPlugin()
{
  this->cla =  0;
  this->cl0 =  0;
  this->clda =  0;
  this->clde =  0;
  this->cd_a =  0;
  this->cd_b =  0;
  this->cd_c =  0;
  this->cma0 =  0;
  this->cma1 =  0;
  this->cma2 =  0;
  this->cma3 =  0;
  this->cma4 =  0;
  this->cma5 =  0;
  this->cma6 =  0;
  this->cmda =  0;
  this->cmde =  0;
  this->crda =  0;
  this->cydr =  0;
  this->forward = ignition::math::Vector3d(1, 0, 0);
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->area = 0;
  this->alpha = 0;
  this->sweep = 0;
  this->velocityStall = 0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  this->radialSymmetry = false;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;

  this->lastTime = 0;

}

/////////////////////////////////////////////////
LiftDragPlugin::~LiftDragPlugin()
{
}

/////////////////////////////////////////////////
void LiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  this->physics = this->world->Physics();
#else
  this->physics = this->world->GetPhysicsEngine();
#endif
  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cl0"))
    this->cl0 = _sdf->Get<double>("cl0");

  if (_sdf->HasElement("clda"))
    this->clda = _sdf->Get<double>("clda");

  if (_sdf->HasElement("clde"))
    this->clde = _sdf->Get<double>("clde");

  if (_sdf->HasElement("cd_a"))
    this->cd_a = _sdf->Get<double>("cd_a");

  if (_sdf->HasElement("cd_b"))
    this->cd_b = _sdf->Get<double>("cd_b");

  if (_sdf->HasElement("cd_c"))
    this->cd_c = _sdf->Get<double>("cd_c");

  if (_sdf->HasElement("cma6"))
    this->cma6 = _sdf->Get<double>("cma6");

  if (_sdf->HasElement("cma5"))
    this->cma5 = _sdf->Get<double>("cma5");

  if (_sdf->HasElement("cma4"))
    this->cma4 = _sdf->Get<double>("cma4");

  if (_sdf->HasElement("cma3"))
    this->cma3 = _sdf->Get<double>("cma3");

  if (_sdf->HasElement("cma2"))
    this->cma2 = _sdf->Get<double>("cma2");

  if (_sdf->HasElement("cma1"))
    this->cma1 = _sdf->Get<double>("cma1");

  if (_sdf->HasElement("cma0"))
    this->cma0 = _sdf->Get<double>("cma0");

  if (_sdf->HasElement("cmda"))
    this->cmda = _sdf->Get<double>("cmda");

  if (_sdf->HasElement("cmde"))
    this->cmde = _sdf->Get<double>("cmde");

  if (_sdf->HasElement("crda"))
    this->crda = _sdf->Get<double>("crda");

  if (_sdf->HasElement("cyb"))
    this->cyb = _sdf->Get<double>("cyb");

  if (_sdf->HasElement("cydr"))
    this->cydr = _sdf->Get<double>("cydr");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("chord"))
    this->chord  = _sdf->Get<double>("chord");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragPlugin::OnUpdate, this));
    }
  }

  if (_sdf->HasElement("left_elevon_name"))
  {
    std::string lAileronJointName = _sdf->Get<std::string>("left_elevon_name");
    this->lAileronJoint = this->model->GetJoint(lAileronJointName);
    if (!this->lAileronJoint)
    {
      gzerr << "Joint with name[" << lAileronJointName << "] does not exist.\n";
    }
  }

  if (_sdf->HasElement("right_elevon_name"))
  {
    std::string rAileronJointName = _sdf->Get<std::string>("right_elevon_name");
    this->rAileronJoint = this->model->GetJoint(rAileronJointName);
    if (!this->rAileronJoint)
    {
      gzerr << "Joint with name[" << rAileronJointName << "] does not exist.\n";
    }
  }
  if (_sdf->HasElement("elevator_name"))
  {
    std::string elevatorJointName = _sdf->Get<std::string>("elevator_name");
    this->elevatorJoint = this->model->GetJoint(elevatorJointName);
    if (!this->elevatorJoint)
    {
      gzerr << "Joint with name[" << elevatorJoint << "] does not exist.\n";
    }
  }

  if (_sdf->HasElement("rudder_name"))
  {
    std::string rudderJointName = _sdf->Get<std::string>("rudder_name");
    this->rudderJoint = this->model->GetJoint(rudderJointName);
    if (!this->rudderJoint)
    {
      gzerr << "Joint with name[" << rudderJoint << "] does not exist.\n";
    }
  }
}

/////////////////////////////////////////////////
void LiftDragPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel = this->link->WorldLinearVel(ignition::math::Vector3d(0, 0, 0));
#else
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(ignition::math::Vector3d(0, 0, 0)));
#endif
  ignition::math::Vector3d velI = vel;
  velI.Normalize();

  // smoothing
  // double e = 0.8;
  // this->velSmooth = e*vel + (1.0 - e)*velSmooth;
  // vel = this->velSmooth;

  if (vel.Length() <= 0.01)
    return;

  // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);

  ignition::math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    ignition::math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = ignition::math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  // get cos from trig identity
  double cosSweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  this->sweep = asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*velI;
  ignition::math::Vector3d velInSDPlane = vel - vel.Dot(upwardI)*velI;

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
  liftI.Normalize();

  // get direction of moment
  ignition::math::Vector3d pitchMomentDirection = spanwiseI; // CHECK ACCURACY OF DIRECTION LATER
  ignition::math::Vector3d rollMomentDirection = forwardI; // CHECK ACCURACY OF DIRECTION LATER
  ignition::math::Vector3d yawMomentDirection = upwardI; // CHECK ACCURACY OF DIRECTION LATER

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  double sinBeta = ignition::math::clamp(velI.Dot(spanwiseI), minRatio, maxRatio);

  this->beta = -1*asin(sinBeta);

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  if (liftI.Dot(forwardI) >= 0.0)
    this->alpha = acos(cosAlpha);
  else
    this->alpha = 0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double speedInSDPlane = velInSDPlane.Length();
  double q_LD = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;
  double q_SD = 0.5 * this->rho * speedInSDPlane * speedInSDPlane;

#if GAZEBO_MAJOR_VERSION >= 9
    dal = this->lAileronJoint->Position(0);
    dar = this->rAileronJoint->Position(0);
    de = this->elevatorJoint->Position(0);
    dr = this->rudderJoint->Position(0);
#else
    dal = this->lAileronJoint->GetAngle(0).Radian();
    dar = this->rAileronJoint->GetAngle(0).Radian();
    de = this->elevatorJoint->GetAngle(0).Radian();
    dr = this->rudderJoint->GetAngle(0).Radian();
#endif

  // compute cl at cp, check for stall, correct for sweep
  double cl = 0;
  cl = this->cla * this->alpha + this->cl0 + this->clda * (this->dal + this->dar) + this->clde * this->de;

  // modify cl per control joint value
/*
  if (this->controlJoint)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    double controlAngle = this->controlJoint->Position(0);
#else
    double controlAngle = this->controlJoint->GetAngle(0).Radian();
#endif
    cl = cl + this->controlJointRadToCL * controlAngle;
    /// \TODO: also change cm and cd
  }
*/

  // compute lift force at cp
  ignition::math::Vector3d lift = cl * q_LD * this->area * liftI;

  // compute cd at cp, check for stall, correct for sweep
  double cd = 0;
  cd = this->cd_a*cl*cl + this->cd_b*cl + this->cd_c;

  // drag
  ignition::math::Vector3d drag = cd * q_LD * this->area * dragDirection;

  //compute cm at cg
  double cm = 0;
  double alpha2 = this->alpha*this->alpha;
  double alpha3 = this->alpha*alpha2;
  double alpha4 = this->alpha*alpha3;
  double alpha5 = this->alpha*alpha4;
  double alpha6 = this->alpha*alpha5;

  cm = 
  this->cma6*alpha6 +
  this->cma5*alpha5 +
  this->cma4*alpha4 +
  this->cma3*alpha3 +
  this->cma2*alpha2 +
  this->cma1*this->alpha +
  this->cma0 +
  this->cmda * (this->dal + this->dar) + this->cmde * this->de;

  // double cm = 0;

  double cr;
  cr = this->crda * (this->dal - this->dar);

  double cy;
  cy = this->cydr * this->dr + this->cyb * this->beta;

  // compute moments (torque) at cp
  ignition::math::Vector3d pitchMoment = cm * q_LD * this->area * this->chord * pitchMomentDirection;
  ignition::math::Vector3d rollMoment = cr * q_LD * this->area * this->chord * rollMomentDirection;
  ignition::math::Vector3d yawMoment = cy * q_SD * this->area * this->chord * yawMomentDirection;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d cog = this->link->GetInertial()->CoG();
#else
  ignition::math::Vector3d cog = ignitionFromGazeboMath(this->link->GetInertial()->GetCoG());
#endif

  // force and torque about cg in inertial frame
  ignition::math::Vector3d force = lift + drag;

  ignition::math::Vector3d torque = pitchMoment + rollMoment + yawMoment;

  long double curTime = time(0);
  if (1 && (curTime-lastTime)>0.1)
  {
    lastTime = curTime;

    gzdbg << "=============================\n";
    gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    gzdbg << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure (LD): [" << q_LD << "]\n"
          << "] dynamic pressure (SD): [" << q_SD << "]\n";
    gzdbg << "spd: [" << vel.Length()
          << "] vel: [" << vel << "]\n";
    gzdbg << "LD plane spd: [" << velInLDPlane.Length()
          << "] vel : [" << velInLDPlane << "]\n";
    gzdbg << "forward (inertial): " << forwardI << "\n";
    gzdbg << "upward (inertial): " << upwardI << "\n";
    gzdbg << "lift dir (inertial): " << liftI << "\n";
    gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
    gzdbg << "sweep: " << this->sweep << "\n";
    gzdbg << "alpha: " << this->alpha << "\n";
    gzdbg << "beta: " << this->beta << "\n";
    gzdbg << "lift: " << lift << "\n"
          << "cl:" << cl << "\n";
    gzdbg << "drag: " << drag << " cd: " << cd << "\n" 
          <<" cd_a: " << this->cd_a << "\n"
          << " cd_b: " << this->cd_b << "\n"
          " cd_c: " << this->cd_c << "\n";
    gzdbg << "cm: " << cm << "\n";
    gzdbg << "cr: " << cr << "\n";
    gzdbg << "cy: " << cy << "\n";
    gzdbg << "dal: " << this->dal << "\n";
    gzdbg << "dar: " << this->dar << "\n";
    gzdbg << "de: " << this->de << "\n";
    gzdbg << "dr: " << this->dr << "\n";
    gzdbg << "pitchMoment: " << pitchMoment << "\n";
    gzdbg << "pitchMomentDirection: " << pitchMomentDirection << "\n";
    gzdbg << "rollMoment: " << rollMoment << "\n";
    gzdbg << "rollMomentDirection: " << rollMomentDirection << "\n";
    gzdbg << "yawMoment: " << yawMoment << "\n";
    gzdbg << "yawMomentDirection: " << yawMomentDirection << "\n";
    // gzdbg << "cp momentArm: " << momentArm << "\n";
    gzdbg << "force: " << force << "\n";
    gzdbg << "torque: " << torque << "\n";
    gzdbg << "Elevator Angle: " << this->elevatorJoint->Position(0) << "\n";
  }

  // Correct for nan or inf
  force.Correct();
  // this->cp.Correct();
  torque.Correct();

  // apply forces at cg (with torques for position shift)
  if(abs(this->alpha)<0.7)
  {
    //this->link->AddForce(force);
    this->link->AddTorque(torque);
  }
  else
  {
    this->link->AddForce(ignition::math::Vector3d(0, 0, 0));
    this->link->AddTorque(ignition::math::Vector3d(0, 0, 0)); 
    gzdbg << "Alpha too large. Turning off aero forces..." << "\n";
  }
}
