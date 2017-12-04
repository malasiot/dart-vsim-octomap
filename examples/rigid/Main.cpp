/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include "dart/dart.hpp"
#include "MyWindow.hpp"

#include <fcl/config.h>

using namespace dart;
using namespace dart::dynamics ;
using namespace std ;
using namespace Eigen ;

void createRandomBox(dart::simulation::WorldPtr world, const string &name, const Eigen::Isometry3d& _T,
                        const Eigen::Vector3d& _size,
                        double _mass)
{
  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create(name + "_skeleton");

  dynamics::ShapePtr newShape(new dynamics::BoxShape(_size));

  dynamics::BodyNode::Properties bodyProp;
  bodyProp.mName = name ;
  bodyProp.mFrictionCoeff = 100.0 ;
  bodyProp.mRestitutionCoeff = 0.00 ;

  dynamics::FreeJoint::Properties jointProp;
  jointProp.mName = name + "_joint";
  jointProp.mT_ParentBodyToJoint = _T;

  pair<FreeJoint *, BodyNode *> pair = newSkeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
        nullptr, jointProp, bodyProp);
  BodyNode *bn = pair.second ;
  ShapeNode *shapeNode = pair.second->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(newShape);
  shapeNode->getVisualAspect()->setColor(Vector3d(0.1, 0.1, 0.1));

  Inertia inertia;
  inertia.setMass(1000 * newShape->getVolume());
  inertia.setMoment(newShape->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);

  // Set the spring and damping coefficients for the degrees of freedom
  for(std::size_t i=6; i < newSkeleton->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = newSkeleton->getDof(i);
    dof->setSpringStiffness(1);
    dof->setDampingCoefficient(0.1);
  }
  world->addSkeleton(newSkeleton);
}

Isometry3d getRandTransform(double d)
{
  Isometry3d T = Isometry3d::Identity();

  const Vector3d rotation = math::randomVector<3>(-math::constantsd::pi(),
                                                   math::constantsd::pi());
  const Vector3d position = Vector3d(math::random(-0.4, 0.4),
                                     math::random( d, d + 0.1),
                                     math::random(-0.4, 0.4));

  T.translation() = position;
  T.linear() = math::expMapRot(rotation);

  return T;
}

int main(int argc, char* argv[])
{
  // load a skeleton file
  // create and initialize the world
  dart::simulation::WorldPtr myWorld
      = dart::utils::SkelParser::readWorld(DART_DATA_PATH"skel/tray.skel");
  assert(myWorld != NULL);

  for( uint i=0 ; i< 100 ; i++ ) {
      stringstream bname ;
      bname << "box_" << i ;
      createRandomBox(myWorld, bname.str(), getRandTransform(0.1), Vector3d(0.01, math::random(0.05, 0.1), 0.01), 0.1) ;
  }

  for( uint i=0 ; i< 100 ; i++ ) {
      stringstream bname ;
      bname << "box_" << i+100 ;
      createRandomBox(myWorld, bname.str(), getRandTransform(0.3), Vector3d(0.005, math::random(0.05, 0.1), 0.01), 0.1) ;
  }

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'q': spawn a random cube" << std::endl;
  std::cout << "'w': spawn a random ellipsoid" << std::endl;
  std::cout << "'e': spawn a random cylinder" << std::endl;
  std::cout << "'a': delete a spawned object at last" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Rigid Cubes Falling");
  glutMainLoop();

  return 0;
}
