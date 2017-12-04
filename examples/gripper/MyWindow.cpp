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

#include "MyWindow.hpp"

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace dart::dynamics;

//==============================================================================
MyWindow::MyWindow()
  : SimWindow()
{
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{


  mWorld->step(false);

}

//==============================================================================
void MyWindow::drawWorld() const
{
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawWorld();
}

//==============================================================================
Isometry3d getRandomTransform()
{
  Isometry3d T = Isometry3d::Identity();

  const Vector3d rotation = math::randomVector<3>(-math::constantsd::pi(),
                                                   math::constantsd::pi());
  const Vector3d position = Vector3d(math::random(-1.0, 1.0),
                                     math::random( 0.5, 1.0),
                                     math::random(-1.0, 1.0));

  T.translation() = position;
  T.linear() = math::expMapRot(rotation);

  return T;
}

//==============================================================================
void MyWindow::keyboard(unsigned char key, int x, int y)
{
  switch (key)
  {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating)
        mPlay = false;
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay)
        mSimulating = false;
      break;
    case '[':  // decrease
  //    gripper_->setFingerJointPosition(gripper_->getFingerJointPosition()-0.05);


      glutPostRedisplay();
      break;
    case ']':  // increase

      //gripper_->setFingerJointPosition(gripper_->getFingerJointPosition()+0.05);
      glutPostRedisplay();
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case 'w':  // Spawn an ellipsoid
    case 'W':
    {
      gt_ += 0.01 ;
      Eigen::Isometry3d gripper_trans ;
      gripper_trans.setIdentity() ;
      gripper_trans.translate(Vector3d(0, gt_, 0)) ;
      gripper_trans.rotate(AngleAxisd(M_PI/2, Vector3d::UnitX())) ;

      break ;
    }
    case 'e':
    case 'E':
    {
      gt_ -= 0.01 ;
      Eigen::Isometry3d gripper_trans ;
      gripper_trans.setIdentity() ;
      gripper_trans.translate(Vector3d(0, gt_, 0)) ;
      gripper_trans.rotate(AngleAxisd(M_PI/2, Vector3d::UnitX())) ;




      break;
    }
    case 'a':    // Remove the skeleton added at last
    case 'A':
    {
      remove_ground_ = true ;


        break ;
    }
    default:
      Win3D::keyboard(key, x, y);
  }
  glutPostRedisplay();
}


