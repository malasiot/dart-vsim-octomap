#include "gripper.hpp"
#include <dart/utils/urdf/DartLoader.hpp>

using namespace dart ;
using namespace dart::dynamics ;
using namespace std ;


bool Gripper::init(const dart::simulation::WorldPtr &world, const Eigen::Isometry3d &trans) {

    using dart::common::Uri;
    using dart::utils::DartLoader;


    DartLoader loader;

    loader.addPackageDirectory("rethink_ee_description", "/home/malasiot/source/radioroso_ws/src/baxter_common/rethink_ee_description/");

    gripper_ = loader.parseSkeleton("/home/malasiot/source/radioroso_ws/right_end_effector.urdf");

    if ( !gripper_ ) return false ;

    left_finger_ = gripper_->getDof("r_gripper_l_finger_joint");
    right_finger_ = gripper_->getDof("r_gripper_r_finger_joint");

    left_finger_->getJoint()->setActuatorType(Joint::SERVO);
    left_finger_->getJoint()->setForceUpperLimit(0, 0.005);
    left_finger_->getJoint()->setPositionLimitEnforced(true);

    right_finger_->getJoint()->setActuatorType(Joint::SERVO);
    right_finger_->getJoint()->setForceUpperLimit(0, 0.005);
    right_finger_->getJoint()->setPositionLimitEnforced(true);


    base_joint_ = dynamic_cast<WeldJoint *>(gripper_->getJoint("right_gripper_base")) ;

    gripper_->setSelfCollisionCheck(false);

    base_joint_->setTransformFromParentBodyNode(trans);

    left_finger_->setPosition( pos_left_finger_ = left_finger_->getPositionUpperLimit()) ;
    right_finger_->setPosition(pos_right_finger_ = right_finger_->getPositionUpperLimit()) ;

    auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();
    auto collisionGroup = world->getConstraintSolver()->getCollisionGroup();
    auto newGroup = collisionEngine->createCollisionGroup(gripper_.get());

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
   // bool collision = collisionGroup->collide(newGroup.get(), option, &result);

    world->addSkeleton(gripper_);

    return false ;
    //return !collision ;

}

void Gripper::open() {

}

void Gripper::close() {
    const double speed = -0.1 ;
    left_finger_->setCommand(speed) ;
    right_finger_->setCommand(speed) ;
}

bool Gripper::stabilized() {
    double lpos = left_finger_->getPosition() ;
    double rpos = right_finger_->getPosition() ;

    double dleft = lpos - pos_left_finger_ ;
    double dright = rpos - pos_right_finger_ ;

    double total = fabs(dleft) + fabs(dright) ;

    pos_left_finger_ = lpos ;
    pos_right_finger_ = rpos ;

    return total < 1.0e-6 ;
}

