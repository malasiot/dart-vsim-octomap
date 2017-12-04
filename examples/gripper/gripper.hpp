#ifndef __GRIPPER_HPP__
#define __GRIPPER_HPP__

#include <string>
#include <dart/dart.hpp>

class Gripper {
public:
    Gripper() = default;

    bool init(const dart::simulation::WorldPtr &world, const Eigen::Isometry3d &trans) ;

    void open() ;
    void close() ;


    bool stabilized() ;

private:

    dart::dynamics::SkeletonPtr gripper_ ;

    dart::dynamics::DegreeOfFreedom *left_finger_ = nullptr ;
    dart::dynamics::DegreeOfFreedom *right_finger_ = nullptr ;
    dart::dynamics::WeldJoint *base_joint_ = nullptr ;
    double pos_left_finger_, pos_right_finger_ ;

};


#endif
