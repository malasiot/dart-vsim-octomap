#include "CloudShape.hpp"

#include "dart/config.hpp"
#include "dart/common/Console.hpp"
#include "dart/dynamics/BoxShape.hpp"

#include <iostream>
namespace dart {
namespace dynamics {

const std::string& CloudShape::getType() const
{
  return getStaticType();
}


const std::string& CloudShape::getStaticType()
{
  static const std::string type("OctmapShape");
  return type;
}

//==============================================================================
Eigen::Matrix3d CloudShape::computeInertia(double _mass) const
{
  // Use bounding box to represent the mesh
  return BoxShape::computeInertia(mBoundingBox.computeFullExtents(), _mass);
}

void CloudShape::updateVolume() {
  Eigen::Vector3d bounds = mBoundingBox.computeFullExtents();
  mVolume = bounds.x() * bounds.y() * bounds.z();
  octomap_res_ = pow(mVolume, 0.33)/10 ;
}

void CloudShape::_updateBoundingBoxDim() {

  double max_X = -std::numeric_limits<double>::infinity();
  double max_Y = -std::numeric_limits<double>::infinity();
  double max_Z = -std::numeric_limits<double>::infinity();
  double min_X = std::numeric_limits<double>::infinity();
  double min_Y = std::numeric_limits<double>::infinity();
  double min_Z = std::numeric_limits<double>::infinity();

  for (unsigned int j = 0; j < pts_.size() ; j++) {
      max_X = std::max<double>(max_X, pts_[j].x() * scale_.x()) ;
      max_Y = std::max<double>(max_Y, pts_[j].y() * scale_.y()) ;
      max_Z = std::max<double>(max_Z, pts_[j].z() * scale_.z()) ;
      min_X = std::min<double>(min_X, pts_[j].x() * scale_.x()) ;
      min_Y = std::min<double>(min_Y, pts_[j].y() * scale_.y()) ;
      min_Z = std::min<double>(min_Z, pts_[j].z() * scale_.z()) ;
  }
  mBoundingBox.setMin(Eigen::Vector3d(min_X, min_Y, min_Z));
  mBoundingBox.setMax(Eigen::Vector3d(max_X, max_Y, max_Z));
}




}
}
