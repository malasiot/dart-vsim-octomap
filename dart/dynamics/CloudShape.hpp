#ifndef __CLOUD_SHAPE_HPP__
#define __CLOUD_SHAPE_HPP__

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {


/// \brief
class CloudShape : public Shape {
public:


  /// \brief Constructor.
  CloudShape(const std::vector<Eigen::Vector3f> &pts, const Eigen::Vector3f &scale = {1.0, 1.0, 1.0} ): pts_(pts), scale_(scale) {
      _updateBoundingBoxDim();
      updateVolume();
  }

  void setOctomapResolution(float res) { octomap_res_ = res ; }
  float getOctomapResolution() const { return octomap_res_ ; }

  /// \brief Destructor.
  virtual ~CloudShape() {}

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  Eigen::Matrix3d computeInertia(double mass) const override;

  const std::vector<Eigen::Vector3f> &pts() const { return pts_ ; }

  const Eigen::Vector3f &scale() const { return scale_ ; }

protected:

   // Documentation inherited.
   void updateVolume() override;
   void _updateBoundingBoxDim() ;

   std::vector<Eigen::Vector3f> pts_;
   Eigen::Vector3f scale_ ;
   float octomap_res_ ;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}}

#endif
