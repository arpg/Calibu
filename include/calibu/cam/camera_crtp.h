/*
  This file is part of the Calibu Project.
  https://github.com/arpg/Calibu

  Copyright (C) 2013 George Washington University,
  Copyright (C) 2015 University of Colorado,
  Steven Lovegrove,
  Nima Keivan,
  Christoffer Heckman,
  Gabe Sibley

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/
#pragma once
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <vector>

namespace calibu {

/*
  CameraInterface is the top-level mostly pure-virtual interface
  class all cameras must honor.
*/

template<typename Scalar = double>
class CameraInterface {
protected:
  typedef Eigen::Matrix<Scalar, 2, 1> Vec2t;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3t;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecXt;
  typedef Sophus::SE3Group<Scalar> SE3t;

public:
  CameraInterface() {}

  CameraInterface(const CameraInterface<Scalar>& other) :
          image_size_(other.image_size_), params_(other.params_) {}

  virtual ~CameraInterface() {}

  /** Change camera model image size. */
  virtual void Scale( const Scalar& s ) = 0;

  virtual void PrintInfo() const = 0;

  virtual Eigen::Matrix<Scalar,3,3> K() const = 0;

  virtual void SetType() const = 0;

  /** Unproject an image location into world coordinates. */
  virtual Vec3t Unproject(const Vec2t& pix) const = 0;

  /** Project a world point into an image location. */
  virtual Vec2t Project(const Vec3t& ray) const = 0;

  /** Derivative of the Project along a ray */
  virtual Eigen::Matrix<Scalar, 2, 3>
  dProject_dray(const Vec3t& ray) const = 0;

  virtual Eigen::Matrix<Scalar, 2, Eigen::Dynamic>
  dProject_dparams(const Vec3t& ray) const = 0;

  virtual Eigen::Matrix<Scalar, 3, Eigen::Dynamic>
  dUnproject_dparams(const Vec2t& pix) const = 0;

  /**
   * Project a point into a camera located at t_ba.
   *
   * @param t_ba Location of camera to project into.
   * @param ray Homogeneous ray
   * @param rho Inverse depth of point
   * @returns A non-homegeneous pixel location in the second camera.
   */
  Vec2t Transfer3d(const SE3t& t_ba,
                   const Vec3t& ray,
                   const Scalar rho) const {
    const Vec3t ray_dehomogenized =
                    t_ba.rotationMatrix() * ray + rho * t_ba.translation();
    return Project(ray_dehomogenized);
  }

  /**
   * The derivative of the projection from Transfer3d wrt the point
   * being transfered.
   */
  Eigen::Matrix<Scalar, 2, 4> dTransfer3d_dray(const SE3t& t_ba,
                                               const Vec3t& ray,
                                               const Scalar rho) const {
    const Eigen::Matrix<Scalar, 3, 3> rot_matrix = t_ba.rotationMatrix();
    const Vec3t ray_dehomogenized =
                    rot_matrix * ray + rho * t_ba.translation();
    const Eigen::Matrix<Scalar, 2, 3> dproject_dray =
                    dProject_dray(ray_dehomogenized);
    Eigen::Matrix<Scalar, 2, 4> dtransfer3d_dray;
    dtransfer3d_dray.template topLeftCorner<2, 3>() =
                    dproject_dray * rot_matrix;
    dtransfer3d_dray.col(3) = dproject_dray * t_ba.translation();
    return dtransfer3d_dray;
  }

  Eigen::Matrix<Scalar, 2, Eigen::Dynamic> dTransfer_dparams(
                  const SE3t& t_ba,
                  const Vec2t& pix,
                  const Scalar rho) const
  {
    const Vec3t ray = Unproject(pix);
    const Eigen::Matrix<Scalar, 3, 3> rot_matrix = t_ba.rotationMatrix();
    const Vec3t ray_dehomogenized =
                    rot_matrix * ray + rho * t_ba.translation();
    const Eigen::Matrix<Scalar, 2, 3> dproject_dray =
                    dProject_dray(ray_dehomogenized);
    const Eigen::Matrix<Scalar, 2, 3> dtransfer3d_dray =
                    dproject_dray * rot_matrix;
    const Eigen::Matrix<Scalar, 3, Eigen::Dynamic> dray_dparams =
                    dUnproject_dparams(pix);

    const Eigen::Matrix<Scalar, 2, Eigen::Dynamic> d_project_dparams =
                    dProject_dparams(ray_dehomogenized);

    return d_project_dparams + dtransfer3d_dray * dray_dparams;
  }

  /// Metadata member functions from CameraModelInterface.h (non-CRTP).
  /// Returns the camera intrinsics & parameters.
  const Eigen::VectorXd& GetParams() const {
    return params_;
  }

  // Why are there const & non-const versions?
  Eigen::VectorXd& GetParams() {
    return params_;
  }

  /// Return the number of camera parameters (4 intrinsics + params by model).
  uint32_t NumParams() const {
    return params_.rows();
  }

  /// Returns width of image captured by camera.
  unsigned int Width() const {
    return image_size_[0];
  }

  /// Returns height parameter of image captured by camera.
  unsigned int Height() const {
    return image_size_[1];
  }

  /// Returns if this CameraInterface is initialized and can be used.
  // Can we excise this method? --crh
  bool IsInitialized() const {
    return 1;
  }

  /// Get version, type, sn, idx, name, etc.
  int
  Version() const {
    return version_;
  }

  std::string
  Type() const {
    return type_;
  }

  Eigen::Matrix<Scalar,3,3>
  RDF() const {
    return rdf_;
  }

  uint64_t
  SerialNumber() const {
    return serialNo_;
  }

  int
  Index() const {
    return index_;
  }

  /// Camera name, e.g. "Left".
  std::string
  Name() const {
    return name_;
  }

  /// Set name of camera, e.g. "left".
  void
  SetName( const std::string& sName ) {
    name_ = sName;
  }

  /// Set serial number of camera, if applicable.
  void
  SetSerialNumber( const uint64_t nSerialNo ) {
    serialNo_ = nSerialNo ;
  }

  /// Set index for multi-camera rigs.
  void SetIndex( const int nIndex ) {
    index_ = nIndex ;
  }

  /// Set version number of camera.
  void SetVersion( int nVersion ) {
    version_ = nVersion ;
  }

  /// Set width and height of image.
  void SetImageDimensions( const int nWidth, const int nHeight ) {
    image_size_[0] = nWidth;
    image_size_[1] = nHeight;
  }

  /// Set "right-down-forward" convention.
  void SetRDF(const Eigen::Matrix<Scalar, 3, 3>& sRDF) {
    rdf_ = sRDF;
  }

  /// Set generic camera intrinsics and parameters.
  void SetParams( const Eigen::VectorXd params ) {
    params_ = params;
  }

  /// Set the pose of the camera (typically in the "rig" frame).
  void SetPose( const SE3t& t_rc ) {
    t_rc_ = t_rc;
  }

  /// Return camera pose (typically in the "rigs" frame).
  SE3t Pose() const {
    return t_rc_;
  }


protected:
  /// Direct input of parameters and image size to create a camera inteface.
  CameraInterface(const Eigen::VectorXd& params_in,
                  const Eigen::Vector2i& image_size)
          : params_(params_in), image_size_(image_size) {
  }

  Eigen::Vector2i image_size_;

  /// All the camera parameters (fu, fv, u0, v0, ...distortion).
  Eigen::VectorXd params_;


  /// Protected data structures for CameraInterface.
  SE3t t_rc_;
  Eigen::Matrix<Scalar, 3, 3> rdf_;
  int version_;
  int index_;
  uint64_t serialNo_;
  std::string name_;
  std::string type_;
};

/// Rig class: a collection of cameras. Note, cameras contain transformation
/// from Rig to camera inside the Camera interface.
template<typename Scalar = double>
class Rig {
 public:
  Rig() {}

  void AddCamera(std::shared_ptr<CameraInterface<Scalar>> cam) {
    cameras_.push_back(cam);
  }

  /// Method to clear rig elements.
    void Clear() {
    for (std::shared_ptr<CameraInterface<Scalar>> ptr : cameras_) {
      ptr.reset();
    }
    cameras_.clear();
  }

  ~Rig() {
    Clear();
  }

  std::vector<std::shared_ptr<CameraInterface<Scalar>>> cameras_;
};
}  // namespace calibu
