#ifndef DRIVER_VELODYNE_POINT_TYPES_H
#define DRIVER_VELODYNE_POINT_TYPES_H

#include <pcl/point_types.h>

#include <limits>

namespace driver {
namespace velodyne {

/** Euclidean Velodyne coordinate, including intensity and ring number.
 * vertiacal angle, timestamp*/
struct PointXYZRRIAR {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float range;                     // range = (x^2+y^2+z^2)^0.5
  float radius;                    // radisu = (x^2+y^2)^0.5
  uint8_t intensity;               // laser intensity reading
  uint16_t angle;                  // azimuth angle, 0.01 degree
  uint8_t ring;                    // laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

struct PointXYZRRIARL {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float range;                     // range = (x^2+y^2+z^2)^0.5
  float radius;                    // radisu = (x^2+y^2)^0.5
  uint8_t intensity;               // laser intensity reading
  uint16_t angle;                  // azimuth angle, 0.01 degree
  uint8_t ring;                    // laser ring number
  uint8_t label;                   // point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

// point type from lyft-dataset
struct PointXYZIR {
  PCL_ADD_POINT4D;  // quad-word XYZ
  float intensity;
  float ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

template <typename PointType>
bool IsHitFree(const PointType& point) {
  if (point.range >= std::numeric_limits<float>::max() / 10.0) {
    return true;
  }
  return false;
}

template <typename PointType>
bool IsHitEmpty(const PointType& point) {
  if (point.range >= 0.0 &&
      point.range <= std::numeric_limits<float>::epsilon() * 10.0) {
    return true;
  }
  return false;
}

template <typename PointType>
bool IsValidPoint(const PointType& point) {
  return !IsHitFree<PointType>(point) && !IsHitEmpty<PointType>(point);
}

}  // namespace velodyne
}  // namespace driver

POINT_CLOUD_REGISTER_POINT_STRUCT(
    driver::velodyne::PointXYZRRIAR,
    (float, x, x)(float, y, y)(float, z, z)(float, range, range)(
        float, radius, radius)(uint8_t, intensity,
                               intensity)(uint16_t, angle, angle)(uint8_t, ring,
                                                                  ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    driver::velodyne::PointXYZRRIARL,
    (float, x, x)(float, y, y)(float, z, z)(float, range, range)(
        float, radius, radius)(uint8_t, intensity, intensity)(
        uint16_t, angle, angle)(uint8_t, ring, ring)(uint8_t, label, label))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    driver::velodyne::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, ring, ring))

#endif  // __VELODYNE_POINTCLOUD_POINT_TYPES_H
