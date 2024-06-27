#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#define PCL_NO_PRECOMPILE
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h> // removeNaNFromPointCloud function 
#include <pcl/filters/extract_indices.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_lib.h"

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))
// clang-format on

struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  // PCL_ADD_INTENSITY;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint16_t, ambient, ambient)
                                  (std::uint32_t, range, range)
                                  )
// clang-format on

namespace faster_lio {

enum class LidarType { AVIA = 1, VELO32, OUST64 , RoboSense };

/**
 * point cloud preprocess
 * just unify the point format from livox/velodyne to PCL
 */
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// processors
    void Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudType::Ptr &pcl_out);
    void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out);
    void Set(LidarType lid_type, double bld, int pfilt_num);

    // accessors
    double &Blind() { return blind_; }
    double &Set_rangeMinX() { return rangeMinX_; }
    double &Set_rangeMaxX() { return rangeMaxX_; }
    double &Set_rangeMinY() { return rangeMinY_; }
    double &Set_rangeMaxY() { return rangeMaxY_; }
    double &Set_rangeMinZ() { return rangeMinZ_; }
    double &Set_rangeMaxZ() { return rangeMaxZ_; }
    
    int &NumScans() { return num_scans_; }
    int &PointFilterNum() { return point_filter_num_; }
    bool &FeatureEnabled() { return feature_enabled_; }
    float &TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }

   private:
    void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    bool feature_enabled_ = false;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    double rangeMinX_ = -200.0 , rangeMaxX_ = 200.0,rangeMinY_ = -200.0 , rangeMaxY_ = 200.0,rangeMinZ_ = -200.0 , rangeMaxZ_ = 200.0;
    float time_scale_ = 1e-3;
    bool given_offset_time_ = false;
};
}  // namespace faster_lio

#endif
