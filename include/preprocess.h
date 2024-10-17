#pragma once

#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <common_pcl.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

enum LID_TYPE {
  AVIA = 1,
  VELO16 = 2,
  OUST64 = 3,
  MID360 = 4,
  XT32 = 5,
  L515 = 6,
  VELO32 = 7
};
enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };
enum Feature {
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};
enum Surround { Prev, Next };
enum E_jump { Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind };

struct orgtype {
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype() {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity,intensity)
                                  (float, time, time)
                                  (uint16_t, ring, ring)
)

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
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint32_t, range, range)
)

namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float    intensity;
  double   timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp,timestamp)
                                  (uint16_t, ring, ring) 
)

namespace livox_ros 
{
struct EIGEN_ALIGN16 LivoxPointXYZRTLT {
  float x;              /**< X axis, Unit:m */
  float y;              /**< Y axis, Unit:m */
  float z;              /**< Z axis, Unit:m */
  uint8_t reflectivity; /**< Reflectivity   */
  uint8_t tag;          /**< Livox point tag   */
  uint8_t line;         /**< Laser line id     */
  uint32_t offset_time; /**< Time offset, Unit:ns */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace livox_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::LivoxPointXYZRTLT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, reflectivity, reflectivity)
                                  (uint8_t, tag, tag)
                                  (uint8_t, line, line)
                                  (uint32_t, offset_time, offset_time)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, FastLioPointCloud::Ptr &pcl_out);
  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, FastLioPointCloud::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::msg::PointCloud2::UniquePtr pointcloud;
  FastLioPointCloud    pl_full, pl_corn, pl_surf;
  FastLioPointCloud    pl_buff[ 128 ]; // maximum 128 line lidar
  vector< orgtype > typess[ 128 ];  // maximum 128 line lidar
  int               lidar_type, point_filter_num, SCAN_RATE, N_SCANS, MAX_LINE_NUM, time_unit;
  double            blind, blind_sqr;
  double            time_unit_scale;
  bool              feature_enabled, given_offset_time, calib_laser;

private:
  void avia_handler( const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg );
  void velodyne_handler( const sensor_msgs::msg::PointCloud2::UniquePtr &msg );
  void oust64_handler( const sensor_msgs::msg::PointCloud2::UniquePtr &msg );
  void mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void xt32_handler( const sensor_msgs::msg::PointCloud2::UniquePtr &msg );
  void l515_handler( const sensor_msgs::msg::PointCloud2::UniquePtr &msg );
  void velodyne32_handler( const sensor_msgs::msg::PointCloud2::UniquePtr &msg );
  void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);
  void give_feature( FastLioPointCloud &pl, vector< orgtype > &types );
  void pub_func( FastLioPointCloud &pl, const rclcpp::Time &ct );
  int  plane_judge( const FastLioPointCloud &pl, vector< orgtype > &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct );
  bool small_plane( const FastLioPointCloud &pl, vector< orgtype > &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct );
  bool edge_jump_judge( const FastLioPointCloud &pl, vector< orgtype > &types, uint i, Surround nor_dir );

  int    group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};

#endif  // PREPROCESS_H
