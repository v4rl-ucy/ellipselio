/**
 * @file lidar_processing.h
 * @brief LiDAR point cloud processing with vendor-specific support.
 * @details Processes point clouds from multiple LiDAR vendors (Livox, Velodyne,
 * Ouster, Hesai, Gazebo), handles temporal synchronization, range binning with
 * octree indexing, and point undistortion.
 */

#ifndef LIDAR_PROCESSING_H_
#define LIDAR_PROCESSING_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common_lib.h"

/**
 * @enum LidType
 * @brief Supported LiDAR sensor types.
 */
enum class LidType {
  kLivox = 1,     ///< Livox Mid-360 or Mid-70
  kVelodyne = 2,  ///< Velodyne VLP-16, VLP-32, etc.
  kOuster = 3,    ///< Ouster OS1, OS2 (64/128 lines)
  kHesai = 4,     ///< Hesai QT/XT series
  kGazebo = 5,    ///< Gazebo simulator
};

/**
 * @struct LidarParams
 * @brief LiDAR sensor parameters and configuration.
 * @details Contains sensor-specific information needed for point extraction,
 *          calibration, and temporal synchronization.
 */
struct LidarParams {
  /// @brief Sensor type (see @ref LidType enum)
  int type;
  /// @brief Frame rate / scan rate in Hz
  int rate;
  /// @brief Number of measurement lines (0 for non-scanning sensors)
  int scan_lines;
  /// @brief Minimum measurement range in meters
  double min_range;
  /// @brief Maximum measurement range in meters
  double max_range;
  /// @brief Vertical field-of-view in degrees
  double vertical_fov;
  /// @brief ROS 2 topic name for point cloud subscription
  std::string topic;
};

/**
 * @class LidarProcess
 * @brief Processes incoming LiDAR point clouds for odometry estimation.
 * @details Implements vendor-specific point extraction, range-based binning
 * with octree spatial indexing, temporal tracking, and integration with the
 * tensor voting algorithm for ellipsoid fitting.
 */
class LidarProcess {
 public:
  /**
   * @brief Construct a LiDAR processor with specified parameters.
   * @param params LiDAR sensor configuration (vendor, rate, range, etc.)
   * @param map_resolution Target resolution for map representation in meters
   * @param node Shared pointer to ROS 2 node for subscriptions and logging
   */
  ~LidarProcess();
  LidarProcess(LidarParams params, float map_resolution,
               rclcpp::Node::SharedPtr node);

  /**
   * @brief Clear all buffered point clouds.
   */
  void ClearPointCloud();

  /**
   * @brief Retrieve processed point cloud and temporal/spatial metadata.
   * @param pc Output: processed point cloud in EllipseLio format
   * @param start_time Output: timestamp of earliest point in cloud
   * @param end_time Output: timestamp of latest point in cloud
   * @param bin_pcs_sizes Output: array of point counts per range bin
   * @param start_bin Output: index of first non-empty bin
   * @param mean_bin Output: index of bin with median point count
   * @return True if point cloud successfully extracted; false if empty/stale
   * @details Returns false if no new scan data is available or scan age
   *          exceeds timeout. Point cloud is organized into range bins
   *          with octree indexing for efficient spatial queries.
   */
  bool GetPointCloud(EllipseLioPointCloudPtr pc, rclcpp::Time* start_time,
                     rclcpp::Time* end_time, Eigen::ArrayXi* bin_pcs_sizes,
                     int* start_bin, int* mean_bin);

  /// @brief Number of range bins used for organization
  int num_bins_;
  /// @brief Flag indicating whether LiDAR data has been received
  bool lidar_has_data_;

  /// @brief Resolution of range bins in meters
  float scan_res_;
  /// @brief Minimum allowed bin resolution
  float min_scan_res_;
  /// @brief Maximum search radius for neighbor queries in meters
  float max_search_rad_;
  /// @brief Maximum octree resolution for finest subdivision
  float max_octree_res_;

  /// @brief Frame counter (incremented per received scan)
  std::atomic<int> lidar_counter_;
  /// @brief Temporal offset applied to timestamps (for sync adjustment)
  rclcpp::Duration lidar_time_offset_;
  /// @brief Timestamp of first point in current scan
  rclcpp::Time lidar_start_time_;
  /// @brief Timestamp of last point in current scan
  rclcpp::Time lidar_end_time_;

  /// @brief Array of point counts per bin
  std::vector<int> bucket_sizes_;
  /// @brief Minimum neighbor count per bin
  std::vector<int> cnt_neighbours_;
  /// @brief Minimum neighbors for octree operations per bin
  std::vector<int> min_neighbours_;
  /// @brief Maximum neighbors for octree operations per bin
  std::vector<int> max_neighbours_;
  /// @brief Search radius per bin
  std::vector<float> search_radii_;
  /// @brief Match radius per bin
  std::vector<float> match_radii_;
  /// @brief Scan line separation per bin
  std::vector<float> scan_line_sep_;
  /// @brief Octree resolution per bin
  std::vector<float> octree_resolutions_;

 private:
  /**
   * @brief ROS 2 callback for point cloud reception.
   * @param msg_in Incoming PointCloud2 message
   * @details Dispatches to vendor-specific template handler based on sensor
   * type.
   */
  void LidarCallback(const sensor_msgs::msg::PointCloud2::UniquePtr msg_in);

  /**
   * @brief Process received point cloud message.
   * @param msg PointCloud2 message to process
   * @details Converts to vendor-specific point type and calls template handler.
   */
  void Process(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Set start/end timestamps for a range bin.
   * @param bin_idx Index of range bin to update
   */
  void SetMinMaxTime(int bin_idx);

  /**
   * @brief Clear all range bins and octrees.
   */
  void ClearBins();

  /**
   * @brief Extract intensity and timestamp for Livox point.
   * @param in_pt0 First point in scan (for undistortion reference)
   * @param in_pt Current point to process
   * @param out_pt Output: EllipseLio point with extracted data
   * @param point_time In/out: timestamp to update with this point
   * @details Vendor-specific overload for Livox sensors.
   */
  void SetPoint(const LivoxPoint& in_pt0, const LivoxPoint& in_pt,
                EllipseLioPoint* out_pt, rclcpp::Time* point_time);

  /// @brief SetPoint overload for Velodyne sensors
  void SetPoint(const VelodynePoint& in_pt0, const VelodynePoint& in_pt,
                EllipseLioPoint* out_pt, rclcpp::Time* point_time);

  /// @brief SetPoint overload for Ouster sensors
  void SetPoint(const OusterPoint& in_pt0, const OusterPoint& in_pt,
                EllipseLioPoint* out_pt, rclcpp::Time* point_time);

  /// @brief SetPoint overload for Hesai sensors
  void SetPoint(const HesaiPoint& in_pt0, const HesaiPoint& in_pt,
                EllipseLioPoint* out_pt, rclcpp::Time* point_time);

  /// @brief SetPoint overload for Gazebo simulator
  void SetPoint(const GazeboPoint& in_pt0, const GazeboPoint& in_pt,
                EllipseLioPoint* out_pt, rclcpp::Time* point_time);

  /**
   * @brief Convert vendor-specific point to EllipseLio format.
   * @tparam InPtType PCL point type from input cloud
   * @param in_pc Input point cloud with vendor-specific points
   * @param pt_idx Index of point to convert
   * @param point_time In/out: timestamp to update
   * @details Template implementation allows reuse for different vendors.
   */
  template <typename InPtType>
  void ConvertPoint(pcl::PointCloud<InPtType>& in_pc, int pt_idx,
                    rclcpp::Time& point_time);

  /**
   * @brief Generic handler for point cloud message of vendor type.
   * @tparam InPtType PCL point type from input cloud
   * @param msg PointCloud2 message to process
   * @details Converts ROS message to PCL cloud and processes all points.
   */
  template <typename InPtType>
  void PointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /// @brief Shared pointer to ROS 2 node
  rclcpp::Node::SharedPtr node_;
  /// @brief Callback group for point cloud processing
  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
  /// @brief Subscriber for point cloud topic
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;

  /// @brief Sizes of points in each range bin
  Eigen::ArrayXi bin_pcs_sizes_;
  /// @brief Intermediate bin size tracking
  Eigen::ArrayXi bin_pcs_i_;

  /// @brief Point indices per range bin
  std::vector<std::vector<int>> bin_idxs_;
  /// @brief Atomic point count per range bin
  std::vector<std::atomic<int>> bin_sizes_;
  /// @brief Octree spatial index per range bin
  std::vector<iOctree::Octree> bin_octrees_;
  /// @brief Point cloud per range bin
  std::vector<EllipseLioPointCloud> bin_pcs_;

  /// @brief Minimum timestamp per range bin
  std::vector<rclcpp::Time> bin_min_times_;
  /// @brief Maximum timestamp per range bin
  std::vector<rclcpp::Time> bin_max_times_;

  /// @brief Temporary processing point cloud
  EllipseLioPointCloudPtr process_pc_;
  /// @brief Final output point cloud
  EllipseLioPointCloudPtr ellipselio_pc_;

  /// @brief Index of first range bin with points
  int start_bin_;
  /// @brief Index of range bin with median point count
  int mean_bin_;
  /// @brief Timestamp of most recently processed point
  double last_lidar_time_;
  /// @brief Mutex for thread-safe point cloud buffer access
  std::mutex lidar_mutex_;
  /// @brief Sensor parameters
  LidarParams params_;
};

#endif  // LIDAR_PROCESSING_H_
