#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"

namespace
{
// NDI reports transforms in millimetres; ROS convention is metres.
constexpr double kMmToM = 0.001;

bool fileReadable(const std::string & path)
{
  std::ifstream f(path);
  return f.good();
}
}  // namespace

/**
 * Connects to the Aurora field generator, loads the reference tool and the
 * distal 6D sensor (identified by its virtual SROM -- a tool definition
 * uploaded over the wire rather than read from a physical connector chip,
 * since the sensor coil has no onboard SROM chip of its own), and publishes
 * both tools' pose.
 *
 * The vendor "Combined API Sample C++" SDK this links against is not
 * vendored in this repo (proprietary, no redistribution grant) -- see
 * README.md in this package for how to obtain and place it locally, and
 * inchiscope_ros2_architecture.md section 2 for why this package is
 * ament_cmake/C++ rather than ament_python like the rest of the stack.
 */
class AuroraTrackerNode : public rclcpp::Node
{
public:
  AuroraTrackerNode()
  : Node("aurora_tracker_node")
  {
    declare_parameter("field_generator_port", "/dev/ttyUSB0");
    declare_parameter("reference_srom_path", "");
    declare_parameter("sensor_srom_path", "");
    declare_parameter("publish_rate_hz", 40.0);
    declare_parameter("reconnect_period_sec", 5.0);
    declare_parameter("field_frame_id", "aurora_field");
    declare_parameter("reference_frame_id", "aurora_reference");
    declare_parameter("sensor_frame_id", "aurora_sensor_0");

    field_frame_id_ = get_parameter("field_frame_id").as_string();
    reference_frame_id_ = get_parameter("reference_frame_id").as_string();
    sensor_frame_id_ = get_parameter("sensor_frame_id").as_string();

    reference_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/aurora/reference/pose", 10);
    sensor_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/aurora/sensor_0/pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const double reconnect_period = get_parameter("reconnect_period_sec").as_double();
    reconnect_timer_ = create_wall_timer(
      std::chrono::duration<double>(reconnect_period),
      std::bind(&AuroraTrackerNode::tryConnect, this));

    // Don't make startup wait a full reconnect_period before the first try.
    tryConnect();
  }

  ~AuroraTrackerNode() override
  {
    if (tracking_) {
      capi_.stopTracking();
    }
  }

private:
  void tryConnect()
  {
    if (tracking_) {
      return;
    }

    const std::string port = get_parameter("field_generator_port").as_string();
    const std::string reference_srom = get_parameter("reference_srom_path").as_string();
    const std::string sensor_srom = get_parameter("sensor_srom_path").as_string();

    if (reference_srom.empty() || sensor_srom.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "reference_srom_path and sensor_srom_path parameters must both be set "
        "-- see inchiscope_aurora/README.md");
      return;
    }
    if (!fileReadable(reference_srom) || !fileReadable(sensor_srom)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "cannot read a configured .rom file (reference: '%s', sensor: '%s')",
        reference_srom.c_str(), sensor_srom.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Connecting to Aurora on %s ...", port.c_str());
    if (capi_.connect(port, Protocol::TCP) != 0) {
      RCLCPP_WARN(get_logger(), "Aurora connect() failed, will retry");
      return;
    }

    if (capi_.initialize() != 0) {
      RCLCPP_ERROR(get_logger(), "Aurora initialize() failed, will retry");
      return;
    }

    reference_port_handle_ = loadTool(reference_srom);
    sensor_port_handle_ = loadTool(sensor_srom);
    if (reference_port_handle_ < 0 || sensor_port_handle_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to load one or both .rom files onto a port handle, will retry");
      return;
    }

    initializeAndEnablePorts();

    if (capi_.startTracking() != 0) {
      RCLCPP_ERROR(get_logger(), "Aurora startTracking() failed, will retry");
      return;
    }

    tracking_ = true;
    reconnect_timer_->cancel();
    RCLCPP_INFO(
      get_logger(), "Aurora tracking started (reference=port %d, sensor_0=port %d)",
      reference_port_handle_, sensor_port_handle_);

    const double rate_hz = get_parameter("publish_rate_hz").as_double();
    poll_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&AuroraTrackerNode::pollAndPublish, this));
  }

  // Requests a port handle and loads the given .rom file onto it -- this is
  // the same mechanism NDI calls "PVWR" whether the file is a physical
  // tool's factory-supplied .rom or a virtual SROM generated for a bare
  // sensor coil; the API makes no distinction once you have a file.
  int loadTool(const std::string & romPath)
  {
    const int portHandle = capi_.portHandleRequest();
    if (portHandle < 0) {
      RCLCPP_ERROR(
        get_logger(), "portHandleRequest() failed for '%s': %s",
        romPath.c_str(), CombinedApi::errorToString(portHandle).c_str());
      return portHandle;
    }
    capi_.loadSromToPort(romPath, portHandle);
    return portHandle;
  }

  void initializeAndEnablePorts()
  {
    const auto handles =
      capi_.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (const auto & info : handles) {
      capi_.portHandleInitialize(info.getPortHandle());
      capi_.portHandleEnable(info.getPortHandle());
    }
  }

  void pollAndPublish()
  {
    // Aurora doesn't support the Vega/Polaris-only BX2 command, so this
    // uses the classic binary BX command instead.
    const std::vector<ToolData> tools = capi_.getTrackingDataBX();
    const rclcpp::Time stamp = now();

    for (const auto & tool : tools) {
      if (tool.transform.toolHandle == static_cast<uint16_t>(reference_port_handle_)) {
        publishToolPose(tool, reference_frame_id_, *reference_pose_pub_, stamp);
      } else if (tool.transform.toolHandle == static_cast<uint16_t>(sensor_port_handle_)) {
        publishToolPose(tool, sensor_frame_id_, *sensor_pose_pub_, stamp);
      }
    }
  }

  void publishToolPose(
    const ToolData & tool, const std::string & frame_id,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped> & pub,
    const rclcpp::Time & stamp)
  {
    if (tool.transform.isMissing()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s is out of view / missing", frame_id.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = field_frame_id_;
    pose.pose.orientation.w = tool.transform.q0;
    pose.pose.orientation.x = tool.transform.qx;
    pose.pose.orientation.y = tool.transform.qy;
    pose.pose.orientation.z = tool.transform.qz;
    pose.pose.position.x = tool.transform.tx * kMmToM;
    pose.pose.position.y = tool.transform.ty * kMmToM;
    pose.pose.position.z = tool.transform.tz * kMmToM;
    pub.publish(pose);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = pose.header;
    tf_msg.child_frame_id = frame_id;
    tf_msg.transform.translation.x = pose.pose.position.x;
    tf_msg.transform.translation.y = pose.pose.position.y;
    tf_msg.transform.translation.z = pose.pose.position.z;
    tf_msg.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  CombinedApi capi_;
  bool tracking_ = false;
  int reference_port_handle_ = -1;
  int sensor_port_handle_ = -1;

  std::string field_frame_id_;
  std::string reference_frame_id_;
  std::string sensor_frame_id_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sensor_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AuroraTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
