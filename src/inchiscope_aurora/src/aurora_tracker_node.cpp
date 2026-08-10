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

// CombinedApi.h declares warningStrings/errorStrings as file-scope statics;
// any translation unit that includes it without using them directly (like
// this one) trips -Wunused-variable under -Wextra. That's a vendor header
// quirk, not a bug in our code, so scope the suppression to just this
// include rather than disabling the warning for the whole file.
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include "CombinedApi.h"
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
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
 * Connects to the Aurora field generator, loads the distal 6D sensor's
 * virtual SROM (a tool definition uploaded over the wire rather than read
 * from a physical connector chip, since the sensor coil has no onboard SROM
 * chip of its own), auto-detects the reference tool (its SROM is on its own
 * physical chip, so no upload is needed for it), and publishes both tools'
 * pose.
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
    declare_parameter("sensor_srom_path", "");
    declare_parameter("sensor_port_number", "02");
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
    const std::string sensor_srom = get_parameter("sensor_srom_path").as_string();

    if (sensor_srom.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "sensor_srom_path parameter must be set -- see inchiscope_aurora/README.md");
      return;
    }
    if (!fileReadable(sensor_srom)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "cannot read the configured sensor .rom file: '%s'", sensor_srom.c_str());
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

    // Both tools are physically wired into SCU ports -- PHRQ (which
    // manufactures a port handle for a tool with no physical connection at
    // all, e.g. an optical passive/wireless marker on Polaris/Vega) doesn't
    // apply here and this firmware rejects it outright. Both the reference
    // port and the sensor's port show up on their own via a port search;
    // the only difference is the sensor's port has no onboard chip, so we
    // PVWR its virtual SROM onto the discovered-but-empty handle before
    // initializing it.
    const std::string sensor_port_number = get_parameter("sensor_port_number").as_string();
    sensor_port_handle_ = findPortHandleByNumber(sensor_port_number);
    if (sensor_port_handle_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "No port handle found for sensor_port_number '%s' -- check the "
        "sensor is plugged into that SCU port, will retry",
        sensor_port_number.c_str());
      return;
    }
    capi_.loadSromToPort(sensor_srom, sensor_port_handle_);

    initializeAndEnablePorts();

    reference_port_handle_ = findReferencePortHandle();
    if (reference_port_handle_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "No reference tool detected (only the sensor port came up enabled) "
        "-- check the reference tool is connected, will retry");
      return;
    }

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

  // Finds the not-yet-initialized port handle matching a given physical SCU
  // port number (e.g. "02"). Both the reference tool and the sensor coil
  // are physically wired in, so both show up here without needing PHRQ --
  // this firmware rejects PHRQ outright regardless of its arguments (it's
  // meant for tools with no physical connection at all, e.g. Polaris/Vega
  // passive/active-wireless markers, which doesn't describe an Aurora
  // sensor). PortHandleInfo doesn't expose a distinct "physical port"
  // field, but for Aurora the port handle value returned by PHSR is itself
  // the physical port number.
  int findPortHandleByNumber(const std::string & portNumber)
  {
    const auto handles =
      capi_.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    const int wanted = capi_.stringToInt(portNumber);
    for (const auto & info : handles) {
      if (capi_.stringToInt(info.getPortHandle()) == wanted) {
        return wanted;
      }
    }
    return -1;
  }

  void initializeAndEnablePorts()
  {
    // Covers both the sensor's port (now holding the virtual SROM we just
    // PVWR'd onto it) and the reference tool's port, which shows up here
    // automatically once its on-chip SROM is read -- no PVWR needed for it.
    const auto handles =
      capi_.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (const auto & info : handles) {
      capi_.portHandleInitialize(info.getPortHandle());
      capi_.portHandleEnable(info.getPortHandle());
    }
  }

  // The reference tool is whichever enabled port isn't the sensor port we
  // just created ourselves. Only expects one such tool; warns and ignores
  // the rest if more than one shows up.
  int findReferencePortHandle()
  {
    const auto enabled =
      capi_.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
    int reference_handle = -1;
    for (const auto & info : enabled) {
      const int handle = capi_.stringToInt(info.getPortHandle());
      if (handle == sensor_port_handle_) {
        continue;
      }
      if (reference_handle >= 0) {
        RCLCPP_WARN(
          get_logger(),
          "Multiple non-sensor tools enabled; using port %d as reference, "
          "ignoring port %d",
          reference_handle, handle);
        continue;
      }
      reference_handle = handle;
    }
    return reference_handle;
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
