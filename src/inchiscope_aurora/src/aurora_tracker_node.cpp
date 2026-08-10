#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
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
 * Connects to the Aurora field generator, auto-detects both tools via PHSR,
 * uploads the distal 6D sensor's virtual SROM onto its (chipless) port --
 * a tool definition sent over the wire rather than read from a physical
 * connector chip -- and publishes both tools' pose. The reference tool
 * needs no upload since its SROM is on its own physical chip.
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
    // The reference tool sits flat on the table, so its frame is effectively
    // world/bench-fixed. This is the pose the reconstruction pipeline should
    // consume from the rosbag -- it cancels out the field generator's
    // arbitrary internal frame (and any reference-tool drift) rather than
    // requiring every downstream consumer to redo the tf lookup themselves.
    sensor_relative_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/aurora/sensor_0/pose_relative_to_reference", 10);
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

    freeStalePorts();

    // PHRQ doesn't exist in Aurora's command set at all (confirmed against
    // the official API guide's command list) -- it's Polaris/Vega-only
    // terminology for manufacturing a handle for a tool with no physical
    // connection. Aurora auto-detects and assigns handles to both tools via
    // plain PHSR, chip or no chip (API guide, PHSR usage note 1), since
    // every Aurora tool is physically wired to a numbered SCU port.
    if (!classifyPortsAndLoadSensor(sensor_srom)) {
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

  // Discovers both tools (PHSR auto-assigns handles to both, chip or not)
  // and tells them apart by trying PINIT on each: PINIT is what actually
  // reads the onboard chip, so a chipless port with nothing PVWR'd yet has
  // nothing to initialize from and fails. (PHINF can't be used for this --
  // confirmed on real hardware that both ports report identical all-zero
  // placeholder fields, Tool Type/serial number included, until PINIT has
  // actually run on them; the chip isn't read any earlier than that.)
  // Whichever port fails gets the sensor's virtual SROM uploaded (PVWR)
  // and is initialized again -- PVWR must happen before PINIT for a tool
  // with no onboard SROM (API guide, PINIT usage note 1).
  //
  // This also sidesteps a real gap in the vendor C++ wrapper: PHINF's
  // "Main Type" field (which would directly say "Reference") and its
  // "physical port location" field (reply options 0001's tool type byte
  // and 0080 respectively) are both parsed by the library internally but
  // never exposed publicly -- see PortHandleInfo.h/.cpp.
  bool classifyPortsAndLoadSensor(const std::string & sensor_srom)
  {
    const auto handles =
      capi_.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);

    sensor_port_handle_ = -1;
    reference_port_handle_ = -1;

    for (const auto & info : handles) {
      const std::string handle_str = info.getPortHandle();
      const int handle = capi_.stringToInt(handle_str);

      if (capi_.portHandleInitialize(handle_str) == 0) {
        if (reference_port_handle_ >= 0) {
          RCLCPP_WARN(
            get_logger(),
            "Multiple tools initialized from onboard chip data; using "
            "port %d as the reference, ignoring port %d",
            reference_port_handle_, handle);
          continue;
        }
        reference_port_handle_ = handle;
        continue;
      }

      // PINIT failed -- no chip and nothing PVWR'd yet, so this must be
      // our sensor. Upload its virtual SROM and initialize again.
      if (sensor_port_handle_ >= 0) {
        RCLCPP_WARN(
          get_logger(),
          "Multiple chipless ports found; using port %d as the sensor, "
          "ignoring port %d", sensor_port_handle_, handle);
        continue;
      }
      capi_.loadSromToPort(sensor_srom, handle);
      if (capi_.portHandleInitialize(handle_str) != 0) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to initialize port %d even after loading its virtual "
          "SROM -- will retry", handle);
        continue;
      }
      sensor_port_handle_ = handle;
    }

    if (sensor_port_handle_ < 0 || reference_port_handle_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Expected one chip tool (reference) and one chipless tool "
        "(sensor) on the SCU; found %s%s -- will retry",
        reference_port_handle_ < 0 ? "no reference tool " : "",
        sensor_port_handle_ < 0 ? "no chipless sensor port " : "");
      return false;
    }

    return true;
  }

  // API guide Figure 2-1, step 1: free any port handles left over from a
  // previous run (e.g. after a tool was unplugged) before doing anything
  // else.
  void freeStalePorts()
  {
    const auto to_free =
      capi_.portHandleSearchRequest(PortHandleSearchRequestOption::PortsToFree);
    for (const auto & info : to_free) {
      capi_.portHandleFree(info.getPortHandle());
    }
  }

  // API guide Figure 2-1, steps 2-3: PINIT and PENA are each their own
  // loop-until-empty pass, not interleaved -- initializing one port handle
  // can cause a new one to appear (the guide's example: the second channel
  // of a dual-5DOF tool), so a single combined pass could miss it. Both
  // tools are already PINIT'd by classifyPortsAndLoadSensor() by the time
  // this runs, so in the normal case the first loop below finds nothing
  // and only PENA remains -- it stays here (rather than folded into
  // classifyPortsAndLoadSensor) to still catch a split-port dual-5DOF
  // tool's second channel, which only shows up after its sibling is
  // initialized.
  void initializeAndEnablePorts()
  {
    constexpr int kMaxPasses = 10;

    for (int pass = 0; pass < kMaxPasses; ++pass) {
      const auto not_init =
        capi_.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
      if (not_init.empty()) {
        break;
      }
      for (const auto & info : not_init) {
        capi_.portHandleInitialize(info.getPortHandle());
      }
    }

    for (int pass = 0; pass < kMaxPasses; ++pass) {
      const auto not_enabled =
        capi_.portHandleSearchRequest(PortHandleSearchRequestOption::NotEnabled);
      if (not_enabled.empty()) {
        break;
      }
      for (const auto & info : not_enabled) {
        capi_.portHandleEnable(info.getPortHandle());
      }
    }
  }

  void pollAndPublish()
  {
    // Aurora doesn't support the Vega/Polaris-only BX2 command, so this
    // uses the classic binary BX command instead.
    const std::vector<ToolData> tools = capi_.getTrackingDataBX();
    const rclcpp::Time stamp = now();

    const ToolData * reference_tool = nullptr;
    const ToolData * sensor_tool = nullptr;

    for (const auto & tool : tools) {
      if (tool.transform.toolHandle == static_cast<uint16_t>(reference_port_handle_)) {
        reference_tool = &tool;
        publishToolPose(tool, reference_frame_id_, *reference_pose_pub_, stamp);
      } else if (tool.transform.toolHandle == static_cast<uint16_t>(sensor_port_handle_)) {
        sensor_tool = &tool;
        publishToolPose(tool, sensor_frame_id_, *sensor_pose_pub_, stamp);
      }
    }

    if (reference_tool && sensor_tool &&
      !reference_tool->transform.isMissing() && !sensor_tool->transform.isMissing())
    {
      publishSensorRelativeToReference(*reference_tool, *sensor_tool, stamp);
    }
  }

  static tf2::Transform toTf2Transform(const Transform & t)
  {
    return tf2::Transform(
      tf2::Quaternion(t.qx, t.qy, t.qz, t.q0),
      tf2::Vector3(t.tx * kMmToM, t.ty * kMmToM, t.tz * kMmToM));
  }

  // The reference tool sits flat on the table -- treat it as the world
  // frame and express the sensor's pose relative to it, cancelling out the
  // field generator's arbitrary internal frame (and any reference-tool
  // drift): T_reference_to_sensor = T_field_to_reference^-1 * T_field_to_sensor.
  // Composed directly from this poll's two transforms (both already in the
  // shared aurora_field frame) rather than via a tf2 buffer/listener
  // round-trip, since both are already on hand here.
  void publishSensorRelativeToReference(
    const ToolData & reference_tool, const ToolData & sensor_tool, const rclcpp::Time & stamp)
  {
    const tf2::Transform field_to_reference = toTf2Transform(reference_tool.transform);
    const tf2::Transform field_to_sensor = toTf2Transform(sensor_tool.transform);
    const tf2::Transform reference_to_sensor = field_to_reference.inverse() * field_to_sensor;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = reference_frame_id_;

    const tf2::Vector3 & origin = reference_to_sensor.getOrigin();
    pose.pose.position.x = origin.x();
    pose.pose.position.y = origin.y();
    pose.pose.position.z = origin.z();

    const tf2::Quaternion rotation = reference_to_sensor.getRotation();
    pose.pose.orientation.x = rotation.x();
    pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z();
    pose.pose.orientation.w = rotation.w();

    sensor_relative_pose_pub_->publish(pose);
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sensor_relative_pose_pub_;
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
