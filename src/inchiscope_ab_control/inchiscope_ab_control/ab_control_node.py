import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from inchiscope_msgs.action import SetAbDiameter
from inchiscope_msgs.msg import PressureStateArray, ValveCommand

AB_IDS = ('proximal', 'central', 'distal')


class AbControlNode(Node):
    """Hosts the SetAbDiameter action server for proximal/central/distal.

    Phase 2 scope (inchiscope_ros2_architecture.md section 5): map target
    diameter -> pressure via a placeholder curve (swap in the real fit from
    the paper's Fig. 4a-I data once derived), apply a hard pressure ceiling
    per AB from the anchoring-force characterisation (paper section III-A),
    reject/clamp requests above it and report that in the action result, and
    publish duty-cycle commands to /firmware/valve_cmd.

    This node currently only wires up the ROS2 interfaces (action server,
    command publisher, state subscription, per-AB pressure ceiling params);
    the diameter->pressure mapping and clamp are not implemented yet, so
    goals are aborted with an explanatory message.
    """

    def __init__(self):
        super().__init__('ab_control_node')

        for ab_id in AB_IDS:
            self.declare_parameter(f'pressure_ceiling_kpa.{ab_id}', 100.0)

        self._valve_cmd_pub = self.create_publisher(
            ValveCommand, '/firmware/valve_cmd', 10
        )
        self.create_subscription(
            PressureStateArray, '/firmware/pressure_state', self._on_pressure_state, 10
        )
        self._latest_pressure_state = {}

        self._action_server = ActionServer(
            self, SetAbDiameter, 'set_ab_diameter', self._execute_set_ab_diameter
        )

    def _on_pressure_state(self, msg: PressureStateArray):
        for pressure in msg.pressures:
            self._latest_pressure_state[pressure.ab_id] = pressure

    def _pressure_ceiling_kpa(self, ab_id: str) -> float:
        return float(self.get_parameter(f'pressure_ceiling_kpa.{ab_id}').value)

    async def _execute_set_ab_diameter(self, goal_handle):
        result = SetAbDiameter.Result()
        if goal_handle.request.ab_id not in AB_IDS:
            goal_handle.abort()
            result.success = False
            result.message = f'unknown ab_id: {goal_handle.request.ab_id}'
            return result

        self.get_logger().warn(
            'SetAbDiameter diameter->pressure mapping not yet implemented '
            '(Phase 2) -- aborting goal'
        )
        goal_handle.abort()
        result.success = False
        result.message = 'diameter->pressure curve not yet implemented'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = AbControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
