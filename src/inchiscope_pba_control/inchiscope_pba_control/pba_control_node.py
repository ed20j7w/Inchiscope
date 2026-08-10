import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from inchiscope_msgs.action import MovePba
from inchiscope_msgs.msg import PistonCommand, PistonStateArray

PBA_IDS = ('proximal', 'distal')


class PbaControlNode(Node):
    """Hosts the MovePba action server for both PBAs.

    Phase 2 scope (inchiscope_ros2_architecture.md section 5): port the
    constant-curvature model + cubic piston<->bellows inversion from the
    original firmware main.cpp (unchanged math, now running on the PC),
    convert (arc_length, bend_angle, bend_plane) -> 3 bellows lengths -> 3
    piston targets, publish those to /firmware/piston_cmd, and watch
    /firmware/piston_state for at_target to drive action feedback/result.
    Also exposes a lighter joystick-style topic for direct teleop of the
    distal PBA, bypassing the action interface.

    This node currently only wires up the ROS2 interfaces (action server,
    command publisher, state subscription); the kinematics have not been
    ported yet, so goals are aborted with an explanatory message.
    """

    def __init__(self):
        super().__init__('pba_control_node')

        self._piston_cmd_pub = self.create_publisher(
            PistonCommand, '/firmware/piston_cmd', 10
        )
        self.create_subscription(
            PistonStateArray, '/firmware/piston_state', self._on_piston_state, 10
        )
        self._latest_piston_state = {}

        self._action_server = ActionServer(
            self, MovePba, 'move_pba', self._execute_move_pba
        )

    def _on_piston_state(self, msg: PistonStateArray):
        for piston in msg.pistons:
            self._latest_piston_state[piston.id] = piston

    async def _execute_move_pba(self, goal_handle):
        result = MovePba.Result()
        if goal_handle.request.pba_id not in PBA_IDS:
            goal_handle.abort()
            result.success = False
            result.message = f'unknown pba_id: {goal_handle.request.pba_id}'
            return result

        self.get_logger().warn(
            'MovePba kinematics not yet ported from firmware (Phase 2) -- aborting goal'
        )
        goal_handle.abort()
        result.success = False
        result.message = 'constant-curvature model not yet implemented'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PbaControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
