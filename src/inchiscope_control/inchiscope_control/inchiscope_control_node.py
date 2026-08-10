import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from inchiscope_msgs.action import MovePba, SetAbDiameter, TransitionMode
from inchiscope_msgs.msg import InchiscopeState

VALID_MODES = (
    'idle',
    'calibrating',
    'full_extension',
    'front_steering',
    'anchored_observation',
)


class InchiscopeControlNode(Node):
    """State machine hosting the TransitionMode action server.

    Phase 3 scope (inchiscope_ros2_architecture.md section 5): on a mode
    transition, sequence calls to the SetAbDiameter and MovePba action
    clients (e.g. the inching cycle described in the paper's Fig. 3b), and
    keep /inchiscope/state updated throughout so any subscriber (operator
    UI, logger) can observe progress without blocking on the action.

    This node currently wires up the action server/clients and publishes a
    static idle state; the transition sequencing itself is not implemented
    yet, so goals are aborted with an explanatory message.
    """

    def __init__(self):
        super().__init__('inchiscope_control_node')

        self._mode = 'idle'
        self._phase = 'idle'
        self._anchored = False

        self._state_pub = self.create_publisher(InchiscopeState, '/inchiscope/state', 10)
        self._state_timer = self.create_timer(0.2, self._publish_state)

        self._set_ab_diameter_client = ActionClient(self, SetAbDiameter, 'set_ab_diameter')
        self._move_pba_client = ActionClient(self, MovePba, 'move_pba')

        self._action_server = ActionServer(
            self, TransitionMode, 'transition_mode', self._execute_transition_mode
        )

    def _publish_state(self):
        msg = InchiscopeState()
        msg.mode = self._mode
        msg.phase = self._phase
        msg.anchored = self._anchored
        self._state_pub.publish(msg)

    async def _execute_transition_mode(self, goal_handle):
        target_mode = goal_handle.request.target_mode
        result = TransitionMode.Result()

        if target_mode not in VALID_MODES:
            goal_handle.abort()
            result.success = False
            result.message = f'unknown target_mode: {target_mode}'
            return result

        self.get_logger().warn(
            f'Transition sequencing to "{target_mode}" not yet implemented '
            '(Phase 3) -- aborting goal'
        )
        goal_handle.abort()
        result.success = False
        result.message = 'mode transition sequencing not yet implemented'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = InchiscopeControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
