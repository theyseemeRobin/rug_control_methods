#!/usr/bin/env python
import random

import rospy
from franka_msgs.msg import FrankaState
import tf.transformations
import numpy as np
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl

from controller_interface import ControllerInterface

class InteractivePosePlanner:
    """
    Very basic time based pose planner, as implemented by the cartesina_pose_example_controller
    """

    def __init__(self):
        self.marker_pose = PoseStamped()
        self.position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

        self.int_marker = InteractiveMarker()
        self.setup_marker()

        self.server = InteractiveMarkerServer('equilibrium_pose_marker')
        self.server.insert(self.int_marker, self.update_marker)
        self.server.applyChanges()

    def get_pose_stamped(self):
        return self.marker_pose

    def wait_for_initial_pose(self):
        msg = rospy.wait_for_message(
            "franka_state_controller/franka_states",
            FrankaState
            )  # type: FrankaState

        initial_quaternion = \
            tf.transformations.quaternion_from_matrix(
                np.transpose(
                    np.reshape(
                        msg.O_T_EE,
                        (4, 4)
                        )
                    )
            )
        initial_quaternion = initial_quaternion / \
                             np.linalg.norm(initial_quaternion)
        self.marker_pose.pose.orientation.x = initial_quaternion[0]
        self.marker_pose.pose.orientation.y = initial_quaternion[1]
        self.marker_pose.pose.orientation.z = initial_quaternion[2]
        self.marker_pose.pose.orientation.w = initial_quaternion[3]
        self.marker_pose.pose.position.x = msg.O_T_EE[12]
        self.marker_pose.pose.position.y = msg.O_T_EE[13]
        self.marker_pose.pose.position.z = msg.O_T_EE[14]

    def setup_marker(self):
        self.wait_for_initial_pose()
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = rospy.get_param("~link_name", "panda_link0")
        self.int_marker.scale = 0.3
        self.int_marker.name = "equilibrium_pose"
        self.int_marker.description = ("Equilibrium Pose\nBE CAREFUL! "
                                       "If you move the \nequilibrium "
                                       "pose the robot will follow it\n"
                                       "so be aware of potential collisions")
        self.int_marker.pose = self.marker_pose.pose
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(control)

    def update_marker(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.marker_pose.pose.position.x = max(
                [min(
                    [feedback.pose.position.x,
                     self.position_limits[0][1]]
                    ),
                 self.position_limits[0][0]]
                )
            self.marker_pose.pose.position.y = max(
                [min(
                    [feedback.pose.position.y,
                     self.position_limits[1][1]]
                    ),
                 self.position_limits[1][0]]
                )
            self.marker_pose.pose.position.z = max(
                [min(
                    [feedback.pose.position.z,
                     self.position_limits[2][1]]
                    ),
                 self.position_limits[2][0]]
                )
            self.marker_pose.pose.orientation = feedback.pose.orientation
        self.server.applyChanges()


def callback(controller_interface, planner, timer_event):
    """
    Callback function that is called periodically to compute and send the sinusoidal trajectory point.
    Args:
        controller_interface (ControllerInterface): The controller interface to send commands.
        timer_event: The timer event that triggers this callback.
    """
    trajectory_point = planner.get_pose_stamped()
    controller_interface.workspace_command(trajectory_point)


if __name__ == '__main__':
    rospy.init_node('sinusoidal_trajectory_planner', anonymous=True)

    #
    controller_interface = ControllerInterface(jointspace_topic="/rug_panda/joint_target", workspace_topic="/rug_panda/workspace_target")
    planner = InteractivePosePlanner()

    timer = rospy.Timer(
        rospy.Duration(0.1),
        lambda timer_event: callback(controller_interface, planner, timer_event)
    )
    rospy.spin()
    rospy.loginfo("Interactive Trajectory Planner Initialized")
