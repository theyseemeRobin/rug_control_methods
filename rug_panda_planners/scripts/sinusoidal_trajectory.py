#!/usr/bin/env python
import rospy
from franka_msgs.msg import FrankaState
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint

from controller_interface import ControllerInterface


class SinusoidalTrajectory:
    def __init__(self, initial_joint_angles, amplitude=0.1, frequency=1.0):
        """
        Initializes the SinusoidalTrajectory with initial joint angles, amplitude, and frequency.
        Args:
            initial_joint_angles (list): Initial joint angles of the robot.
            amplitude (float): Amplitude of the sinusoidal trajectory.
            frequency (float): Frequency of the sinusoidal trajectory.
        """
        self.initial_joint_angles = initial_joint_angles
        self.amplitude = amplitude
        self.frequency = frequency
        self.start_time = None

    def get_trajectory_point(self):
        if self.start_time is None:
            self.start_time = rospy.get_time()
        time_since_start = rospy.get_time() - self.start_time
        positions = [0.0] * len(self.initial_joint_angles)
        trajectory_point = JointTrajectoryPoint(positions=positions)
        for i, angle in enumerate(self.initial_joint_angles):
            delta_angle = np.pi / 16 * (1 - np.cos(np.pi / 5.0 * time_since_start)) * 0.2
            if  i == 4:
                trajectory_point.positions[i] = angle + self.amplitude * np.sin(2 * np.pi * self.frequency * time_since_start)
            else:
                trajectory_point.positions[i] = angle + delta_angle
        trajectory_point.time_from_start = rospy.Duration(time_since_start)
        return trajectory_point


def callback(controller_interface, sinusoidal_trajectory, timer_event):
    """
    Callback function that is called periodically to compute and send the sinusoidal trajectory point.
    Args:
        controller_interface (ControllerInterface): The controller interface to send commands.
        timer_event: The timer event that triggers this callback.
    """
    trajectory_point = sinusoidal_trajectory.get_trajectory_point()
    controller_interface.joint_command(trajectory_point)

if __name__ == '__main__':
    rospy.init_node('sinusoidal_trajectory_planner', anonymous=True)

    jointspace_topic = rospy.get_param("~jointspace_topic", "/rug_panda/joint_target")
    workspace_topic = rospy.get_param("~workspace_topic", "/rug_panda/workspace_target")
    controller_interface = ControllerInterface(jointspace_topic=jointspace_topic, workspace_topic=workspace_topic)

    # one time retrieval of the initial state of the robot
    franka_state = rospy.wait_for_message('franka_state_controller/franka_states', FrankaState)
    initial_joint_angles = franka_state.q
    sinusoidal_trajectory = SinusoidalTrajectory(initial_joint_angles)

    timer = rospy.Timer(
        rospy.Duration(0.1),
        lambda timer_event: callback(controller_interface, sinusoidal_trajectory, timer_event)
    )
    rospy.spin()
    rospy.loginfo("Sinusoidal Trajectory Planner Node Initialized")
