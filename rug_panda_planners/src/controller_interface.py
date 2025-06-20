import actionlib
from franka_gripper.msg import (
    MoveGoal,
    MoveAction,
    GraspAction,
    GraspGoal,
    HomingAction,
    HomingGoal,
    StopAction,
    StopGoal,
    GraspEpsilon
)
import rospy
from franka_msgs.msg import FrankaState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

from moveit_fk import ForwardKinematics
from moveit_ik import InverseKinematics


def grasp_client_command(client, goal):
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    return result


class ControllerInterface:
    """
    This class provides a python interface for sending commands to the controller. The way the regular
    jointTrajectoryPoints are processed is defined within the controllers, and can therefore easily be changed. The
    gripper commands make use of the action servers defined in the franka_gripper package. More information can be found
    here: https://frankaemika.github.io/docs/franka_ros.html#franka-gripper.
    """

    def __init__(self, jointspace_topic='/rug_panda/joint_target', workspace_topic='/rug_panda/workspace_target'):
        """
        Initializes the ControllerInterface with a publisher topic.
        Args:
            pub_topic (str): The topic to which the controller will publish commands.
        """

        self.jointspace_topic = jointspace_topic
        self.workspace_topic = workspace_topic
        self.jointspace_publisher = rospy.Publisher(jointspace_topic, JointTrajectoryPoint, queue_size=10)
        self.workspace_publisher = rospy.Publisher(workspace_topic, PoseStamped, queue_size=10)

        self.gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.gripper_homing_client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
        self.gripper_stop_client = actionlib.SimpleActionClient('/franka_gripper/stop', StopAction)

        self.fk = ForwardKinematics()
        self.ik = InverseKinematics()

    def joint_command(self, joint_trajectory_point, send_workspace=True):
        """
        Sends a JointTrajectoryPoint to the controller. If send_workspace is True, it also sends the workspace command
        by computing the forward kinematics for the given joint trajectory point.

        Args:
            joint_trajectory_point (JointTrajectoryPoint): The trajectory point to send.
        """
        if not isinstance(joint_trajectory_point, JointTrajectoryPoint):
            raise TypeError("joint_trajectory_point must be of type JointTrajectoryPoint")
        self.jointspace_publisher.publish(joint_trajectory_point)

        if send_workspace:
            pose_stamped = self.fk.get_fk(joint_trajectory_point, 'panda_hand', 'panda_link0')
            self.workspace_command(pose_stamped, send_jointspace=False)

    def workspace_command(self, pose_stamped, send_jointspace=True):
        """
        Sends a pose_stamped to the controller. If send_jointspace is True, it also sends the joint space command
        by computing the inverse kinematics for the given pose. Note that with the current IK, a solution is not
        guaranteed, in which case no joint space command is sent.

        Args:
            pose_stamped (PoseStamped): The pose to send.
            send_jointspace (bool): Whether to also send the joint space command.
        """
        if not isinstance(pose_stamped, PoseStamped):
            raise TypeError("pose_stamped must be of type JointTrajectoryPoint")
        self.workspace_publisher.publish(pose_stamped)

        if send_jointspace:
            try:
                joint_trajectory_point = self.ik.get_ik(
                    pose_stamped,
                )
            except Exception as e:
                rospy.logwarn("Inverse Kinematics failed: {}, not sending joint space command.".format(e))
                return
            self.joint_command(joint_trajectory_point, send_workspace=False)


    def gripper_grasp(
            self,
            width = 0,
            speed = 1,
            force = 5.0,
            inner_epsilon = 0.001,
            outer_epsilon = 0.001,
    ):
        """
        Performs a grasp action with the gripper. It tries to grasp at the desired width with a desired force while
        closing with the given speed. The operation is successful if the distance between the gripper fingers is:
        width - inner_epsilon < distance < width + outer_epsilon.

        Args:
            width (float): The desired width to grasp at. Default is 0.
            speed (float): The speed at which the gripper closes. Default is 1.
            force (float): The force with which the gripper closes. Default is 5.0.
            inner_epsilon (float): The inner epsilon for the grasping operation. Default is 0.001.
            outer_epsilon (float): The outer epsilon for the grasping operation. Default is 0.001.
        """
        goal = GraspGoal(width=width, speed=speed, force=force, epsilon=GraspEpsilon(inner=inner_epsilon, outer=outer_epsilon))
        return grasp_client_command(self.gripper_grasp_client, goal)

    def gripper_move(self, width, speed):
        """
        Moves the gripper to a specified width at a given speed.
        Args:
            width (float): The desired width to move the gripper to.
            speed (float): The speed at which the gripper moves.
        """
        goal = MoveGoal(width=width, speed=speed)
        return grasp_client_command(self.gripper_move_client, goal)

    def gripper_homing(self):
        """
        Homes the gripper and updates the maximum width given the mounted fingers.
        """
        goal = HomingGoal()
        return grasp_client_command(self.gripper_homing_client, goal)

    def gripper_stop(self):
        """
        Stops the currently active gripper action.
        """
        goal = StopGoal()
        return grasp_client_command(self.gripper_stop_client, goal)