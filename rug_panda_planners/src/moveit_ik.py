import rospy
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped


def joint_state_to_joint_trajectory_point(joint_state):
    """
    Converts a JointState message to a JointTrajectoryPoint for use in inverse kinematics.

    Args:
        joint_state (JointState): The joint state message containing positions, velocities, and effort.
    Returns:
        JointTrajectoryPoint: A JointTrajectoryPoint with positions, velocities, effort, and accelerations.
    """
    joint_trajectory_point = JointTrajectoryPoint()
    joint_trajectory_point.positions = joint_state.position[:7]
    joint_trajectory_point.velocities = joint_state.velocity[:7]
    joint_trajectory_point.effort = joint_state.effort[:7]
    joint_trajectory_point.accelerations = [0] * 7
    return joint_trajectory_point

class InverseKinematics(object):
    def __init__(self, group_name="panda_arm"):
        """
        Initializes the InverseKinematics class with the specified MoveIt group name.

        Args:
            group_name (str): The name of the MoveIt group for which to compute inverse kinematics.
        """
        # Wait for the inverse kinematics service to be available
        rospy.wait_for_service('/compute_ik', timeout=rospy.Duration(5.0))
        self.ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        self.group_name = group_name
        self.move_group = MoveGroupCommander(group_name)

    def get_ik(
            self,
            pose,
            avoid_collisions=True,
            timeout=1,
            frame_id='panda_link0'
    ):
        """
        Computes the inverse kinematics for a given pose.

        Args:
            pose (PoseStamped): The desired pose for the end effector.
            avoid_collisions (bool): Whether to avoid collisions during IK computation.
            timeout (float): The timeout for the IK service call in seconds.
            frame_id (str): The frame in which the pose is expressed.
        Returns:
            JointTrajectoryPoint: A joint trajectory point representing the IK solution.
        """

        req = GetPositionIKRequest()
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.timeout = rospy.Duration(timeout)
        req.ik_request.group_name = self.group_name
        req.ik_request.pose_stamped = pose
        req.ik_request.pose_stamped.header.frame_id = frame_id

        seed_joint_values = self.move_group.get_current_joint_values()
        joint_state = JointState()
        joint_state.position= seed_joint_values
        joint_state.header.frame_id = frame_id
        joint_state.name = [f'panda_joint{i}' for i in range(1, 8)]
        req.ik_request.robot_state.joint_state = joint_state

        try:
            resp = self.ik_srv(req)
            ik_solution = joint_state_to_joint_trajectory_point(resp.solution.joint_state)
            if resp.error_code.val != resp.error_code.SUCCESS:
                raise RuntimeError(f"MoveIt inverse kinematics failed with error code {resp.error_code.val}")
            return ik_solution
        except rospy.ServiceException as e:
            rospy.logerr("MoveIt inverse kinematics service exception: " + str(e))
            raise e