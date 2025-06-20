import rospy
from moveit_msgs.srv import GetPositionFK, GetPositionFKResponse
from moveit_msgs.srv import GetPositionFKRequest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def joint_trajectory_point_to_joint_state(joint_trajectory_point):
    """
    Converts a JointTrajectoryPoint to a JointState message for forward kinematics computation.

    Args:
        joint_trajectory_point (JointTrajectoryPoint): The joint trajectory point containing positions, velocities, and effort.
    Returns:
        JointState: A JointState message with the joint positions, velocities, and effort.
    """
    joint_state = JointState()
    joint_state.position = joint_trajectory_point.positions
    joint_state.velocity = joint_trajectory_point.velocities
    joint_state.effort = joint_trajectory_point.effort
    joint_state.header.frame_id = 'panda_link0'
    joint_state.name = ['panda_joint' + str(i) for i in range(1, 8)]
    return joint_state


class ForwardKinematics(object):
    def __init__(self):
        self.fk_srv = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        self.fk_srv.wait_for_service(timeout=rospy.Duration(5.0))

    def get_fk(
            self,
            joint_trajectory_point,
            fk_link="panda_hand",
            frame_id="panda_link0"
):
        """
        Computes the forward kinematics for a given joint trajectory point and link.
        Args:
            joint_trajectory_point (JointTrajectoryPoint): The joint trajectory point containing joint positions.
            fk_link (str): The link for which to compute the forward kinematics.
            frame_id (str): The frame in which the pose should be expressed.

        Returns:
            geometry_msgs.msg.PoseStamped: The pose of the specified link in the given frame.
        """
        req = GetPositionFKRequest()
        req.header.frame_id = frame_id
        req.fk_link_names = [fk_link]
        req.robot_state.joint_state = joint_trajectory_point_to_joint_state(joint_trajectory_point)
        try:
            resp = self.fk_srv.call(req)
            return resp.pose_stamped[0]
        except rospy.ServiceException as e:
            rospy.logerr("MoveIt forward kinematics service exception: " + str(e))
            raise e