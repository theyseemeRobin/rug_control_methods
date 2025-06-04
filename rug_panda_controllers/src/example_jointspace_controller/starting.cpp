#include <rug_panda_controllers/example_jointspace_controller.h>

namespace rug_panda_controllers
{

void ExampleJointspaceController::starting(const ros::Time& /*time*/) {
    franka::RobotState initial_state = state_handle->getRobotState();
    Eigen::Map<Vector7d> q(initial_state.q.data());
    target_joint_angles = q;
    initial_joint_angles = q;
    elapsed_time = ros::Duration(0.0);
}

}    // namespace rug_panda_controllers