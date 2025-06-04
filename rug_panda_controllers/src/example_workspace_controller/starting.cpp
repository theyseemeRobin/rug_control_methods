#include <rug_panda_controllers/example_workspace_controller.h>

namespace rug_panda_controllers
{

void ExampleWorkspaceController::starting(const ros::Time& /*time*/) {
    franka::RobotState initial_state = state_handle->getRobotState();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    target_pose = initial_transform.translation();
    target_orientation = Eigen::Quaterniond(initial_transform.rotation());
}

}    // namespace rug_panda_controllers