#include <rug_panda_controllers/example_jointspace_controller.h>
#include <cmath>

namespace rug_panda_controllers
{

void ExampleJointspaceController::update(const ros::Time& /*time*/, const ros::Duration& period) {
    elapsed_time += period;

    // get state variables
    franka::RobotState robot_state = state_handle->getRobotState();
    std::array<double, 42> jacobian_array = model_handle->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Vector7d> dq(robot_state.dq.data());
    Eigen::Map<Vector7d> q(robot_state.q.data());

    Vector7d error;
    error = target_joint_angles - q;


    // compute control
    Eigen::VectorXd torques(7);
    torques = error;

    // Send torque commands
    for (size_t i = 0; i < 7; ++i)
    {
        joint_handles[i].setCommand(torques[i]);
    }
}

}    // namespace rug_panda_controllers