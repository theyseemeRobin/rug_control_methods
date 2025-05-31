#include <rug_panda_controllers/example_workspace_controller.h>

namespace rug_panda_controllers
{

void ExampleWorkspaceController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Vector7d> dq(robot_state.dq.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));


    // compute control
    Eigen::VectorXd torques(7);
    torques.setZero();              // Now fixed to zero as an examples

    // Send torque commands
    for (size_t i = 0; i < 7; ++i)
    {
        joint_handles_[i].setCommand(torques(i));
    }
}

}    // namespace rug_panda_controllers