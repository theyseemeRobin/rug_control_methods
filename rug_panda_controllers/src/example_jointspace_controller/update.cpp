#include <rug_panda_controllers/example_jointspace_controller.h>
#include <cmath>

namespace rug_panda_controllers
{

void ExampleJointspaceController::update(const ros::Time& /*time*/, const ros::Duration& period) {
    elapsed_time_ += period;

    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Vector7d> dq(robot_state.dq.data());
    Eigen::Map<Vector7d> q(robot_state.q.data());

    Vector7d error;
    double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
    for (size_t i = 0; i < 7; ++i) {
        if(i == 4)
        {
            target_joint_angles[i] = initial_joint_angles[i] - delta_angle;
        }
        else
        {
            target_joint_angles[i] = initial_joint_angles[i] + delta_angle;
        }
    }
    error = target_joint_angles - q;


    // compute control
    Eigen::VectorXd torques(7);
    torques = 600 * error - 30 * dq;

    // Send torque commands
    for (size_t i = 0; i < 7; ++i)
    {
        joint_handles_[i].setCommand(torques[i]);
    }
}

}    // namespace rug_panda_controllers