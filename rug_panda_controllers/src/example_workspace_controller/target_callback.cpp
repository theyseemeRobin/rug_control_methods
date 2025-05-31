#include <rug_panda_controllers/example_workspace_controller.h>

namespace rug_panda_controllers
{


void ExampleWorkspaceController::targetCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg)
{
    std::lock_guard<std::mutex> target_mutex_lock(target_mutex);

    // position
    target_pose << msg->positions[0], msg->positions[1], msg->positions[2];
    Eigen::Quaterniond last_target_orientation(target_orientation);
    target_orientation.coeffs() << msg->positions[4], msg->positions[5], msg->positions[6], msg->positions[7];
    if (last_target_orientation.coeffs().dot(target_orientation.coeffs()) < 0.0) {
        target_orientation.coeffs() << -target_orientation.coeffs();
    }
}

}        // namespace rug_panda_controllers