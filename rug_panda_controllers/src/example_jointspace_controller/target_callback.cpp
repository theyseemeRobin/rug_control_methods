#include <rug_panda_controllers/example_jointspace_controller.h>

namespace rug_panda_controllers
{


void ExampleJointspaceController::targetCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg)
{
    std::lock_guard<std::mutex> target_mutex_lock(target_mutex);

    if (msg->positions.size() != 7) {
        throw std::invalid_argument("JointInverseDynamics: Invalid positions size, must be 7");
    }
    for (size_t i = 0; i < 7; ++i) {
        target_joint_angles[i] = msg->positions[i];
    }
}

}        // namespace rug_panda_controllers