#include <rug_panda_controllers/example_workspace_controller.h>

namespace rug_panda_controllers
{


void ExampleWorkspaceController::targetCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    std::lock_guard<std::mutex> target_posemutex_lock(target_mutex);
    target_pose << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_target_orientation(target_orientation);
    target_orientation.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_target_orientation.coeffs().dot(target_orientation.coeffs()) < 0.0) {
    target_orientation.coeffs() << -target_orientation.coeffs();
  }
    
}

}        // namespace rug_panda_controllers