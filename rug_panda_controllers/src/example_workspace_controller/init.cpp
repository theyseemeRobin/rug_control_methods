#include <rug_panda_controllers/example_workspace_controller.h>
#include <pluginlib/class_list_macros.h>

namespace rug_panda_controllers
{

bool ExampleWorkspaceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{

        sub_target = node_handle.subscribe(
                        "/rug_panda/workspace_target",
                        20,
                        &ExampleWorkspaceController::targetCallback,
                        this,
                        ros::TransportHints().reliable().tcpNoDelay()
        );

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
                ROS_ERROR_STREAM("ExampleWorkspaceController: Could not read parameter arm_id");
                return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
                ROS_ERROR(
                                "ExampleWorkspaceController: Invalid or no joint_names parameters provided, "
                                "aborting controller init!");
                return false;
        }

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
                ROS_ERROR_STREAM(
                                "ExampleWorkspaceController: Error getting model interface from hardware");
                return false;
        }
        try {
                model_handle = std::make_unique<franka_hw::FrankaModelHandle>(
                                model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                                "ExampleWorkspaceController: Exception getting model handle from interface: "
                                << ex.what());
                return false;
        }

        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM(
                "ExampleWorkspaceController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
                "ExampleWorkspaceController: Exception getting state handle from interface: "
                << ex.what());
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
                "ExampleWorkspaceController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
            joint_handles.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "ExampleWorkspaceController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    target_pose.setZero();
    target_orientation.coeffs() << 0.0, 0.0, 0.0, 1.0;

    return true;
}

}    // namespace rug_panda_controllers

PLUGINLIB_EXPORT_CLASS(rug_panda_controllers::ExampleWorkspaceController, controller_interface::ControllerBase)