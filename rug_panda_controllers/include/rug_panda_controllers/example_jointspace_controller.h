#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace rug_panda_controllers {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

class ExampleJointspaceController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                  	        franka_hw::FrankaStateInterface>
{

	// handles
	std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
	std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
	std::vector<hardware_interface::JointHandle> joint_handles_;

	// target
	ros::Subscriber sub_target;
	std::mutex target_mutex;
	Vector7d target_joint_angles;
	Vector7d initial_joint_angles;

	ros::Duration elapsed_time_;

	public:
		bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
		void starting(const ros::Time&) override;
		void update(const ros::Time&, const ros::Duration& period) override;

	private:
		void targetCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg);

};
}    // namespace rug_panda_controllers