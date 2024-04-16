#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <unistd.h>

#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>

#include <ariac_msgs/srv/conveyor_belt_control.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>
#include <ariac_msgs/srv/move_to_pos.hpp>
#include <ariac_msgs/srv/get_pos.hpp>
#include <ariac_msgs/srv/set_joint_value_target.hpp>
#include <ariac_msgs/srv/attach.hpp>
#include <ariac_msgs/srv/place_part_on_kit_tray.hpp>
#include <ariac_msgs/srv/multiply_pose.hpp>
#include <ariac_msgs/srv/assemble.hpp>

#include <geometry_msgs/msg/pose.hpp>


class RobotCommander : public rclcpp::Node {
public:
    RobotCommander();

    ~RobotCommander() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr node2_;
    std::thread executor_thread_;
    rclcpp::Executor::SharedPtr executor_;
// MoveIt Interfaces
    moveit::planning_interface::MoveGroupInterface floor_robot_;
    moveit::planning_interface::MoveGroupInterface ceiling_robot_;

// ROS Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr floor_robot_move_home_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ceiling_robot_move_home_srv_;
    rclcpp::Service<ariac_msgs::srv::MultiplyPose>::SharedPtr multiply_pose_srv_;
    rclcpp::Service<ariac_msgs::srv::MoveToPos>::SharedPtr move_to_srv_;
    rclcpp::Service<ariac_msgs::srv::SetJointValueTarget>::SharedPtr set_joint_value_target_srv_;
    rclcpp::Service<ariac_msgs::srv::Attach>::SharedPtr wait_for_attach_srv_;
    rclcpp::Service<ariac_msgs::srv::Attach>::SharedPtr detach_srv_;
    rclcpp::Service<ariac_msgs::srv::Assemble>::SharedPtr wait_for_assemble_srv_;

    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
    rclcpp::Client<ariac_msgs::srv::ConveyorBeltControl>::SharedPtr set_conveyor_power_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_bins_camera_sub_;

    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;

    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;

    rclcpp::SubscriptionOptions options;

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
    // Assembly States
    std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;

    bool conveyor_bins_camera_received_data = false;
    // Bins
    std::vector<ariac_msgs::msg::PartPose> conveyor_bins_parts_;
    // Sensor poses
    geometry_msgs::msg::Pose conveyor_bins_camera_pose_;
    // Constants
    double kit_tray_thickness_ = 0.01;
    double drop_height_ = 0.002;
    double pick_offset_ = 0.003;
    double battery_grip_offset_ = -0.05;
    // Quadrant Offsets
    std::map<int, std::pair<double, double>> quad_offsets_ = {
            {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
    };
    std::map<int, std::string> part_colors_ = {
            {ariac_msgs::msg::Part::RED,    "red"},
            {ariac_msgs::msg::Part::BLUE,   "blue"},
            {ariac_msgs::msg::Part::GREEN,  "green"},
            {ariac_msgs::msg::Part::ORANGE, "orange"},
            {ariac_msgs::msg::Part::PURPLE, "purple"},
    };
    std::map<int, std::string> part_types_ = {
            {ariac_msgs::msg::Part::BATTERY,   "battery"},
            {ariac_msgs::msg::Part::PUMP,      "pump"},
            {ariac_msgs::msg::Part::REGULATOR, "regulator"},
            {ariac_msgs::msg::Part::SENSOR,    "sensor"}
    };
    // Part heights
    std::map<int, double> part_heights_ = {
            {ariac_msgs::msg::Part::BATTERY,   0.04},
            {ariac_msgs::msg::Part::PUMP,      0.12},
            {ariac_msgs::msg::Part::REGULATOR, 0.07},
            {ariac_msgs::msg::Part::SENSOR,    0.07}
    };
    std::map<std::string, double> rail_positions_ = {
            {"agv1",       -4.5},
            {"agv2",       -1.2},
            {"agv3",       1.2},
            {"agv4",       4.5},
            {"left_bins",  3},
            {"right_bins", -3}
    };


//  function declaration

    void AddModelToPlanningScene(
            std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);

    geometry_msgs::msg::Pose BuildPose(
            double x, double y, double z, geometry_msgs::msg::Quaternion orientation);

    geometry_msgs::msg::Pose MultiplyPose(
            geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);

    geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

    double GetYaw(geometry_msgs::msg::Pose pose);

// Service Callbacks

    bool MultiplyPoseSrv(ariac_msgs::srv::MultiplyPose::Request::SharedPtr req,
                         ariac_msgs::srv::MultiplyPose::Response::SharedPtr res);

    void FloorRobotMoveHome(
            std_srvs::srv::Trigger::Request::SharedPtr req,
            std_srvs::srv::Trigger::Response::SharedPtr res);

    bool WaitForAttach(ariac_msgs::srv::Attach::Request::SharedPtr req,
                       ariac_msgs::srv::Attach::Response::SharedPtr res);

    bool Detach(ariac_msgs::srv::Attach::Request::SharedPtr req,
                ariac_msgs::srv::Attach::Response::SharedPtr res);

    bool WaitForAssemble(ariac_msgs::srv::Assemble::Request::SharedPtr req,
                         ariac_msgs::srv::Assemble::Response::SharedPtr res);

    void CeilingRobotMoveHome(
            std_srvs::srv::Trigger::Request::SharedPtr req,
            std_srvs::srv::Trigger::Response::SharedPtr res);

    bool FloorRobotMovetoTarget();

    bool RobotMovetoTarget(ariac_msgs::srv::MoveToPos::Request::SharedPtr req,
                           ariac_msgs::srv::MoveToPos::Response::SharedPtr res);

    bool SetJointValueTarget(ariac_msgs::srv::SetJointValueTarget::Request::SharedPtr req,
                             ariac_msgs::srv::SetJointValueTarget::Response::SharedPtr res);

    bool CeilingRobotMovetoTarget();

    bool RobotMoveCartesian(const std::string &robot_name,
                            const std::vector<geometry_msgs::msg::Pose> &waypoints, double vsf, double asf,
                            bool avoid_collisions);


    void conveyor_bins_camera_cb(
            ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

    void floor_gripper_state_cb(ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

    void ceiling_gripper_state_cb(ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

    void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

    void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

    void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

    void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

    void AddModelsToPlanningScene();

    geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

};
