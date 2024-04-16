#include <robot_commander.hpp>

RobotCommander::RobotCommander()
        : Node("robot_commander"),
          node_(std::make_shared<rclcpp::Node>("example_group_node")),
          node2_(std::make_shared<rclcpp::Node>("example_group_node2")),
          executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
          floor_robot_(node_, "floor_robot"),
          ceiling_robot_(node2_, "ceiling_robot"),
          planning_scene_() {
    executor_->add_node(node_);
    executor_->add_node(node2_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
// Use upper joint velocity and acceleration limits
    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);

    ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
    ceiling_robot_.setMaxVelocityScalingFactor(1.0);

    topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = topic_cb_group_;
    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

// Register services
    floor_robot_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
            "/competitor/move_floor_robot_home",
            std::bind(
                    &RobotCommander::FloorRobotMoveHome, this,
                    std::placeholders::_1, std::placeholders::_2));

    ceiling_robot_move_home_srv_ = create_service<std_srvs::srv::Trigger>(
            "/competitor/move_ceiling_robot_home",
            std::bind(
                    &RobotCommander::CeilingRobotMoveHome, this,
                    std::placeholders::_1, std::placeholders::_2));

    move_to_srv_ = create_service<ariac_msgs::srv::MoveToPos>(
            "/ariac/move_to_pos",
            std::bind(
                    &RobotCommander::RobotMovetoTarget, this,
                    std::placeholders::_1, std::placeholders::_2));

    multiply_pose_srv_ = create_service<ariac_msgs::srv::MultiplyPose>(
            "/ariac/multiply_pose",
            std::bind(
                    &RobotCommander::MultiplyPoseSrv, this,
                    std::placeholders::_1, std::placeholders::_2));
    set_joint_value_target_srv_ = create_service<ariac_msgs::srv::SetJointValueTarget>(
            "/ariac/set_joint_value_target", std::bind(
                    &RobotCommander::SetJointValueTarget, this,
                    std::placeholders::_1, std::placeholders::_2));
    wait_for_attach_srv_ = create_service<ariac_msgs::srv::Attach>(
            "/ariac/wait_for_attach", std::bind(
                    &RobotCommander::WaitForAttach, this,
                    std::placeholders::_1, std::placeholders::_2));
    detach_srv_ = create_service<ariac_msgs::srv::Attach>("/ariac/detach_object", std::bind(
            &RobotCommander::Detach, this,
            std::placeholders::_1, std::placeholders::_2));

    wait_for_assemble_srv_ = create_service<ariac_msgs::srv::Assemble>("/ariac/wait_for_assemble", std::bind(
            &RobotCommander::WaitForAssemble, this, std::placeholders::_1, std::placeholders::_2));


    floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
            "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
            std::bind(&RobotCommander::floor_gripper_state_cb, this, std::placeholders::_1), options);

    ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
            "/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(),
            std::bind(&RobotCommander::ceiling_gripper_state_cb, this, std::placeholders::_1), options);

    conveyor_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/conveyor_bins_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&RobotCommander::conveyor_bins_camera_cb, this, std::placeholders::_1), options);

    as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
            "/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(),
            std::bind(&RobotCommander::as1_state_cb, this, std::placeholders::_1), options);

    as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
            "/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(),
            std::bind(&RobotCommander::as2_state_cb, this, std::placeholders::_1), options);

    as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
            "/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(),
            std::bind(&RobotCommander::as3_state_cb, this, std::placeholders::_1), options);

    as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
            "/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(),
            std::bind(&RobotCommander::as4_state_cb, this, std::placeholders::_1), options);

    floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>(
            "/ariac/floor_robot_enable_gripper", rmw_qos_profile_services_default,
            client_cb_group_);
    ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>(
            "/ariac/ceiling_robot_enable_gripper", rmw_qos_profile_services_default, client_cb_group_);


    AddModelsToPlanningScene();
    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

RobotCommander::~RobotCommander() {
    floor_robot_.~MoveGroupInterface();
    ceiling_robot_.~MoveGroupInterface();
}

void RobotCommander::AddModelToPlanningScene(
        std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose) {
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ariac_test");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision);

    planning_scene_.addCollisionObjects(collision_objects);
}

// CeilingRobotMovetoTarget
bool RobotCommander::CeilingRobotMovetoTarget() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(ceiling_robot_.plan(plan));

    if (success) {
        return static_cast<bool>(ceiling_robot_.execute(plan));
    } else {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

void RobotCommander::CeilingRobotMoveHome(
        std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res) {
    (void) req; // remove unused parameter warning
    ceiling_robot_.setNamedTarget("home");

    bool success = CeilingRobotMovetoTarget();

    if (success) {
        res->success = true;
    } else {
        res->message = "Unable to generate trajectory";
        res->success = false;
    }
}

bool RobotCommander::RobotMovetoTarget(ariac_msgs::srv::MoveToPos::Request::SharedPtr req,
                                       ariac_msgs::srv::MoveToPos::Response::SharedPtr res) {
    if (req->type == "cartesian") {
        res->success = RobotMoveCartesian(req->robot_name, req->waypoints, 0.5, 0.5, req->avoid_collisions);
        return res->success;
    } else {
        if (req->robot_name == "floor_robot") {
            floor_robot_.setPoseTarget(req->pose);
        } else {
            ceiling_robot_.setPoseTarget(req->pose);
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(floor_robot_.plan(plan));

        if (success) {
            res->success = true;
            return static_cast<bool>(floor_robot_.execute(plan));
        } else {
            res->success = false;
            RCLCPP_ERROR(get_logger(), "Unable to generate plan");
            return false;
        }
    }
}

bool RobotCommander::WaitForAttach(ariac_msgs::srv::Attach::Request::SharedPtr req,
                                   ariac_msgs::srv::Attach::Response::SharedPtr res) {
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    ariac_msgs::msg::VacuumGripperState *robot_state;
    geometry_msgs::msg::Pose starting_pose;
    if (req->robot_name == "floor_robot") {
        robot_state = &floor_gripper_state_;
        starting_pose = floor_robot_.getCurrentPose().pose;
    } else {
        robot_state = &ceiling_gripper_state_;
        starting_pose = ceiling_robot_.getCurrentPose().pose;
    }

    while (!(*robot_state).attached) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");
        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);
        if (req->robot_name == "ceiling_bot")
            RobotMoveCartesian(req->robot_name, waypoints, 0.01, 0.01, false);
        else
            RobotMoveCartesian(req->robot_name, waypoints, 0.2, 0.1, true);
        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(req->timeout)) {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return false;
        }
    }
    int part_type = req->part.type;
    int part_color = req->part.color;
    std::string part_name = part_colors_[part_color] + "_" + part_types_[part_type];
    AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", req->pose);
    if (req->robot_name == "floor_robot"){
        floor_robot_.attachObject(part_name);
        auto current_pose = floor_robot_.getCurrentPose().pose;
        current_pose.position.z += 0.2;
        waypoints.clear();
        waypoints.push_back(current_pose);
        RobotMoveCartesian("floor_robot", waypoints, 0.7, 0.7, false);
    }
    else if (req->robot_name == "ceiling_robot"){
        ceiling_robot_.attachObject(part_name);
        auto current_pose = ceiling_robot_.getCurrentPose().pose;
        current_pose.position.z += 0.2;
        waypoints.clear();
        waypoints.push_back(current_pose);
        RobotMoveCartesian("ceiling_robot", waypoints, 0.7, 0.7, true);
        }

    res->success = true;

    return true;
}

bool RobotCommander::Detach(ariac_msgs::srv::Attach::Request::SharedPtr req,
                            ariac_msgs::srv::Attach::Response::SharedPtr res) {
    int part_type = req->part.type;
    int part_color = req->part.color;
    std::string part_name = part_colors_[part_color] + "_" + part_types_[part_type];
    std::vector<geometry_msgs::msg::Pose> waypoints;
    if (req->robot_name == "floor_robot"){
        floor_robot_.detachObject(part_name);
        geometry_msgs::msg::Pose current_pose = floor_robot_.getCurrentPose().pose;
        current_pose.position.z += 0.2;
        waypoints.push_back(current_pose);
        RobotMoveCartesian("floor_robot", waypoints, 0.2, 0.1, false);}
    else if (req->robot_name == "ceiling_robot"){
        ceiling_robot_.detachObject(part_name);
        auto current_pose = ceiling_robot_.getCurrentPose().pose;

        if (part_type == ariac_msgs::msg::Part::REGULATOR){
            current_pose.position.x -= 0.05;
            }else{
        current_pose.position.z += 0.1;
        }
        waypoints.push_back(current_pose);
        RobotMoveCartesian("ceiling_robot", waypoints, 0.3, 0.3, true);
        //CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);
    }

    usleep(300);
    res->success = true;
    return true;
}

bool RobotCommander::RobotMoveCartesian(
        const std::string &robot_name, const std::vector<geometry_msgs::msg::Pose> &waypoints, double vsf, double asf,
        bool avoid_collisions = false) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    if (robot_name == "floor_robot") {
        double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        if (path_fraction < 0.9) {
            RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
            return false;
        }
        // Retime trajectory
        robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
        rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
        totg_.computeTimeStamps(rt, vsf, asf);
        rt.getRobotTrajectoryMsg(trajectory);
        return static_cast<bool>(floor_robot_.execute(trajectory));
    } else if (robot_name == "ceiling_robot") {
        double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);
        if (path_fraction < 0.9) {
            RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
            return false;
        }
        // Retime trajectory
        robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
        rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
        totg_.computeTimeStamps(rt, vsf, asf);
        rt.getRobotTrajectoryMsg(trajectory);
        return static_cast<bool>(ceiling_robot_.execute(trajectory));
    }
    return false;
}

bool RobotCommander::WaitForAssemble(ariac_msgs::srv::Assemble::Request::SharedPtr req,
                                     ariac_msgs::srv::Assemble::Response::SharedPtr res) {
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

    bool assembled = false;
    while (!assembled) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

        // Check if part is assembled
        switch (req->part.type) {
            case ariac_msgs::msg::Part::BATTERY:
                assembled = assembly_station_states_[req->station].battery_attached;
                break;
            case ariac_msgs::msg::Part::PUMP:
                assembled = assembly_station_states_[req->station].pump_attached;
                break;
            case ariac_msgs::msg::Part::SENSOR:
                assembled = assembly_station_states_[req->station].sensor_attached;
                break;
            case ariac_msgs::msg::Part::REGULATOR:
                assembled = assembly_station_states_[req->station].regulator_attached;
                break;
            default:
                RCLCPP_WARN(get_logger(), "Not a valid part type");
                return false;
        }

        double step = 0.0005;

        waypoints.clear();
        starting_pose.position.x += step * req->install_direction[0];
        starting_pose.position.y += step * req->install_direction[1];
        starting_pose.position.z += step * req->install_direction[2];
        waypoints.push_back(starting_pose);

        RobotMoveCartesian("ceiling_robot", waypoints, 0.01, 0.01, false);

        usleep(300);

        if (now() - start > rclcpp::Duration::from_seconds(3)) {
            RCLCPP_ERROR(get_logger(), "Unable to assemble object");
            //ceiling_robot_.stop();
            return false;
        }
    }

    RCLCPP_INFO(get_logger(), "Part is assembled");
    res->success = true;
    return true;
}

// ****************************************************** //
//                  floor_robot                         //
// ****************************************************** //
// FloorRobotMovetoTarget
bool RobotCommander::FloorRobotMovetoTarget() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_robot_.plan(plan));

    if (success) {
        return static_cast<bool>(floor_robot_.execute(plan));
    } else {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool RobotCommander::SetJointValueTarget(ariac_msgs::srv::SetJointValueTarget::Request::SharedPtr req,
                                         ariac_msgs::srv::SetJointValueTarget::Response::SharedPtr res) {
    if (req->robot_name == "floor_robot") {
        floor_robot_.setJointValueTarget(req->joint, req->value);
        res->success = FloorRobotMovetoTarget();
    } else if (req->robot_name == "ceiling_robot") {
        ceiling_robot_.setJointValueTarget(req->joint, req->value);
        res->success = CeilingRobotMovetoTarget();
    }
    return res->success;
}


void RobotCommander::FloorRobotMoveHome(
        std_srvs::srv::Trigger::Request::SharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr res) {
    (void) req; // remove unused parameter warning
    floor_robot_.setNamedTarget("home");


    bool success = FloorRobotMovetoTarget();

    if (success) {
        res->success = true;
    } else {
        res->message = "Unable to generate trajectory";
        res->success = false;
    }
}




// ****************************************************** //
//                  floor_gripper                         //

void RobotCommander::floor_gripper_state_cb(
        const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) {
    floor_gripper_state_ = *msg;
}

void RobotCommander::ceiling_gripper_state_cb(
        const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) {
    ceiling_gripper_state_ = *msg;
}

void RobotCommander::conveyor_bins_camera_cb(
        const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) {
    if (!conveyor_bins_camera_received_data) {
        RCLCPP_INFO(get_logger(), "Received data from conveyor bins camera");
        conveyor_bins_camera_received_data = true;
    }

    conveyor_bins_parts_ = msg->part_poses;
    conveyor_bins_camera_pose_ = msg->sensor_pose;
}

// uint8 AS1=1
//uint8 AS2=2
//uint8 AS3=3
//uint8 AS4=4
void RobotCommander::as1_state_cb(
        const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg) {
    assembly_station_states_.insert_or_assign(1, *msg);
}

void RobotCommander::as2_state_cb(
        const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg) {
    assembly_station_states_.insert_or_assign(2, *msg);
}

void RobotCommander::as3_state_cb(
        const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg) {
    assembly_station_states_.insert_or_assign(3, *msg);
}

void RobotCommander::as4_state_cb(
        const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg) {
    assembly_station_states_.insert_or_assign(4, *msg);
}

// ****************************************************** //
//                  utils                         //
geometry_msgs::msg::Pose RobotCommander::BuildPose(
        double x, double y, double z, geometry_msgs::msg::Quaternion orientation) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose RobotCommander::MultiplyPose(
        geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2) {
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
}

bool RobotCommander::MultiplyPoseSrv(
        ariac_msgs::srv::MultiplyPose::Request::SharedPtr req,
        ariac_msgs::srv::MultiplyPose::Response::SharedPtr res) {
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(req->poses[0], f1);
    for (int i = 1; i < (int)req->poses.size(); i++) {
        tf2::fromMsg(req->poses[i], f2);
        f1 = f1 * f2;
    }
    res->pose = tf2::toMsg(f1);
    res->success = true;
    return true;
}

geometry_msgs::msg::Quaternion RobotCommander::SetRobotOrientation(double rotation) {
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
}

double RobotCommander::GetYaw(geometry_msgs::msg::Pose pose) {
    tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::msg::Quaternion RobotCommander::QuaternionFromRPY(double r, double p, double y) {
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}


void RobotCommander::AddModelsToPlanningScene() {
    // Add bins
    std::map<std::string, std::pair<double, double>> bin_positions = {
            {"bin1", std::pair<double, double>(-1.9, 3.375)},
            {"bin2", std::pair<double, double>(-1.9, 2.625)},
            {"bin3", std::pair<double, double>(-2.65, 2.625)},
            {"bin4", std::pair<double, double>(-2.65, 3.375)},
            {"bin5", std::pair<double, double>(-1.9, -3.375)},
            {"bin6", std::pair<double, double>(-1.9, -2.625)},
            {"bin7", std::pair<double, double>(-2.65, -2.625)},
            {"bin8", std::pair<double, double>(-2.65, -3.375)}};

    geometry_msgs::msg::Pose bin_pose;
    for (auto const &bin: bin_positions) {
        bin_pose.position.x = bin.second.first;
        bin_pose.position.y = bin.second.second;
        bin_pose.position.z = 0;
        bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

        AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
    }

    // Add assembly stations
    std::map<std::string, std::pair<double, double>> assembly_station_positions = {
            {"as1", std::pair<double, double>(-7.3, 3)},
            {"as2", std::pair<double, double>(-12.3, 3)},
            {"as3", std::pair<double, double>(-7.3, -3)},
            {"as4", std::pair<double, double>(-12.3, -3)},
    };

    geometry_msgs::msg::Pose assembly_station_pose;
    for (auto const &station: assembly_station_positions) {
        assembly_station_pose.position.x = station.second.first;
        assembly_station_pose.position.y = station.second.second;
        assembly_station_pose.position.z = 0;
        assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
    }

    // Add assembly briefcases
    std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
            {"as1_insert", std::pair<double, double>(-7.7, 3)},
            {"as2_insert", std::pair<double, double>(-12.7, 3)},
            {"as3_insert", std::pair<double, double>(-7.7, -3)},
            {"as4_insert", std::pair<double, double>(-12.7, -3)},
    };

    geometry_msgs::msg::Pose assembly_insert_pose;
    for (auto const &insert: assembly_insert_positions) {
        assembly_insert_pose.position.x = insert.second.first;
        assembly_insert_pose.position.y = insert.second.second;
        assembly_insert_pose.position.z = 1.011;
        assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
    }

    geometry_msgs::msg::Pose conveyor_pose;
    conveyor_pose.position.x = -0.6;
    conveyor_pose.position.y = 0;
    conveyor_pose.position.z = 0;
    conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

    geometry_msgs::msg::Pose kts1_table_pose;
    kts1_table_pose.position.x = -1.3;
    kts1_table_pose.position.y = -5.84;
    kts1_table_pose.position.z = 0;
    kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

    geometry_msgs::msg::Pose kts2_table_pose;
    kts2_table_pose.position.x = -1.3;
    kts2_table_pose.position.y = 5.84;
    kts2_table_pose.position.z = 0;
    kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto robot_commander = std::make_shared<RobotCommander>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot_commander);
    executor.spin();
//    std::thread([&executor]() { executor.spin(); }).detach();
//    rclcpp::spin(robot_commander);
    rclcpp::shutdown();
}