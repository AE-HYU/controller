#include "controller/controller.hpp"

#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <csignal>
#include <memory>

using ackermann_msgs::msg::AckermannDriveStamped;
using ae_hyu_msgs::msg::WpntArray;
using nav_msgs::msg::Odometry;
using rcl_interfaces::msg::FloatingPointRange;
using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::ParameterType;

namespace controller {

Controller::Controller()
: rclcpp::Node(
    "controller_manager",
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
{
    if (!initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize controller");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Controller initialized successfully");
}

bool Controller::initialize() {
    // Get essential parameters
    LUT_path_ = this->get_parameter("lookup_table_path").as_string();
    mode_ = this->get_parameter("mode").as_string();
    RCLCPP_INFO(this->get_logger(), "Using lookup table: %s", LUT_path_.c_str());

    // Initialize publishers
    drive_pub_ = this->create_publisher<AckermannDriveStamped>("/drive", 10);

    // Initialize MAP controller
    if (mode_ == "MAP") {
        RCLCPP_INFO(this->get_logger(), "Initializing MAP controller");
        init_map_controller();
    } else if (mode_ == "PP") {
        RCLCPP_INFO(this->get_logger(), "PP controller not implemented yet");
        return false;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s", mode_.c_str());
        return false;
    }

    // Initialize subscribers with generic topic names
    sub_track_length_ = this->create_subscription<WpntArray>("/planned_path", 10,
                        std::bind(&Controller::track_length_cb, this, std::placeholders::_1));
    sub_local_waypoints_ = this->create_subscription<WpntArray>("/planned_path", 10,
                          std::bind(&Controller::local_waypoint_cb, this, std::placeholders::_1));
    sub_car_state_ = this->create_subscription<Odometry>("/odom", 10,
                     std::bind(&Controller::car_state_cb, this, std::placeholders::_1));

    // Initialize parameter event handler with delayed initialization
    param_init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        [this]() {
            param_init_timer_->cancel();
            param_handler_ = std::make_unique<ParameterEventHandler>(
                this->shared_from_this(), rclcpp::ParametersQoS());
            callback_handle_ = param_handler_->add_parameter_event_callback(
                [this](const rcl_interfaces::msg::ParameterEvent & ev) {
                    this->on_parameter_event(ev);
                });
            RCLCPP_INFO(this->get_logger(), "ParameterEventHandler initialized");
        });

    // Initialize main control timer
    const double period = 1.0 / static_cast<double>(rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&Controller::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Controller ready");
    return true;
}

void Controller::init_map_controller() {
    const std::string l1_params_path = this->get_parameter("l1_params_path").as_string();
    declare_l1_dynamic_parameters_from_yaml(l1_params_path);

    // Initialize acceleration rolling buffer
    acc_now_ = Eigen::VectorXd::Zero(10);

    // Create MAP controller with logging callbacks
    auto info = [this](const std::string& s){ RCLCPP_INFO(this->get_logger(), "%s", s.c_str()); };
    auto warn = [this](const std::string& s){ RCLCPP_WARN(this->get_logger(), "%s", s.c_str()); };

    map_controller_ = std::make_unique<controller::MAP_Controller>(
        this->get_parameter("t_clip_min").as_double(),
        this->get_parameter("t_clip_max").as_double(),
        this->get_parameter("m_l1").as_double(),
        this->get_parameter("q_l1").as_double(),
        this->get_parameter("speed_lookahead").as_double(),
        this->get_parameter("lat_err_coeff").as_double(),
        this->get_parameter("acc_scaler_for_steer").as_double(),
        this->get_parameter("dec_scaler_for_steer").as_double(),
        this->get_parameter("start_scale_speed").as_double(),
        this->get_parameter("end_scale_speed").as_double(),
        this->get_parameter("downscale_factor").as_double(),
        this->get_parameter("speed_lookahead_for_steer").as_double(),
        static_cast<double>(rate_),
        LUT_path_,
        info, warn);
}

void Controller::declare_l1_dynamic_parameters_from_yaml(const std::string& yaml_path) {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["controller"] || !root["controller"]["ros__parameters"]) {
        throw std::runtime_error("Invalid l1_params YAML: missing controller.ros__parameters");
    }
    const YAML::Node params = root["controller"]["ros__parameters"];

    auto fp = [](double a, double b, double step){
        FloatingPointRange r;
        r.from_value=a;
        r.to_value=b;
        r.step=step;
        return r;
    };

    auto declare_double = [&](const std::string& name, double def, FloatingPointRange range) {
        ParameterDescriptor desc;
        desc.type = ParameterType::PARAMETER_DOUBLE;
        desc.floating_point_range = { range };
        (void)this->declare_parameter<double>(name, def, desc);
    };

    // Declare dynamic parameters
    declare_double("t_clip_min", params["t_clip_min"].as<double>(), fp(0.0, 1.5, 0.01));
    declare_double("t_clip_max", params["t_clip_max"].as<double>(), fp(0.0, 10.0, 0.01));
    declare_double("m_l1", params["m_l1"].as<double>(), fp(0.0, 1.0, 0.001));
    declare_double("q_l1", params["q_l1"].as<double>(), fp(-1.0, 1.0, 0.001));
    declare_double("speed_lookahead", params["speed_lookahead"].as<double>(), fp(0.0, 1.0, 0.01));
    declare_double("lat_err_coeff", params["lat_err_coeff"].as<double>(), fp(0.0, 1.0, 0.01));
    declare_double("acc_scaler_for_steer", params["acc_scaler_for_steer"].as<double>(), fp(0.0, 1.5, 0.01));
    declare_double("dec_scaler_for_steer", params["dec_scaler_for_steer"].as<double>(), fp(0.0, 1.5, 0.01));
    declare_double("start_scale_speed", params["start_scale_speed"].as<double>(), fp(0.0, 10.0, 0.01));
    declare_double("end_scale_speed", params["end_scale_speed"].as<double>(), fp(0.0, 10.0, 0.01));
    declare_double("downscale_factor", params["downscale_factor"].as<double>(), fp(0.0, 0.5, 0.01));
    declare_double("speed_lookahead_for_steer", params["speed_lookahead_for_steer"].as<double>(), fp(0.0, 0.2, 0.01));
}

void Controller::wait_for_messages() {
    RCLCPP_INFO(this->get_logger(), "Controller waiting for messages...");
    bool waypoint_array_received = false;
    bool car_state_received = false;

    while (rclcpp::ok() && (!waypoint_array_received || !car_state_received)) {
        rclcpp::spin_some(this->shared_from_this());

        if (waypoint_array_in_map_.size() > 0 && !waypoint_array_received) {
            RCLCPP_INFO(this->get_logger(), "Received waypoint array");
            waypoint_array_received = true;
        }
        if (speed_now_.has_value() && position_in_map_.has_value() &&
            position_in_map_frenet_.has_value() && !car_state_received) {
            RCLCPP_INFO(this->get_logger(), "Received car state messages");
            car_state_received = true;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    RCLCPP_INFO(this->get_logger(), "All required messages received. Continuing...");
}

void Controller::control_loop() {
    // Check if shutdown was requested
    if (shutdown_requested_.load()) {
        publish_stop_command();
        return;
    }

    if (mode_ != "MAP" || !map_controller_) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Unsupported mode or controller not ready");
        return;
    }

    if (!speed_now_.has_value() || !position_in_map_.has_value() ||
        !position_in_map_frenet_.has_value() || waypoint_array_in_map_.rows() == 0) {
        return;
    }

    auto [speed, steer] = map_cycle();
    RCLCPP_DEBUG(this->get_logger(), "Control output: steering=%.6f, speed=%.2f", steer, speed);

    // Create and publish drive message
    AckermannDriveStamped ack;
    const rclcpp::Time now = this->get_clock()->now();
    const int64_t ns = now.nanoseconds();
    ack.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    ack.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    ack.header.frame_id = "base_link";
    ack.drive.steering_angle = steer;
    ack.drive.speed = speed;

    drive_pub_->publish(ack);
}

std::pair<double,double> Controller::map_cycle() {
    auto res = map_controller_->main_loop(
        position_in_map_.value(),
        waypoint_array_in_map_,
        speed_now_.value(),
        Eigen::Vector2d(position_in_map_frenet_.value()(0), position_in_map_frenet_.value()(1)),
        acc_now_,
        track_length_.value_or(0.0));

    waypoint_safety_counter_ += 1;
    if (waypoint_safety_counter_ >= rate_ * 10) {  // 10 second timeout
        RCLCPP_WARN(this->get_logger(), "No fresh local waypoints. STOPPING!!");
        res.speed = 0.0;
        res.steering_angle = 0.0;
    }
    return {res.speed, res.steering_angle};
}

// Callback functions
void Controller::track_length_cb(const WpntArray::SharedPtr msg) {
    if (msg->wpnts.empty()) return;
    track_length_ = msg->wpnts.back().s_m;
}

void Controller::local_waypoint_cb(const WpntArray::SharedPtr msg) {
    const auto N = static_cast<int>(msg->wpnts.size());
    if (N <= 0) return;

    // Convert waypoint array to matrix format [x, y, speed, ratio, s, kappa, psi, ax]
    waypoint_array_in_map_.resize(N, 8);
    for (int i = 0; i < N; ++i) {
        const auto & w = msg->wpnts[i];
        const double ratio = (w.d_left + w.d_right) != 0.0
                             ? std::min(w.d_left, w.d_right) / (w.d_right + w.d_left)
                             : 0.0;
        waypoint_array_in_map_(i,0) = w.x_m;
        waypoint_array_in_map_(i,1) = w.y_m;
        waypoint_array_in_map_(i,2) = w.vx_mps;
        waypoint_array_in_map_(i,3) = ratio;
        waypoint_array_in_map_(i,4) = w.s_m;
        waypoint_array_in_map_(i,5) = w.kappa_radpm;
        waypoint_array_in_map_(i,6) = w.psi_rad;
        waypoint_array_in_map_(i,7) = w.ax_mps2;
    }
    waypoint_safety_counter_ = 0;
}

void Controller::car_state_cb(const Odometry::SharedPtr msg) {
    speed_now_ = msg->twist.twist.linear.x;

    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;

    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    Eigen::RowVector3d pose;
    pose << p.x, p.y, yaw;
    position_in_map_ = pose;

    // Extract frenet coordinates from the same odom message
    Eigen::Vector4d fr;
    fr << 0.0, 0.0, msg->twist.twist.linear.x, msg->twist.twist.linear.y;
    position_in_map_frenet_ = fr;
}

void Controller::on_parameter_event(const rcl_interfaces::msg::ParameterEvent & event) {
    if (event.node != "/controller_manager") return;
    if (!map_controller_) return;
    if (mode_ != "MAP") return;

    auto getp = [&](const char* name){ return this->get_parameter(name).as_double(); };

    // Update MAP controller parameters
    map_controller_->set_t_clip_min(getp("t_clip_min"));
    map_controller_->set_t_clip_max(getp("t_clip_max"));
    map_controller_->set_m_l1(getp("m_l1"));
    map_controller_->set_q_l1(getp("q_l1"));
    map_controller_->set_speed_lookahead(getp("speed_lookahead"));
    map_controller_->set_lat_err_coeff(getp("lat_err_coeff"));
    map_controller_->set_acc_scaler_for_steer(getp("acc_scaler_for_steer"));
    map_controller_->set_dec_scaler_for_steer(getp("dec_scaler_for_steer"));
    map_controller_->set_start_scale_speed(getp("start_scale_speed"));
    map_controller_->set_end_scale_speed(getp("end_scale_speed"));
    map_controller_->set_downscale_factor(getp("downscale_factor"));
    map_controller_->set_speed_lookahead_for_steer(getp("speed_lookahead_for_steer"));

    RCLCPP_INFO(this->get_logger(), "Updated parameters");
}

// Emergency stop functions
void Controller::publish_stop_command() {
    AckermannDriveStamped ack;

    const rclcpp::Time now = this->get_clock()->now();
    const int64_t ns = now.nanoseconds();
    ack.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    ack.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

    ack.header.frame_id = "base_link";
    ack.drive.speed = 0.0;
    ack.drive.steering_angle = 0.0;
    ack.drive.acceleration = -5.0;  // Emergency brake
    ack.drive.jerk = 0.0;
    ack.drive.steering_angle_velocity = 0.0;

    drive_pub_->publish(ack);

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Publishing stop command - Vehicle stopped for safety");
}

void Controller::shutdown_handler() {
    RCLCPP_WARN(this->get_logger(), "Shutdown signal received - stopping vehicle safely");
    shutdown_requested_.store(true);

    // Publish multiple stop commands to ensure vehicle stops
    for (int i = 0; i < 5; ++i) {
        publish_stop_command();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

} // namespace controller

// Global pointer to the node for signal handler
std::shared_ptr<controller::Controller> g_node = nullptr;

void signalHandler(int /* signum */) {
    if (g_node) {
        g_node->shutdown_handler();
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<controller::Controller>();
    g_node = node;  // Set global pointer for signal handler

    // Register signal handlers
    std::signal(SIGINT, signalHandler);   // Ctrl+C
    std::signal(SIGTERM, signalHandler);  // Termination signal

    RCLCPP_INFO(node->get_logger(), "Starting Controller Node with graceful shutdown");

    // Wait for required messages before starting control loop
    node->wait_for_messages();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}