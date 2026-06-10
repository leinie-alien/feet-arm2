#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <limits.h>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "arm2_task/common_units.hpp"
#include "arm2_task/kinematics_engine.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr int kDof = 5;
using JointArray = std::array<double, kDof>;

struct RobotSnapshot
{
    JointArray q{};
    JointArray dq{};
    JointArray tau{};
    bool valid{false};
};

struct DetectionResult
{
    bool success{false};
    int marker_count{0};
    int charuco_corner_count{0};
    double reprojection_error_px{std::numeric_limits<double>::infinity()};
    cv::Vec3d rvec{0.0, 0.0, 0.0};
    cv::Vec3d tvec{0.0, 0.0, 0.0};
    cv::Mat overlay_bgr;
};

struct RecordedSample
{
    int sample_index{0};
    int64_t stamp_ns{0};
    std::string image_path;
    std::string overlay_path;
    int charuco_corner_count{0};
    double reprojection_error_px{0.0};
    JointArray q{};
    JointArray dq{};
    JointArray tau{};
    Eigen::Matrix3d world_R_link4{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d world_t_link4{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d camera_R_board{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d camera_t_board{Eigen::Vector3d::Zero()};
};

bool is_absolute_path(const std::string & path)
{
    return !path.empty() && path.front() == '/';
}

std::string join_paths(const std::string & lhs, const std::string & rhs)
{
    if (lhs.empty()) {
        return rhs;
    }
    if (rhs.empty()) {
        return lhs;
    }
    if (lhs.back() == '/') {
        return lhs + rhs;
    }
    return lhs + "/" + rhs;
}

std::string current_working_directory()
{
    char buffer[PATH_MAX];
    if (::getcwd(buffer, sizeof(buffer)) == nullptr) {
        return ".";
    }
    return std::string(buffer);
}

std::string resolve_workspace_path(const std::string & path)
{
    if (is_absolute_path(path)) {
        return path;
    }

    const char * workspace_dir = std::getenv("ARM_WORKSPACE_DIR");
    if (workspace_dir != nullptr && std::strlen(workspace_dir) > 0U) {
        return join_paths(workspace_dir, path);
    }

    return join_paths(current_working_directory(), path);
}

std::string parent_directory(const std::string & path)
{
    const auto pos = path.find_last_of('/');
    if (pos == std::string::npos) {
        return std::string();
    }
    if (pos == 0U) {
        return "/";
    }
    return path.substr(0, pos);
}

bool ensure_directory_recursive(const std::string & directory)
{
    if (directory.empty() || directory == ".") {
        return true;
    }

    std::string current;
    std::size_t start = 0U;
    if (directory.front() == '/') {
        current = "/";
        start = 1U;
    }

    while (start <= directory.size()) {
        const auto separator = directory.find('/', start);
        const std::string part = directory.substr(start, separator - start);
        if (!part.empty()) {
            current = current == "/" ? current + part : current + "/" + part;
            if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
                return false;
            }
        }

        if (separator == std::string::npos) {
            break;
        }
        start = separator + 1U;
    }

    return true;
}

std::string format_joint_array(const JointArray & values)
{
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0U) {
            oss << ", ";
        }
        oss << std::fixed << std::setprecision(3) << values[i];
    }
    oss << "]";
    return oss.str();
}

Eigen::VectorXd to_eigen(const JointArray & values)
{
    Eigen::VectorXd out(kDof);
    for (int i = 0; i < kDof; ++i) {
        out[i] = values[static_cast<std::size_t>(i)];
    }
    return out;
}

cv::Mat eigen_rotation_to_cv(const Eigen::Matrix3d & rotation)
{
    cv::Mat out(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.at<double>(r, c) = rotation(r, c);
        }
    }
    return out;
}

cv::Mat eigen_vector_to_cv(const Eigen::Vector3d & vector)
{
    cv::Mat out(3, 1, CV_64F);
    for (int i = 0; i < 3; ++i) {
        out.at<double>(i, 0) = vector(i);
    }
    return out;
}

Eigen::Matrix3d cv_rotation_to_eigen(const cv::Mat & rotation)
{
    Eigen::Matrix3d out = Eigen::Matrix3d::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out(r, c) = rotation.at<double>(r, c);
        }
    }
    return out;
}

Eigen::Vector3d cv_vector_to_eigen(const cv::Vec3d & vector)
{
    return Eigen::Vector3d(vector[0], vector[1], vector[2]);
}

std::array<double, 4> quaternion_xyzw_from_rotation(const Eigen::Matrix3d & rotation)
{
    const Eigen::Quaterniond quat(rotation);
    return {quat.x(), quat.y(), quat.z(), quat.w()};
}

std::array<double, 4> quaternion_xyzw_from_cv_rvec(const cv::Vec3d & rvec)
{
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    return quaternion_xyzw_from_rotation(cv_rotation_to_eigen(rotation));
}

std::string make_relative_path(const std::string & root, const std::string & path)
{
    if (path.rfind(root, 0) == 0) {
        std::string rel = path.substr(root.size());
        if (!rel.empty() && rel.front() == '/') {
            rel.erase(rel.begin());
        }
        return rel;
    }
    return path;
}

int parse_camera_index(const std::string & camera_source, bool * parsed)
{
    char * end = nullptr;
    errno = 0;
    const long value = std::strtol(camera_source.c_str(), &end, 10);
    const bool ok = errno == 0 && end != nullptr && *end == '\0';
    if (parsed != nullptr) {
        *parsed = ok;
    }
    return ok ? static_cast<int>(value) : 0;
}

int dictionary_from_name(const std::string & name)
{
    const std::vector<std::pair<std::string, int>> entries = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
        {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
        {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
        {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
        {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
    };
    for (const auto & entry : entries) {
        if (entry.first == name) {
            return entry.second;
        }
    }
    throw std::runtime_error("Unsupported ArUco dictionary: " + name);
}

}  // namespace

class HandeyeCharucoCaptureNode : public rclcpp::Node
{
public:
    HandeyeCharucoCaptureNode()
        : Node("handeye_charuco_capture_node")
    {
        std::string share_dir = ament_index_cpp::get_package_share_directory("arm2_task");
        std::string rel_urdf_path = this->declare_parameter("urdf_path", std::string("urdf/arm2.urdf"));
        const std::string urdf_path = share_dir + "/" + rel_urdf_path;

        auto l1 = this->declare_parameter("robot_geometry.l1", 0.0845);
        auto l2 = this->declare_parameter("robot_geometry.l2", 0.350005);
        auto l3 = this->declare_parameter("robot_geometry.l3", 0.243441);
        auto l4 = this->declare_parameter("robot_geometry.l4", 0.046);
        arm2_task::RobotGeometry geometry{};
        geometry.L1 = l1;
        geometry.L2 = l2;
        geometry.L3 = l3;
        geometry.L4 = l4;
        kin_engine_ = std::make_unique<arm2_task::KinematicsEngine>(urdf_path, geometry);

        output_dir_ = resolve_workspace_path(this->declare_parameter(
            "handeye_charuco_capture.output_dir",
            std::string("recordings/handeye_charuco_capture")));
        image_dir_ = join_paths(output_dir_, "images");
        overlay_dir_ = join_paths(output_dir_, "overlays");
        dataset_file_ = join_paths(output_dir_, "dataset.yml");

        state_topic_ = this->declare_parameter(
            "handeye_charuco_capture.state_topic",
            std::string("/arm2/_lowState/joint"));
        ready_topic_ = this->declare_parameter(
            "handeye_charuco_capture.ready_topic",
            std::string("/robot_driver/ready"));
        require_ready_ = this->declare_parameter("handeye_charuco_capture.require_ready", true);
        camera_source_ = this->declare_parameter(
            "handeye_charuco_capture.camera_source",
            std::string("0"));
        capture_width_ = this->declare_parameter("handeye_charuco_capture.capture_width", 1280);
        capture_height_ = this->declare_parameter("handeye_charuco_capture.capture_height", 720);
        preview_rate_hz_ = this->declare_parameter("handeye_charuco_capture.preview_rate_hz", 8.0);
        preview_enabled_ = this->declare_parameter("handeye_charuco_capture.preview_enabled", true);
        preview_scale_ = this->declare_parameter("handeye_charuco_capture.preview_scale", 1.0);
        preview_window_name_ = this->declare_parameter(
            "handeye_charuco_capture.preview_window_name",
            std::string("Handeye ChArUco Preview"));
        overwrite_output_ = this->declare_parameter("handeye_charuco_capture.overwrite_output", true);

        board_squares_x_ = this->declare_parameter("handeye_charuco_capture.board_squares_x", 7);
        board_squares_y_ = this->declare_parameter("handeye_charuco_capture.board_squares_y", 5);
        square_length_m_ = this->declare_parameter("handeye_charuco_capture.square_length_m", 0.03);
        marker_length_m_ = this->declare_parameter("handeye_charuco_capture.marker_length_m", 0.022);
        dictionary_name_ = this->declare_parameter(
            "handeye_charuco_capture.dictionary_name",
            std::string("DICT_4X4_50"));
        min_charuco_corners_ = this->declare_parameter(
            "handeye_charuco_capture.min_charuco_corners",
            8);

        intrinsics_file_ = this->declare_parameter(
            "handeye_charuco_capture.intrinsics_file",
            std::string());
        const auto camera_matrix_values =
            this->declare_double_array_parameter("handeye_charuco_capture.camera_matrix");
        const auto dist_coeff_values =
            this->declare_double_array_parameter("handeye_charuco_capture.dist_coeffs");

        if (!ensure_directory_recursive(output_dir_) ||
            !ensure_directory_recursive(image_dir_) ||
            !ensure_directory_recursive(overlay_dir_)) {
            throw std::runtime_error("Failed to create output directories under " + output_dir_);
        }
        if (!overwrite_output_ && ::access(dataset_file_.c_str(), F_OK) == 0) {
            throw std::runtime_error("Dataset file already exists: " + dataset_file_);
        }

        load_intrinsics(camera_matrix_values, dist_coeff_values);
        prepare_charuco_board();
        open_camera();
        setup_subscriptions();
        setup_keyboard();
        setup_preview_window();

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(0.5, preview_rate_hz_)));
        timer_ = this->create_wall_timer(period, std::bind(&HandeyeCharucoCaptureNode::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Handeye ChArUco capture node started. output_dir=%s camera_source=%s board=%dx%d square=%.4fm marker=%.4fm dict=%s",
            output_dir_.c_str(),
            camera_source_.c_str(),
            board_squares_x_,
            board_squares_y_,
            square_length_m_,
            marker_length_m_,
            dictionary_name_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "Keyboard: c=capture, v=preview, u=undo, s=save, h=help, q=quit");
    }

    ~HandeyeCharucoCaptureNode() override
    {
        shutdown_requested_.store(true);
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        close_preview_window();
        restore_terminal();
        save_dataset("final");
    }

private:
    std::vector<double> declare_double_array_parameter(const std::string & name)
    {
        try {
            return this->declare_parameter(name, std::vector<double>{});
        } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
            this->undeclare_parameter(name);
            this->declare_parameter(name, std::vector<double>{});
            return std::vector<double>{};
        }
    }

    void load_intrinsics(
        const std::vector<double> & camera_matrix_values,
        const std::vector<double> & dist_coeff_values)
    {
        if (!intrinsics_file_.empty()) {
            const std::string resolved = resolve_workspace_path(intrinsics_file_);
            cv::FileStorage fs(resolved, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                throw std::runtime_error("Failed to open intrinsics file: " + resolved);
            }
            fs["camera_matrix"] >> camera_matrix_;
            fs["dist_coeffs"] >> dist_coeffs_;
            if (camera_matrix_.empty()) {
                fs["K"] >> camera_matrix_;
            }
            if (dist_coeffs_.empty()) {
                fs["D"] >> dist_coeffs_;
            }
            intrinsics_file_ = resolved;
        } else {
            if (camera_matrix_values.size() != 9U) {
                throw std::runtime_error(
                    "handeye_charuco_capture.camera_matrix must have 9 values when intrinsics_file is unset");
            }
            camera_matrix_ = cv::Mat(3, 3, CV_64F);
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    camera_matrix_.at<double>(r, c) =
                        camera_matrix_values[static_cast<std::size_t>(3 * r + c)];
                }
            }
            dist_coeffs_ = cv::Mat(static_cast<int>(dist_coeff_values.size()), 1, CV_64F);
            for (std::size_t i = 0; i < dist_coeff_values.size(); ++i) {
                dist_coeffs_.at<double>(static_cast<int>(i), 0) = dist_coeff_values[i];
            }
        }

        if (camera_matrix_.empty() || camera_matrix_.rows != 3 || camera_matrix_.cols != 3) {
            throw std::runtime_error("Invalid camera_matrix for handeye_charuco_capture");
        }
        if (dist_coeffs_.empty()) {
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        }
        camera_matrix_.convertTo(camera_matrix_, CV_64F);
        dist_coeffs_.convertTo(dist_coeffs_, CV_64F);
    }

    void prepare_charuco_board()
    {
        if (board_squares_x_ < 2 || board_squares_y_ < 2) {
            throw std::runtime_error("ChArUco board dimensions must be >= 2");
        }
        if (square_length_m_ <= 0.0 || marker_length_m_ <= 0.0 ||
            marker_length_m_ >= square_length_m_) {
            throw std::runtime_error("Invalid ChArUco square/marker lengths");
        }

        dictionary_id_ = dictionary_from_name(dictionary_name_);
        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id_);
        board_ = cv::aruco::CharucoBoard::create(
            board_squares_x_,
            board_squares_y_,
            static_cast<float>(square_length_m_),
            static_cast<float>(marker_length_m_),
            dictionary_);
    }

    void open_camera()
    {
        bool parsed_index = false;
        const int camera_index = parse_camera_index(camera_source_, &parsed_index);
        const bool opened = parsed_index ? capture_.open(camera_index, cv::CAP_V4L2) : capture_.open(camera_source_, cv::CAP_V4L2);
        if (!opened) {
            throw std::runtime_error("Failed to open camera source: " + camera_source_);
        }
        if (capture_width_ > 0) {
            capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(capture_width_));
        }
        if (capture_height_ > 0) {
            capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(capture_height_));
        }
    }

    void setup_preview_window()
    {
        if (!preview_enabled_) {
            return;
        }
        const char * display_env = std::getenv("DISPLAY");
        if (display_env == nullptr || std::strlen(display_env) == 0U) {
            preview_enabled_ = false;
            RCLCPP_WARN(this->get_logger(), "Preview window disabled: DISPLAY is not set.");
            return;
        }
        try {
            cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
            preview_window_initialized_ = true;
        } catch (const cv::Exception & ex) {
            preview_enabled_ = false;
            preview_window_initialized_ = false;
            RCLCPP_WARN(this->get_logger(), "Preview window disabled: %s", ex.what());
        }
    }

    void close_preview_window()
    {
        if (!preview_window_initialized_) {
            return;
        }
        try {
            cv::destroyWindow(preview_window_name_);
        } catch (const cv::Exception &) {
        }
        preview_window_initialized_ = false;
    }

    void setup_subscriptions()
    {
        auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
            state_topic_,
            state_qos,
            std::bind(&HandeyeCharucoCaptureNode::state_callback, this, std::placeholders::_1));
        ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            ready_topic_,
            ready_qos,
            std::bind(&HandeyeCharucoCaptureNode::ready_callback, this, std::placeholders::_1));
    }

    void setup_keyboard()
    {
        stdin_is_tty_ = ::isatty(STDIN_FILENO);
        if (!stdin_is_tty_) {
            RCLCPP_WARN(this->get_logger(), "STDIN is not a TTY. Keyboard capture is disabled.");
            return;
        }
        if (::tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to read terminal settings; keyboard capture disabled.");
            stdin_is_tty_ = false;
            return;
        }
        struct termios raw = original_termios_;
        raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to switch terminal to raw mode; keyboard capture disabled.");
            stdin_is_tty_ = false;
            return;
        }
        terminal_raw_enabled_ = true;
        keyboard_thread_ = std::thread(&HandeyeCharucoCaptureNode::keyboard_loop, this);
    }

    void restore_terminal()
    {
        if (terminal_raw_enabled_) {
            ::tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
            terminal_raw_enabled_ = false;
        }
    }

    void keyboard_loop()
    {
        while (!shutdown_requested_.load()) {
            char ch = '\0';
            const ssize_t nread = ::read(STDIN_FILENO, &ch, 1);
            if (nread <= 0) {
                if (shutdown_requested_.load()) {
                    break;
                }
                std::this_thread::sleep_for(20ms);
                continue;
            }
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                pending_commands_.push_back(ch);
            }
            if (ch == 'q') {
                break;
            }
        }
    }

    void state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        if (!msg || msg->motor_state.size() < static_cast<std::size_t>(kDof)) {
            return;
        }
        RobotSnapshot snapshot;
        for (int i = 0; i < kDof; ++i) {
            if (!msg->motor_state[static_cast<std::size_t>(i)].valid) {
                return;
            }
            snapshot.q[static_cast<std::size_t>(i)] = msg->motor_state[static_cast<std::size_t>(i)].q;
            snapshot.dq[static_cast<std::size_t>(i)] = msg->motor_state[static_cast<std::size_t>(i)].dq;
            snapshot.tau[static_cast<std::size_t>(i)] = msg->motor_state[static_cast<std::size_t>(i)].tau_est;
        }
        snapshot.valid = true;
        {
            std::lock_guard<std::mutex> lock(robot_mutex_);
            latest_robot_ = snapshot;
        }
    }

    void ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg) {
            driver_ready_ = msg->data;
        }
    }

    void timer_callback()
    {
        cv::Mat frame;
        if (!capture_.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to read a frame from camera source %s",
                camera_source_.c_str());
            return;
        }
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_ = frame.clone();
        }

        const DetectionResult live_detection = detect_charuco_pose(frame);
        render_preview(frame, live_detection);

        std::vector<char> commands;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            commands.swap(pending_commands_);
        }
        for (const char cmd : commands) {
            handle_command(cmd);
        }
    }

    void handle_command(char cmd)
    {
        switch (cmd) {
            case 'c':
                perform_capture(true);
                break;
            case 'v':
                perform_capture(false);
                break;
            case 'u':
                undo_last_sample();
                break;
            case 's':
                save_dataset("manual");
                break;
            case 'h':
                RCLCPP_INFO(
                    this->get_logger(),
                    "Keyboard: c=capture, v=preview, u=undo, s=save, h=help, q=quit");
                break;
            case 'q':
                RCLCPP_INFO(this->get_logger(), "Quit requested from keyboard.");
                save_dataset("quit");
                shutdown_requested_.store(true);
                rclcpp::shutdown();
                break;
            default:
                break;
        }
    }

    bool get_latest_snapshot(RobotSnapshot * snapshot)
    {
        std::lock_guard<std::mutex> lock(robot_mutex_);
        if (!latest_robot_.valid) {
            return false;
        }
        if (snapshot != nullptr) {
            *snapshot = latest_robot_;
        }
        return true;
    }

    bool get_latest_frame(cv::Mat * frame)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (latest_frame_.empty()) {
            return false;
        }
        if (frame != nullptr) {
            *frame = latest_frame_.clone();
        }
        return true;
    }

    DetectionResult detect_charuco_pose(const cv::Mat & frame_bgr)
    {
        DetectionResult result;
        result.overlay_bgr = frame_bgr.clone();

        cv::Mat gray;
        cv::cvtColor(frame_bgr, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<std::vector<cv::Point2f>> rejected;
        cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids);
        result.marker_count = static_cast<int>(marker_ids.size());
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(result.overlay_bgr, marker_corners, marker_ids);
        }

        cv::Mat charuco_corners;
        cv::Mat charuco_ids;
        if (!marker_ids.empty()) {
            cv::aruco::interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                board_,
                charuco_corners,
                charuco_ids,
                camera_matrix_,
                dist_coeffs_);
        }

        result.charuco_corner_count = charuco_ids.rows;
        if (result.charuco_corner_count > 0) {
            cv::aruco::drawDetectedCornersCharuco(result.overlay_bgr, charuco_corners, charuco_ids);
        }
        if (result.charuco_corner_count < min_charuco_corners_) {
            return result;
        }

        cv::Vec3d rvec;
        cv::Vec3d tvec;
        const bool pose_ok = cv::aruco::estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            board_,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec);
        if (!pose_ok) {
            return result;
        }

        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> image_points;
        object_points.reserve(static_cast<std::size_t>(charuco_ids.rows));
        image_points.reserve(static_cast<std::size_t>(charuco_ids.rows));
        for (int i = 0; i < charuco_ids.rows; ++i) {
            const int id = charuco_ids.at<int>(i, 0);
            if (id < 0 || id >= static_cast<int>(board_->chessboardCorners.size())) {
                continue;
            }
            object_points.push_back(board_->chessboardCorners[static_cast<std::size_t>(id)]);
            image_points.push_back(charuco_corners.at<cv::Point2f>(i, 0));
        }
        if (!object_points.empty()) {
            std::vector<cv::Point2f> reproj;
            cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, reproj);
            double accum = 0.0;
            for (std::size_t i = 0; i < reproj.size(); ++i) {
                accum += cv::norm(reproj[i] - image_points[i]);
            }
            result.reprojection_error_px = accum / static_cast<double>(reproj.size());
        }

        cv::drawFrameAxes(
            result.overlay_bgr,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec,
            static_cast<float>(square_length_m_ * 1.5));

        result.success = std::isfinite(result.reprojection_error_px);
        result.rvec = rvec;
        result.tvec = tvec;
        return result;
    }

    void draw_status_panel(cv::Mat * image, const DetectionResult & detection) const
    {
        if (image == nullptr || image->empty()) {
            return;
        }
        cv::Mat & canvas = *image;
        const int panel_height = 96;
        cv::rectangle(
            canvas,
            cv::Rect(0, 0, canvas.cols, std::min(panel_height, canvas.rows)),
            cv::Scalar(0, 0, 0),
            cv::FILLED);

        const std::string line1 =
            "markers=" + std::to_string(detection.marker_count) +
            " corners=" + std::to_string(detection.charuco_corner_count) +
            " samples=" + std::to_string(samples_.size()) +
            " failed=" + std::to_string(failed_capture_count_);
        std::ostringstream line2_builder;
        line2_builder << std::fixed << std::setprecision(3);
        line2_builder << "reproj=";
        if (std::isfinite(detection.reprojection_error_px)) {
            line2_builder << detection.reprojection_error_px << " px";
        } else {
            line2_builder << "n/a";
        }
        line2_builder << " status=" << (detection.success ? "POSE_OK" : "SEARCHING");
        const std::string line2 = line2_builder.str();
        const std::string line3 = "keys: c capture | v verify | u undo | s save | q quit";

        cv::putText(canvas, line1, cv::Point(12, 26), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2);
        cv::putText(canvas, line2, cv::Point(12, 52), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);
        cv::putText(canvas, line3, cv::Point(12, 78), cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(255, 255, 255), 1);
    }

    void render_preview(const cv::Mat & frame, const DetectionResult & detection)
    {
        if (!preview_enabled_ || !preview_window_initialized_) {
            return;
        }

        cv::Mat preview = detection.overlay_bgr.empty() ? frame.clone() : detection.overlay_bgr.clone();
        draw_status_panel(&preview, detection);

        if (std::abs(preview_scale_ - 1.0) > 1e-6 && preview_scale_ > 0.0) {
            cv::Mat scaled;
            cv::resize(preview, scaled, cv::Size(), preview_scale_, preview_scale_, cv::INTER_LINEAR);
            preview = scaled;
        }

        try {
            cv::imshow(preview_window_name_, preview);
            const int key = cv::waitKey(1);
            if (key >= 0) {
                const char ch = static_cast<char>(std::tolower(static_cast<unsigned char>(key & 0xFF)));
                switch (ch) {
                    case 'c':
                    case 'v':
                    case 'u':
                    case 's':
                    case 'h':
                    case 'q': {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        pending_commands_.push_back(ch);
                        break;
                    }
                    default:
                        break;
                }
            }
        } catch (const cv::Exception & ex) {
            preview_enabled_ = false;
            preview_window_initialized_ = false;
            RCLCPP_WARN(this->get_logger(), "Preview window disabled after OpenCV error: %s", ex.what());
        }
    }

    void perform_capture(bool save_sample)
    {
        if (require_ready_ && !driver_ready_) {
            RCLCPP_WARN(this->get_logger(), "Driver is not ready; capture ignored.");
            return;
        }

        RobotSnapshot snapshot;
        if (!get_latest_snapshot(&snapshot)) {
            RCLCPP_WARN(this->get_logger(), "No valid robot state received yet.");
            return;
        }

        cv::Mat frame;
        if (!get_latest_frame(&frame)) {
            RCLCPP_WARN(this->get_logger(), "No camera frame available yet.");
            return;
        }

        DetectionResult detection = detect_charuco_pose(frame);
        if (!detection.success) {
            if (save_sample) {
                save_failed_capture(frame, detection);
            }
            RCLCPP_WARN(
                this->get_logger(),
                "ChArUco detection failed. corners=%d min_required=%d",
                detection.charuco_corner_count,
                min_charuco_corners_);
            return;
        }

        const Eigen::VectorXd q_eigen = to_eigen(snapshot.q);
        const pinocchio::SE3 world_T_link4 = kin_engine_->forwardKinematics(q_eigen);
        const Eigen::Matrix3d world_R_link4 = world_T_link4.rotation();
        const Eigen::Vector3d world_t_link4 = world_T_link4.translation();

        RCLCPP_INFO(
            this->get_logger(),
            "%s pose ok: corners=%d reproj=%.3fpx q=%s world_t_link4=[%.3f, %.3f, %.3f]",
            save_sample ? "Capture" : "Preview",
            detection.charuco_corner_count,
            detection.reprojection_error_px,
            format_joint_array(snapshot.q).c_str(),
            world_t_link4.x(),
            world_t_link4.y(),
            world_t_link4.z());

        if (!save_sample) {
            return;
        }

        RecordedSample sample;
        sample.sample_index = next_sample_index_++;
        sample.stamp_ns = this->now().nanoseconds();
        sample.charuco_corner_count = detection.charuco_corner_count;
        sample.reprojection_error_px = detection.reprojection_error_px;
        sample.q = snapshot.q;
        sample.dq = snapshot.dq;
        sample.tau = snapshot.tau;
        sample.world_R_link4 = world_R_link4;
        sample.world_t_link4 = world_t_link4;

        cv::Mat camera_rotation;
        cv::Rodrigues(detection.rvec, camera_rotation);
        sample.camera_R_board = cv_rotation_to_eigen(camera_rotation);
        sample.camera_t_board = cv_vector_to_eigen(detection.tvec);

        std::ostringstream stem_builder;
        stem_builder << "sample_" << std::setw(4) << std::setfill('0') << sample.sample_index;
        const std::string stem = stem_builder.str();
        const std::string image_file = join_paths(image_dir_, stem + ".png");
        const std::string overlay_file = join_paths(overlay_dir_, stem + "_overlay.png");

        if (!cv::imwrite(image_file, frame)) {
            throw std::runtime_error("Failed to save image: " + image_file);
        }
        if (!cv::imwrite(overlay_file, detection.overlay_bgr)) {
            throw std::runtime_error("Failed to save overlay image: " + overlay_file);
        }

        sample.image_path = make_relative_path(output_dir_, image_file);
        sample.overlay_path = make_relative_path(output_dir_, overlay_file);

        {
            std::lock_guard<std::mutex> lock(sample_mutex_);
            samples_.push_back(sample);
        }
        save_dataset("capture");
        RCLCPP_INFO(
            this->get_logger(),
            "Saved sample %d to %s (total=%zu)",
            sample.sample_index,
            image_file.c_str(),
            samples_.size());
    }

    void save_failed_capture(const cv::Mat & frame, const DetectionResult & detection)
    {
        const int debug_index = failed_capture_count_++;
        std::ostringstream stem_builder;
        stem_builder << "failed_" << std::setw(4) << std::setfill('0') << debug_index;
        const std::string stem = stem_builder.str();
        const std::string image_file = join_paths(image_dir_, stem + ".png");
        const std::string overlay_file = join_paths(overlay_dir_, stem + "_overlay.png");

        if (!cv::imwrite(image_file, frame)) {
            throw std::runtime_error("Failed to save failed-capture image: " + image_file);
        }
        if (!cv::imwrite(overlay_file, detection.overlay_bgr)) {
            throw std::runtime_error("Failed to save failed-capture overlay: " + overlay_file);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Saved failed capture for debugging: image=%s overlay=%s",
            image_file.c_str(),
            overlay_file.c_str());
    }

    void undo_last_sample()
    {
        RecordedSample removed;
        bool has_removed = false;
        {
            std::lock_guard<std::mutex> lock(sample_mutex_);
            if (!samples_.empty()) {
                removed = samples_.back();
                samples_.pop_back();
                has_removed = true;
                next_sample_index_ = std::max(next_sample_index_, removed.sample_index + 1);
            }
        }
        if (!has_removed) {
            RCLCPP_INFO(this->get_logger(), "Undo ignored: no saved samples.");
            return;
        }

        const std::string image_file = join_paths(output_dir_, removed.image_path);
        const std::string overlay_file = join_paths(output_dir_, removed.overlay_path);
        ::unlink(image_file.c_str());
        ::unlink(overlay_file.c_str());
        save_dataset("undo");
        RCLCPP_INFO(
            this->get_logger(),
            "Removed sample %d (remaining=%zu)",
            removed.sample_index,
            samples_.size());
    }

    void save_dataset(const std::string & reason)
    {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        cv::FileStorage fs(dataset_file_, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            throw std::runtime_error("Failed to open dataset file for write: " + dataset_file_);
        }

        fs << "metadata" << "{";
        fs << "schema" << "arm2_task.handeye_charuco_capture.v1";
        fs << "reason" << reason;
        fs << "camera_source" << camera_source_;
        fs << "intrinsics_file" << intrinsics_file_;
        fs << "camera_matrix" << camera_matrix_;
        fs << "dist_coeffs" << dist_coeffs_;
        fs << "board_squares_x" << board_squares_x_;
        fs << "board_squares_y" << board_squares_y_;
        fs << "square_length_m" << square_length_m_;
        fs << "marker_length_m" << marker_length_m_;
        fs << "dictionary_name" << dictionary_name_;
        fs << "min_charuco_corners" << min_charuco_corners_;
        fs << "sample_count" << static_cast<int>(samples_.size());
        fs << "}";

        fs << "samples" << "[";
        for (const auto & sample : samples_) {
            const Eigen::Matrix3d link4_R_world = sample.world_R_link4.transpose();
            const Eigen::Vector3d link4_t_world = -(link4_R_world * sample.world_t_link4);
            const Eigen::Matrix3d board_R_camera = sample.camera_R_board.transpose();
            const Eigen::Vector3d board_t_camera = -(board_R_camera * sample.camera_t_board);
            const auto world_q_link4 = quaternion_xyzw_from_rotation(sample.world_R_link4);
            const auto link4_q_world = quaternion_xyzw_from_rotation(link4_R_world);
            const auto camera_q_board = quaternion_xyzw_from_rotation(sample.camera_R_board);
            const auto board_q_camera = quaternion_xyzw_from_rotation(board_R_camera);

            fs << "{";
            fs << "sample_index" << sample.sample_index;
            fs << "stamp_ns" << std::to_string(sample.stamp_ns);
            fs << "image_path" << sample.image_path;
            fs << "overlay_path" << sample.overlay_path;
            fs << "charuco_corner_count" << sample.charuco_corner_count;
            fs << "reprojection_error_px" << sample.reprojection_error_px;
            fs << "q" << std::vector<double>(sample.q.begin(), sample.q.end());
            fs << "dq" << std::vector<double>(sample.dq.begin(), sample.dq.end());
            fs << "tau" << std::vector<double>(sample.tau.begin(), sample.tau.end());

            fs << "world_T_link4" << "{";
            fs << "rotation" << eigen_rotation_to_cv(sample.world_R_link4);
            fs << "translation" << eigen_vector_to_cv(sample.world_t_link4);
            fs << "quaternion_xyzw" << std::vector<double>(
                world_q_link4.begin(),
                world_q_link4.end());
            fs << "}";

            fs << "link4_T_world" << "{";
            fs << "rotation" << eigen_rotation_to_cv(link4_R_world);
            fs << "translation" << eigen_vector_to_cv(link4_t_world);
            fs << "quaternion_xyzw" << std::vector<double>(
                link4_q_world.begin(),
                link4_q_world.end());
            fs << "}";

            fs << "camera_T_board" << "{";
            fs << "rotation" << eigen_rotation_to_cv(sample.camera_R_board);
            fs << "translation" << eigen_vector_to_cv(sample.camera_t_board);
            fs << "quaternion_xyzw" << std::vector<double>(
                camera_q_board.begin(),
                camera_q_board.end());
            fs << "}";

            fs << "board_T_camera" << "{";
            fs << "rotation" << eigen_rotation_to_cv(board_R_camera);
            fs << "translation" << eigen_vector_to_cv(board_t_camera);
            fs << "quaternion_xyzw" << std::vector<double>(
                board_q_camera.begin(),
                board_q_camera.end());
            fs << "}";

            fs << "}";
        }
        fs << "]";
        fs.release();

        RCLCPP_INFO(
            this->get_logger(),
            "Dataset %s saved with %zu samples -> %s",
            reason.c_str(),
            samples_.size(),
            dataset_file_.c_str());
    }

    std::unique_ptr<arm2_task::KinematicsEngine> kin_engine_;

    std::string output_dir_;
    std::string image_dir_;
    std::string overlay_dir_;
    std::string dataset_file_;
    std::string state_topic_;
    std::string ready_topic_;
    bool require_ready_{true};
    std::string camera_source_;
    int capture_width_{1280};
    int capture_height_{720};
    double preview_rate_hz_{8.0};
    bool preview_enabled_{true};
    double preview_scale_{1.0};
    std::string preview_window_name_;
    bool preview_window_initialized_{false};
    bool overwrite_output_{true};

    int board_squares_x_{7};
    int board_squares_y_{5};
    double square_length_m_{0.03};
    double marker_length_m_{0.022};
    std::string dictionary_name_;
    int dictionary_id_{cv::aruco::DICT_4X4_50};
    int min_charuco_corners_{8};

    std::string intrinsics_file_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::CharucoBoard> board_;

    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::VideoCapture capture_;
    cv::Mat latest_frame_;
    std::mutex frame_mutex_;

    RobotSnapshot latest_robot_;
    std::mutex robot_mutex_;
    bool driver_ready_{false};

    std::vector<RecordedSample> samples_;
    std::mutex sample_mutex_;
    int next_sample_index_{0};
    int failed_capture_count_{0};

    std::vector<char> pending_commands_;
    std::mutex command_mutex_;
    std::thread keyboard_thread_;
    std::atomic<bool> shutdown_requested_{false};
    bool stdin_is_tty_{false};
    bool terminal_raw_enabled_{false};
    struct termios original_termios_{};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<HandeyeCharucoCaptureNode>());
    } catch (const std::exception & ex) {
        std::fprintf(stderr, "handeye_charuco_capture_node failed: %s\n", ex.what());
        return 1;
    }
    return 0;
}
