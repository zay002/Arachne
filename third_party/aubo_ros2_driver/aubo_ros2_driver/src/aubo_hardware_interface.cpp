#include "aubo_hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>
#include <ctime>
#include <fstream>
namespace aubo_driver {

AuboHardwareInterface::~AuboHardwareInterface()
{
    stopServoMode();
}
bool AuboHardwareInterface::OnActive()
{
    const std::string robot_ip_ = info_.hardware_parameters["robot_ip"];
    rpc_client_ = std::make_shared<RpcClient>();

    rpc_client_->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    rpc_client_->connect(robot_ip_, 30004);
    // 接口调用: 登录
    rpc_client_->login("aubo", "123456");

    rtde_client_ = std::make_shared<RtdeClient>();
    // 接口调用: 连接到 RTDE 服务
    rtde_client_->connect(robot_ip_, 30010);
    // 接口调用: 登录
    rtde_client_->login("aubo", "123456");
    int topic = rtde_client_->setTopic(false, { "R1_message" }, 200, 0);
    if (topic < 0) {
        std::cout << "Set topic fail!" << std::endl;
    }
    rtde_client_->subscribe(topic, [](InputParser &parser) {
        arcs::common_interface::RobotMsgVector msgs;
        msgs = parser.popRobotMsgVector();
        for (size_t i = 0; i < msgs.size(); i++) {
            auto &msg = msgs[i];
        }
    });
    robot_name_ = rpc_client_->getRobotNames().front();

    rpc_client_->getRobotInterface(robot_name_)
    ->getRobotConfig()
    ->setHardwareCustomParameters("[joint_func] \n vff_enable = false\n");

    std::cout << "vff_enable = false" << std::endl;

    // 设置rtde输入
    setInput(rtde_client_);

    // 配置输出
    configSubscribe(rtde_client_);

    return true;
}

hardware_interface::CallbackReturn AuboHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &system_info)
{
    if (hardware_interface::SystemInterface::on_init(system_info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = system_info;
    initialized_ = false;

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        // RRBotSystemPositionOnly has exactly one state and command interface
        // on each joint
        if (joint.command_interfaces.size() != 2) {
            RCLCPP_FATAL(
                rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                         "Joint '%s' has %zu state interface. 1 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                         "Joint '%s' have %s state interface. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn AuboHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("AuboHardwareInterface"),
                "Starting ...please wait...");
    OnActive();
    for (int i = 0; i < 200 && !actual_q_received_; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!actual_q_received_) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "No RTDE actual_q received during activation; refusing to initialize commands.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    readActualQ();

    if (!initialized_) {
        // Initialize the command interfaces from the measured joint state. This
        // prevents an all-zero command from being sent during real-arm startup.
        aubo_position_commands_ = actual_q_copy_;
        aubo_velocity_commands_.fill(0.0);
        initialized_ = true;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AuboHardwareInterface"),
                           "Initialized hold command from actual_q: ["
                               << actual_q_copy_[0] << ", " << actual_q_copy_[1] << ", "
                               << actual_q_copy_[2] << ", " << actual_q_copy_[3] << ", "
                               << actual_q_copy_[4] << ", " << actual_q_copy_[5] << "]");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AuboHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &actual_q_copy_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &joint_velocity_copy_[i]));
    }

    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
AuboHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &aubo_position_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &aubo_velocity_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::return_type AuboHardwareInterface::read(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    readActualQ();
    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        initialized_ = true;
    }

    return hardware_interface::return_type::OK;
}

bool AuboHardwareInterface::teachControlEnabled()
{
    std::ifstream flag("/tmp/arachne_aubo_teach_mode");
    char value = '0';
    return flag.good() && (flag >> value) && value == '1';
}

hardware_interface::return_type AuboHardwareInterface::write(
    const rclcpp::Time &time, const rclcpp::Duration &period)
{
    const bool safety_ok =
        safety_mode_ == SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode;
    const bool recoverable_stop =
        safety_mode_ == SafetyModeType::ProtectiveStop || safety_mode_ == SafetyModeType::SafeguardStop;
    const bool poweroff_prestart =
        robot_mode_ == RobotModeType::PowerOff && safety_mode_ == SafetyModeType::Undefined;
    if (teachControlEnabled()) {
        readActualQ();
        aubo_position_commands_ = actual_q_copy_;
        aubo_velocity_commands_.fill(0.0);
        velocity_command_active_ = false;
        if (safety_ok && servo_mode_start_ && stopServoMode() != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                         "Failed to stop Aubo servo mode for teach control.");
            return hardware_interface::return_type::ERROR;
        }
        if (!teach_mode_warned_) {
            RCLCPP_WARN(rclcpp::get_logger("AuboHardwareInterface"),
                        "Arachne teach mode gate is active; skipping servoJoint writes.");
            teach_mode_warned_ = true;
        }
        return hardware_interface::return_type::OK;
    }
    teach_mode_warned_ = false;
    if (robot_mode_ == RobotModeType::Running && safety_ok) {
        if (!servo_mode_start_ && startServoMode() != 0) {
            readActualQ();
            aubo_position_commands_ = actual_q_copy_;
            aubo_velocity_commands_.fill(0.0);
            velocity_command_active_ = false;
            if (!servo_mode_recovery_warned_) {
                RCLCPP_WARN(
                    rclcpp::get_logger("AuboHardwareInterface"),
                    "Aubo servo mode is not ready after teach/prestart; holding measured joints and retrying.");
                servo_mode_recovery_warned_ = true;
            }
            return hardware_interface::return_type::OK;
        }
        servo_mode_recovery_warned_ = false;
        const bool velocity_commanded = std::any_of(
            aubo_velocity_commands_.begin(), aubo_velocity_commands_.end(),
            [](double velocity) { return std::abs(velocity) > 1e-5; });
        if (velocity_commanded) {
            if (!velocity_command_active_) {
                readActualQ();
                aubo_position_commands_ = actual_q_copy_;
            }
            constexpr double kMaxVelocityCommandRadSec = 0.45;
            const double dt = std::clamp(period.seconds(), 0.001, 0.02);
            for (std::size_t i = 0; i < aubo_position_commands_.size(); ++i) {
                double velocity = aubo_velocity_commands_[i];
                if (!std::isfinite(velocity)) {
                    velocity = 0.0;
                }
                velocity = std::clamp(
                    velocity, -kMaxVelocityCommandRadSec, kMaxVelocityCommandRadSec);
                aubo_position_commands_[i] += velocity * dt;
            }
        } else if (velocity_command_active_) {
            // Deadman release: zero velocity means hold the measured position
            // immediately, not the last lookahead target from the jog stream.
            readActualQ();
            aubo_position_commands_ = actual_q_copy_;
        }
        velocity_command_active_ = velocity_commanded;
        try {
            if (Servoj(aubo_position_commands_) != 0) {
                return hardware_interface::return_type::ERROR;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("AuboHardwareInterface"),
                                "Servoj exception: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
    } else if (safety_ok || recoverable_stop || poweroff_prestart) {
        readActualQ();
        aubo_position_commands_ = actual_q_copy_;
        aubo_velocity_commands_.fill(0.0);
        velocity_command_active_ = false;
        if (!waiting_for_running_warned_) {
            RCLCPP_WARN_STREAM(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Aubo hardware interface is active but robot is not ready for servoJoint. "
                "Holding measured command locally. robot_mode_: "
                    << static_cast<int>(robot_mode_)
                    << ", safety_mode_: " << static_cast<int>(safety_mode_));
            waiting_for_running_warned_ = true;
        }
        return hardware_interface::return_type::OK;
    } else {
        // 机器人状态异常
        RCLCPP_WARN_STREAM(
            rclcpp::get_logger("AuboHardwareInterface"),
            "Robot not in valid state for motion command. Plz check&fix robot status firstly then restart driver"
            << "robot_mode_: " << static_cast<int>(robot_mode_)
            << ", safety_mode_: " << static_cast<int>(safety_mode_));

        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

void AuboHardwareInterface::readActualQ()
{
    // 使用 actual_q_copy_
    // 固定该时间戳下read到的位姿，否则读取到的关节状态不稳定
    // actual_q_copy_必须用 array 否则会 bad_alloc
    {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_copy_[0] = actual_q_[0];
        actual_q_copy_[1] = actual_q_[1];
        actual_q_copy_[2] = actual_q_[2];
        actual_q_copy_[3] = actual_q_[3];
        actual_q_copy_[4] = actual_q_[4];
        actual_q_copy_[5] = actual_q_[5];

    //获取机械臂关节速度

        joint_velocity_copy_[0] = joint_velocity_[0];
        joint_velocity_copy_[1] = joint_velocity_[1];
        joint_velocity_copy_[2] = joint_velocity_[2];
        joint_velocity_copy_[3] = joint_velocity_[3];
        joint_velocity_copy_[4] = joint_velocity_[4];
        joint_velocity_copy_[5] = joint_velocity_[5];
    }
}
// 设置rtde输入

bool AuboHardwareInterface::isServoModeStart()
{
    return servo_mode_start_;
}
int AuboHardwareInterface::startServoMode()
{
    if (servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    //开启servo模式
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(true);
    int i = 0;
    while (!rpc_client_->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo Mode enable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    servo_mode_start_ = true;
    return 0;
}

int AuboHardwareInterface::stopServoMode()
{
    if (!servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    while (!rpc_client_->getRobotInterface(robot_name)
                ->getRobotState()
                ->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // 关闭servo模式
    int i = 0;
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(false);
    while (rpc_client_->getRobotInterface(robot_name)
               ->getMotionControl()
               ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo Mode disable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "Servoj end" << std::endl;
    servo_mode_start_ = false;
    return 0;
}

int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    bool command_all_zero = true;
    bool actual_non_zero = false;
    for (size_t i = 0; i < joint_position_command.size(); ++i) {
        command_all_zero = command_all_zero && std::abs(joint_position_command[i]) < 1e-9;
        actual_non_zero = actual_non_zero || std::abs(actual_q_copy_[i]) > 0.05;
    }
    if (command_all_zero && actual_non_zero) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Refusing all-zero Aubo joint command while actual joints are non-zero.");
        return -1;
    }
    if (!first_servoj_logged_) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("AuboHardwareInterface"),
                           "First servoJoint command: ["
                               << joint_position_command[0] << ", "
                               << joint_position_command[1] << ", "
                               << joint_position_command[2] << ", "
                               << joint_position_command[3] << ", "
                               << joint_position_command[4] << ", "
                               << joint_position_command[5] << "]");
        first_servoj_logged_ = true;
    }
    std::vector<double> traj(6, 0);
    for (size_t i = 0; i < traj.size(); i++) {
        traj[i] = joint_position_command[i];
    }

    if (robot_name_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                     "Aubo robot name is empty; refusing servoJoint command.");
        return -1;
    }

    // Arachne servoJoint tuning: keep one bounded SDK call per ros2_control
    // write cycle. Jetson's stock PREEMPT kernel is not RT; a 125 Hz loop with
    // t=8 ms is steadier than chasing the nominal 5 ms SDK example period.
    constexpr double kServoJointVelocity = 0.2;
    constexpr double kServoJointAcceleration = 0.2;
    constexpr double kServoJointPeriodSec = 0.008;
    constexpr double kServoJointLookaheadSec = 0.12;
    constexpr int kServoJointGain = 150;
    const int servoJoint_num = rpc_client_->getRobotInterface(robot_name_)
                                   ->getMotionControl()
                                   ->servoJoint(
                                       traj,
                                       kServoJointVelocity,
                                       kServoJointAcceleration,
                                       kServoJointPeriodSec,
                                       kServoJointLookaheadSec,
                                       kServoJointGain);
    if (servoJoint_num == 2 && !servo_joint_retry_warned_) {
        RCLCPP_WARN(
            rclcpp::get_logger("AuboHardwareInterface"),
            "servoJoint returned retry/busy status; not blocking the write loop. "
            "The next controller cycle will send the next command.");
        servo_joint_retry_warned_ = true;
    } else if (servoJoint_num != 2) {
        servo_joint_retry_warned_ = false;
    }

    return 0;
}
// 设置rtde输入
void AuboHardwareInterface::setInput(RtdeClientPtr cli)
{
    // 接口调用: 发布
    // 组合设置输入
    int topic5 = cli->setTopic(
        true,
        { "input_bit_registers0_to_31", "input_bit_registers32_to_63",
          "input_bit_registers64_to_127", "input_int_registers_0" },
        1, 5);

    std::vector<int> value = { 0x00ff, 0x00, 0x00, 44 };
    cli->publish(
        5, [value](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value); });

    int topic6 = cli->setTopic(
        true, { "input_float_registers_0", "input_double_registers_1" }, 1, 6);

    std::vector<double> value2 = { 3.1, 4.1 };
    cli->publish(
        6, [value2](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value2); });
}
void AuboHardwareInterface::configSubscribe(RtdeClientPtr cli)
{
    // 接口调用: 设置 topic1
    int topic1 = cli->setTopic(
        false,
        { "R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode",
          "runtime_state", "line_number", "R1_actual_TCP_pose" },
        500, 0);
    // 接口调用: 订阅
    cli->subscribe(topic1, [this](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_ = parser.popVectorDouble();
        joint_velocity_ = parser.popVectorDouble();
        robot_mode_ = parser.popRobotModeType();
        safety_mode_ = parser.popSafetyModeType();
        runtime_state_ = parser.popRuntimeState();
        line_ = parser.popInt32();
        actual_TCP_pose_ = parser.popVectorDouble();
        actual_q_received_ = true;
    });
}
} // namespace aubo_driver

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::SystemInterface)
