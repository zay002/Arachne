#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${ROOT_DIR}/third_party" "${ROOT_DIR}/src/vendor"
PYTHON_BIN="${ARACHNE_SYSTEM_PYTHON:-/usr/bin/python3}"

fetch_repo() {
  local name="$1"
  local url="$2"
  local ref="$3"
  local dest="${ROOT_DIR}/third_party/${name}"
  local existing=""

  if [[ -d "${dest}" ]]; then
    existing="$(find "${dest}" -mindepth 1 -maxdepth 1 ! -name .git ! -name .gitkeep -print -quit)"
    if [[ -n "${existing}" && "${ARACHNE_REFRESH_THIRD_PARTY:-false}" != "true" ]]; then
      echo "Using vendored/local third-party subset: ${name}"
      return
    fi
  fi

  if [[ ! -d "${dest}/.git" ]]; then
    if [[ -d "${dest}" ]]; then
      if [[ -n "${existing}" ]]; then
        rm -rf "${dest}"
      fi
    fi
    mkdir -p "${dest}"
    git init "${dest}"
    git -C "${dest}" remote add origin "${url}"
  fi

  git -C "${dest}" remote set-url origin "${url}"

  local current_ref=""
  current_ref="$(git -C "${dest}" rev-parse HEAD 2>/dev/null || true)"
  if [[ "${current_ref}" == "${ref}" ]]; then
    return
  fi

  local dirty_status=""
  dirty_status="$(git -C "${dest}" status --porcelain | sed '/^?? \.gitkeep$/d')"
  if [[ -n "${dirty_status}" ]]; then
    echo "Refusing to overwrite dirty third-party repo: ${dest}" >&2
    echo "Commit/stash local changes there, or remove the directory and rerun this script." >&2
    exit 1
  fi

  git -C "${dest}" fetch --depth 1 origin "${ref}"
  git -C "${dest}" checkout --detach FETCH_HEAD
}

fetch_repo aubo_description \
  https://github.com/AuboRobot/aubo_description.git \
  47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1

fetch_repo scout_ros2 \
  https://github.com/agilexrobotics/scout_ros2.git \
  bdbb90471613831fb0b2ec01fecac043445313c4

fetch_repo ugv_sdk \
  https://github.com/agilexrobotics/ugv_sdk.git \
  c3dfaf444f9bae10757e546acae055aaf4a13de7

fetch_repo aubo_ros2_driver \
  https://github.com/AuboRobot/aubo_ros2_driver.git \
  85684075d6ff06c5385e39611208e99ebf0f94c6

# ROS 2 Jazzy no longer ships hardware_interface/visibility_control.h.
# The pinned Aubo driver includes it but does not use symbols from it, so this
# small source compatibility patch keeps the third-party checkout buildable.
aubo_hw_header="${ROOT_DIR}/third_party/aubo_ros2_driver/aubo_ros2_driver/include/aubo_hardware_interface.h"
if [[ -f "${aubo_hw_header}" ]] && grep -q 'hardware_interface/visibility_control.h' "${aubo_hw_header}"; then
  sed -i '/hardware_interface\/visibility_control\.h/d' "${aubo_hw_header}"
fi

# Newer controller_manager spawner versions do not always inherit controller
# parameters from the manager node. Passing the same YAML to the spawners keeps
# joint_trajectory_controller from starting with an empty joints list on Jazzy.
aubo_control_launch="${ROOT_DIR}/third_party/aubo_ros2_driver/aubo_ros2_driver/launch/aubo_control.launch.py"
if [[ -f "${aubo_control_launch}" ]] && ! grep -q '"--param-file",[[:space:]]*robot_controllers' "${aubo_control_launch}"; then
  "${PYTHON_BIN}" - "${aubo_control_launch}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
text = text.replace(
'''        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
''',
'''        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            robot_controllers,
        ],
''',
)
text = text.replace(
'''        arguments=[initial_joint_controller, "-c", "/controller_manager"],
''',
'''        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--param-file",
            robot_controllers,
        ],
''',
)
path.write_text(text)
PY
fi

# Keep the Aubo hardware interface safe during remote startup. The upstream
# driver starts servo mode in hardware activation and errors out if the arm is
# not already Running; that prevents us from activating controllers and sending
# a measured hold-position command before the RobotManage.startup lifecycle. This patch delays
# servoJoint writes until RobotMode=Running, initializes commands from RTDE
# actual_q, and refuses an all-zero command when the measured pose is non-zero.
aubo_hw_source="${ROOT_DIR}/third_party/aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp"
if [[ -f "${aubo_hw_source}" ]] && ! grep -q 'Initialized hold command from actual_q' "${aubo_hw_source}"; then
  "${PYTHON_BIN}" - "${aubo_hw_header}" "${aubo_hw_source}" <<'PY'
from pathlib import Path
import sys

header = Path(sys.argv[1])
source = Path(sys.argv[2])

h = header.read_text()
h = h.replace(
"""    std::array<double, 6> aubo_position_commands_;
    std::array<double, 6> aubo_velocity_commands_;
    double speed_scaling_combined_;
    bool controllers_initialized_;
    bool servo_mode_start_{ false };
    bool initialized_;
""",
"""    std::array<double, 6> aubo_position_commands_{};
    std::array<double, 6> aubo_velocity_commands_{};
    double speed_scaling_combined_;
    bool controllers_initialized_;
    bool servo_mode_start_{ false };
    bool initialized_;
    std::atomic<bool> actual_q_received_{ false };
    bool waiting_for_running_warned_{ false };
    bool first_servoj_logged_{ false };
""",
)
h = h.replace(
"""    std::array<double, 6> actual_q_copy_;
    std::array<double, 6> joint_velocity_copy_;
""",
"""    std::array<double, 6> actual_q_copy_{};
    std::array<double, 6> joint_velocity_copy_{};
""",
)
header.write_text(h)

s = source.read_text()
s = s.replace('#include "hardware_interface/types/hardware_interface_type_values.hpp"\n#include <ctime>',
              '#include "hardware_interface/types/hardware_interface_type_values.hpp"\n#include <cmath>\n#include <ctime>')
s = s.replace("\n    startServoMode();\n\n    return true;\n", "\n    return true;\n")
s = s.replace(
    "    if (robot_mode_ == RobotModeType::Running && (safety_mode_ == \n",
    "    if (robot_mode_ == RobotModeType::Running && (safety_mode_ ==\n",
)
s = s.replace(
"""    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    readActualQ();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!initialized_) {
        //获取初始状态
        aubo_position_commands_ = actual_q_copy_;
        initialized_ = true;
    }
""",
"""    for (int i = 0; i < 200 && !actual_q_received_; ++i) {
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
""",
)
s = s.replace(
"""    if (robot_mode_ == RobotModeType::Running && (safety_mode_ ==
        SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode)) {
        try {
            Servoj(aubo_position_commands_);
        } catch (const std::exception &e) {
        }
    }else{
        // 机器人状态异常
        RCLCPP_WARN_STREAM(
            rclcpp::get_logger("AuboHardwareInterface"),
            "Robot not in valid state for motion command. Plz check&fix robot status firstly then restart driver"
            << "robot_mode_: " << static_cast<int>(robot_mode_)
            << ", safety_mode_: " << static_cast<int>(safety_mode_));

        return hardware_interface::return_type::ERROR;
    }
""",
"""    const bool safety_ok =
        safety_mode_ == SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode;
    if (robot_mode_ == RobotModeType::Running && safety_ok) {
        if (!servo_mode_start_ && startServoMode() != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                         "Failed to enter Aubo servo mode; refusing to write commands.");
            return hardware_interface::return_type::ERROR;
        }
        try {
            if (Servoj(aubo_position_commands_) != 0) {
                return hardware_interface::return_type::ERROR;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("AuboHardwareInterface"),
                                "Servoj exception: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
    } else if (safety_ok) {
        readActualQ();
        aubo_position_commands_ = actual_q_copy_;
        aubo_velocity_commands_.fill(0.0);
        if (!waiting_for_running_warned_) {
            RCLCPP_WARN_STREAM(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Aubo hardware interface is active but robot is not Running yet. "
                "Holding measured command locally and not sending servoJoint. robot_mode_: "
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
""",
)
s = s.replace(
"""int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();
""",
"""int AuboHardwareInterface::Servoj(
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
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();
""",
)
s = s.replace(
"""        actual_TCP_pose_ = parser.popVectorDouble();
""",
"""        actual_TCP_pose_ = parser.popVectorDouble();
        actual_q_received_ = true;
""",
)
source.write_text(s)
PY
fi

# Let the teach panel hand-guide the real Aubo arm without fighting the
# ros2_control servo hold loop. The bridge writes this local flag before
# calling RobotManage.freedrive(true); the hardware interface then stops servo
# mode and skips servoJoint writes until the flag is cleared.
if [[ -f "${aubo_hw_source}" ]] && ! grep -q 'teachControlEnabled' "${aubo_hw_source}"; then
  "${PYTHON_BIN}" - "${aubo_hw_header}" "${aubo_hw_source}" <<'PY'
from pathlib import Path
import sys

header = Path(sys.argv[1])
source = Path(sys.argv[2])

h = header.read_text()
h = h.replace(
"""    void readActualQ();

    void setInput(RtdeClientPtr cli);
""",
"""    void readActualQ();

    bool teachControlEnabled();

    void setInput(RtdeClientPtr cli);
""",
)
h = h.replace(
"""    bool waiting_for_running_warned_{ false };
    bool first_servoj_logged_{ false };
""",
"""    bool waiting_for_running_warned_{ false };
    bool first_servoj_logged_{ false };
    bool teach_mode_warned_{ false };
""",
)
header.write_text(h)

s = source.read_text()
if "#include <fstream>" not in s:
    s = s.replace("#include <ctime>\n", "#include <ctime>\n#include <fstream>\n")
s = s.replace(
"""    return hardware_interface::return_type::OK;
}
hardware_interface::return_type AuboHardwareInterface::write(
""",
"""    return hardware_interface::return_type::OK;
}

bool AuboHardwareInterface::teachControlEnabled()
{
    std::ifstream flag("/tmp/arachne_aubo_teach_mode");
    char value = '0';
    return flag.good() && (flag >> value) && value == '1';
}

hardware_interface::return_type AuboHardwareInterface::write(
""",
)
s = s.replace(
"""    const bool safety_ok =
        safety_mode_ == SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode;
    if (robot_mode_ == RobotModeType::Running && safety_ok) {
""",
"""    const bool safety_ok =
        safety_mode_ == SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode;
    if (safety_ok && teachControlEnabled()) {
        readActualQ();
        aubo_position_commands_ = actual_q_copy_;
        aubo_velocity_commands_.fill(0.0);
        if (servo_mode_start_ && stopServoMode() != 0) {
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
""",
)
source.write_text(s)
PY
fi

# Teach mode exits can briefly leave Aubo's servo mode unavailable even though
# RobotMode already reports Running. Keep ros2_control active and retry instead
# of returning ERROR, otherwise the trajectory controller is deactivated and
# teach replay cannot command the arm again.
if [[ -f "${aubo_hw_source}" ]] && ! grep -q 'holding measured joints and retrying' "${aubo_hw_source}"; then
  "${PYTHON_BIN}" - "${aubo_hw_header}" "${aubo_hw_source}" <<'PY'
from pathlib import Path
import sys

header = Path(sys.argv[1])
source = Path(sys.argv[2])

h = header.read_text()
if "servo_mode_recovery_warned_" not in h:
    h = h.replace(
"""    bool first_servoj_logged_{ false };
    bool teach_mode_warned_{ false };
""",
"""    bool first_servoj_logged_{ false };
    bool teach_mode_warned_{ false };
    bool servo_mode_recovery_warned_{ false };
""",
    )
header.write_text(h)

s = source.read_text()
s = s.replace(
"""        if (!servo_mode_start_ && startServoMode() != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("AuboHardwareInterface"),
                         "Failed to enter Aubo servo mode; refusing to write commands.");
            return hardware_interface::return_type::ERROR;
        }
        try {
""",
"""        if (!servo_mode_start_ && startServoMode() != 0) {
            readActualQ();
            aubo_position_commands_ = actual_q_copy_;
            aubo_velocity_commands_.fill(0.0);
            if (!servo_mode_recovery_warned_) {
                RCLCPP_WARN(
                    rclcpp::get_logger("AuboHardwareInterface"),
                    "Aubo servo mode is not ready after teach/prestart; holding measured joints and retrying.");
                servo_mode_recovery_warned_ = true;
            }
            return hardware_interface::return_type::OK;
        }
        servo_mode_recovery_warned_ = false;
        try {
""",
)
source.write_text(s)
PY
fi

# Teach Off can report Running while safety is still ProtectiveStop for a short
# period. During that transition the hardware plugin must keep controller
# manager alive and hold measured joints, not return ERROR.
if [[ -f "${aubo_hw_source}" ]] && ! grep -q 'robot is not ready for servoJoint' "${aubo_hw_source}"; then
  "${PYTHON_BIN}" - "${aubo_hw_source}" <<'PY'
from pathlib import Path
import sys

source = Path(sys.argv[1])
s = source.read_text()
s = s.replace(
"""    const bool safety_ok =
        safety_mode_ == SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode;
    if (safety_ok && teachControlEnabled()) {
""",
"""    const bool safety_ok =
        safety_mode_ == SafetyModeType::Normal || safety_mode_ == SafetyModeType::ReducedMode;
    const bool recoverable_stop =
        safety_mode_ == SafetyModeType::ProtectiveStop || safety_mode_ == SafetyModeType::SafeguardStop;
    if (teachControlEnabled()) {
""",
)
s = s.replace(
"""        if (servo_mode_start_ && stopServoMode() != 0) {
""",
"""        if (safety_ok && servo_mode_start_ && stopServoMode() != 0) {
""",
)
s = s.replace(
"""    } else if (safety_ok) {
        readActualQ();
        aubo_position_commands_ = actual_q_copy_;
        aubo_velocity_commands_.fill(0.0);
        if (!waiting_for_running_warned_) {
            RCLCPP_WARN_STREAM(
                rclcpp::get_logger("AuboHardwareInterface"),
                "Aubo hardware interface is active but robot is not Running yet. "
                "Holding measured command locally and not sending servoJoint. robot_mode_: "
                    << static_cast<int>(robot_mode_)
                    << ", safety_mode_: " << static_cast<int>(safety_mode_));
            waiting_for_running_warned_ = true;
        }
        return hardware_interface::return_type::OK;
""",
"""    } else if (safety_ok || recoverable_stop) {
        readActualQ();
        aubo_position_commands_ = actual_q_copy_;
        aubo_velocity_commands_.fill(0.0);
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
""",
)
source.write_text(s)
PY
fi

fetch_repo dh_ag95_gripper_ros2 \
  https://github.com/ian-chuang/dh_ag95_gripper_ros2.git \
  fc4f80fdfb3acae5626df4359aec1401cb71a9a3

ln -sfn ../../third_party/aubo_description "${ROOT_DIR}/src/vendor/aubo_description"
ln -sfn ../../third_party/dh_ag95_gripper_ros2/dh_ag95_description "${ROOT_DIR}/src/vendor/dh_ag95_description"
ln -sfn ../../third_party/scout_ros2/scout_description "${ROOT_DIR}/src/vendor/scout_description"

echo "Third-party model and hardware ROS packages are ready at pinned revisions."
