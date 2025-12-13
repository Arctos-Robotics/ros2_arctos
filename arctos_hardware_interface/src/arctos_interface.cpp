#include "arctos_hardware_interface/arctos_interface.hpp"
// #include "arctos_hardware_interface/arctos_services.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>
#include <chrono>

using arctos_motor_driver::MotorMode;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace arctos_interface
{

    ArctosInterface::ArctosInterface()
        : SystemInterface(),
          node_(std::make_shared<rclcpp::Node>("arctos_hardware_interface"))
    {
        uart_protocol_ = std::make_shared<arctos_motor_driver::UartProtocol>();
        motor_driver_ = std::make_shared<arctos_motor_driver::MotorDriver>(node_);
        motor_driver_->setProtocol(uart_protocol_);

    }

    ArctosInterface::~ArctosInterface() = default;

    /* communication between the robot hardware needs to be setup and memory dynamic should be allocated */
    /* HardwareInfo info could be collected from yaml file, under tag hardware/parameters*/
    CallbackReturn ArctosInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // Initialize state storage vectors
        joint_position_.resize(info_.joints.size(), 0.0);
        joint_velocities_.resize(info_.joints.size(), 0.0);
        joint_position_command_.resize(info_.joints.size(), 0.0);
        joint_velocities_command_.resize(info_.joints.size(), 0.0);
        motor_ids_.resize(info_.joints.size());

        // Force/torque sensor has 6 readings
        ft_states_.assign(6, 0);
        ft_command_.assign(6, 0);

        // Clear previous interface mappings
        joint_interfaces["position"].clear();
        joint_interfaces["velocity"].clear();

        // Collect joint names for service initialization
        std::vector<std::string> joint_names;
        joint_names.reserve(info_.joints.size());

        
        try
        {
            // Get params from Ros2_control/hardware/param in urdf
            std::string device = info_.hardware_parameters["device"];
            int baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
            int timeout = std::stoi(info_.hardware_parameters["timeout"]);
            // then setup uart connection
            uart_protocol_->setup(device, baud_rate, timeout);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to initialize UART connection, assuming using simulation: %s", e.what());
        }
        
        // Process joints and their interfaces
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            const auto &joint = info_.joints[i];
            joint_names.push_back(joint.name);

            // Create parameter name for this joint's settings
            std::string param_prefix = "motors." + joint.name + ".";

            // Declare parameters for this joint
            node_->declare_parameter(param_prefix + "motor_id", -1);             // Motor/CAN ID
            // node_->declare_parameter(param_prefix + "working_current", 1600); // Default 1.6A
            // node_->declare_parameter(param_prefix + "holding_current", 50);   // Default 50%
            // node_->declare_parameter(param_prefix + "home_current", 800);     // Default 0.8A for homing
            node_->declare_parameter(param_prefix + "hardware_type", "MKS_42D"); // Default MKS Servo
            node_->declare_parameter(param_prefix + "gear_ratio", 1.0);          // Default 1:1 gear ratio
            node_->declare_parameter(param_prefix + "inverted", false);          // Default no inverted in application side
            node_->declare_parameter(param_prefix + "inverted_feedback", false); // Default no inverted in physical side
            node_->declare_parameter(param_prefix + "requires_homing", false);   // Default no homing needed
            node_->declare_parameter(param_prefix + "lower_limit", 0.0);
            node_->declare_parameter(param_prefix + "upper_limit", 0.0);

            // Get motor ID from parameters
            int motor_id;
            if (!node_->get_parameter(param_prefix + "motor_id", motor_id))
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get motor_id for joint %s", joint.name.c_str());
                return CallbackReturn::ERROR;
            }

            if (motor_id < 0)
            {
                RCLCPP_ERROR(node_->get_logger(), "Invalid motor_id (%d) for joint %s", motor_id, joint.name.c_str());
                return CallbackReturn::ERROR;
            }

            motor_ids_[i] = static_cast<uint8_t>(motor_id);

            RCLCPP_INFO(node_->get_logger(), "Configured joint %s with motor_id %d",
                        joint.name.c_str(), motor_id);

            // Track available interfaces
            for (const auto &interface : joint.state_interfaces)
            {
                joint_interfaces[interface.name].push_back(joint.name);
                if (interface.name == "position")
                    has_position_interface_ = true;
                if (interface.name == "velocity")
                    has_velocity_interface_ = true;
            }
            
        }

        node_->declare_parameter("position_tolerance", 0.001);
        node_->declare_parameter("velocity_tolerance", 0.01);

        // Get parameter values
        node_->get_parameter("position_tolerance", position_tolerance_);
        node_->get_parameter("velocity_tolerance", velocity_tolerance_);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(node_->get_logger(), "Transitioning to CONFIGURE state from %s", previous_state.label().c_str());

        try
        {
            initializeMotors();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize motors: %s", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(node_->get_logger(), "Transitioning to ACTIVE state from %s", previous_state.label().c_str());

        // Enable all motors
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            try
            {
                const auto &joint_name = info_.joints[i].name;

                RCLCPP_INFO(node_->get_logger(), "Enabling motor for joint %s", joint_name.c_str());
                // Enable the motor first
                // motor_driver_->enableMotor(joint_name);

                // Check if homing is required
                bool requires_homing = false;
                std::string param_prefix = "motors." + joint_name + ".";
                if (node_->get_parameter(param_prefix + "requires_homing", requires_homing) && requires_homing)
                {
                    RCLCPP_INFO(node_->get_logger(), "Starting homing sequence for joint %s", joint_name.c_str());

                    RCLCPP_INFO(node_->get_logger(), "Homing completed for joint %s", joint_name.c_str());
                }

            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to activate joint %s: %s",
                             info_.joints[i].name.c_str(), e.what());
                return CallbackReturn::ERROR;
            }
            joint_position_command_[i] = 0.0; // force home on first activation
        }

        // thread spin is required for continously reading UART strings, 
        allowSpin = true;
        spinThread = std::thread(
            [&]()
            {
                while (allowSpin) {
                    uart_protocol_->readToBuffer();
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            });

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArctosInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(node_->get_logger(), "Transitioning to INACTIVE state from %s", previous_state.label().c_str());
        // Disable all motors
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            try
            {
                // motor_driver_->disableMotor(info_.joints[i].name);
                i = i;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to disable motor for joint %s: %s",
                             info_.joints[i].name.c_str(), e.what());
            }
        }
        allowSpin = false;
        spinThread.join();
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ArctosInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Add joint state interfaces
        /*The StateInterface objects are read only data handles.
        Their constructors require an (interface name, interface type, and a pointer to a double data value)*/
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            if (has_position_interface_)
            {
                state_interfaces.emplace_back(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_[i]);
            }
            if (has_velocity_interface_)
            {
                state_interfaces.emplace_back(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
            }
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ArctosInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Add joint command interfaces
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            if (has_position_interface_)
            {
                command_interfaces.emplace_back(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]);
            }
            if (has_velocity_interface_)
            {
                command_interfaces.emplace_back(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_command_[i]);
            }
        }

        return command_interfaces;
    }

    /* core method in ros2_control loop
    During the main loop, ros2_control loops over all hardware components and calls the read method.
    -> responsible for updating the data values of the *state_interfaces*, by updating the *class member variable*
    */
    return_type ArctosInterface::read(const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {
        // spin_some is neccessary, if not used, hardware_interface won't catch any subscription event.
        // rclcpp::spin_some(node_);
        // created a thread that "spin()" for event already, hence spin_some here is not needed. :o


        // Process UART messages
        motor_driver_->processUartMessage();

        static rclcpp::Time last_update_time = time; //  Static variable retains value between calls
        auto elapsed_time = time - last_update_time;

        // Limit CAN queries to once every 500ms
        // if (elapsed_time.seconds() > 0.01) {
        //     motor_driver_->updateJointStates();  // Fetch fresh data from CAN bus
        //     last_update_time = time;  //  Now correctly updated after each call
        // }

        // updateJointStates is a function that explicitly request the joint status of the robot.
        // currently, data is updated automatically to robot when running. this is not yet used.
        // motor_driver_->updateJointStates();

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            const std::string &joint_name = info_.joints[i].name;
            try
            {
                RCLCPP_DEBUG(node_->get_logger(), "Reading state for joint %s", joint_name.c_str());

                if (has_position_interface_)
                {
                    // for now, the gripper state will update directly from position_command.
                    // TODO: read actualy gripper state and update.
                    if (joint_name == "Left_jaw_joint") 
                    {
                        joint_position_[i] = joint_position_command_[i];
                    }
                    else 
                    {
                        double pos = motor_driver_->getJointPosition(joint_name, true);
                        joint_position_[i] = pos;
                        RCLCPP_DEBUG(node_->get_logger(), "Updated position for joint %s: %.3f", joint_name.c_str(), pos);
                    }
                    
                }

                if (has_velocity_interface_)
                {
                    // double vel = motor_driver_->getJointVelocity(joint_name);
                    // joint_velocities_[i] = vel;
                    // RCLCPP_DEBUG(node_->get_logger(), "Updated velocity for joint %s: %.3f", joint_name.c_str(), vel);
                }

                rclcpp::Duration time_since_update = motor_driver_->getTimeSinceLastUpdate(joint_name);
                if (time_since_update.seconds() > 1.0)
                {
                    // RCLCPP_WARN(node_->get_logger(),"Stale data for joint %s: %.3f seconds since last update",joint_name.c_str(), time_since_update.seconds());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to read state from joint %s: %s",
                             joint_name.c_str(), e.what());
                return return_type::ERROR;
            }
        }

        return return_type::OK;
    }

    /* The *write* method is another core method in the ros2_control loop.
    It is called after *update* in the realtime loop.
    responsible for updating the data values of the *command_interfaces*
    */
    return_type ArctosInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        static bool isPositionUpdated;
        isPositionUpdated = false;
        // Resize last command vectors if not already done
        if (last_position_command_.size() != info_.joints.size())
        {
            last_position_command_.resize(info_.joints.size(), -1.0);   // force homing on first activation
            last_velocity_command_.resize(info_.joints.size(), 0.0);
            RCLCPP_INFO(node_->get_logger(), "Initialized last command vectors.");
        }

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            try
            {
                // this is a wrong design! who the fuck send velocity via speed control mode (F6??)
                if (has_velocity_interface_)
                {
                    // TODO: Ensure this works properly
                    // Only send if velocity has changed significantly
                    if (std::abs(joint_velocities_command_[i] - last_velocity_command_[i]) > velocity_tolerance_)
                    {
                        // motor_driver_->setJointVelocity(info_.joints[i].name, joint_velocities_command_[i]);
                        RCLCPP_INFO(node_->get_logger(),
                                    "Sent velocity command %.3f to joint %s. Last command: %.3f",
                                    joint_velocities_command_[i], info_.joints[i].name.c_str(),
                                    last_velocity_command_[i]);
                        last_velocity_command_[i] = joint_velocities_command_[i];
                    }
                    else
                    {
                        RCLCPP_DEBUG(node_->get_logger(),
                                     "Velocity command for joint %s unchanged: %.3f",
                                     info_.joints[i].name.c_str(), joint_velocities_command_[i]);
                    }
                }

                if (has_position_interface_)
                {
                    // Only send if position has changed significantly
                    if (std::abs(joint_position_command_[i] - last_position_command_[i]) > position_tolerance_)
                    {
                        if (info_.joints[i].name == "Left_jaw_joint") 
                        {
                            // TODO: write function to control gripper.
                            // down here, we only have to care about what position will we drive our actuator. 
                            // the talking between actions, msg between moveit and ros2 has been handled by moveit before reaching here.
                            continue;
                        }
                        else
                        {
                            motor_driver_->setJointPosition(info_.joints[i].name, joint_position_command_[i], 0, abs(joint_velocities_command_[i] * 10) * 60);
                            RCLCPP_INFO(node_->get_logger(),
                                        "Sent position command %.5f to joint %s. Last command: %.5f.",
                                        joint_position_command_[i], info_.joints[i].name.c_str(),
                                        last_position_command_[i]);
                            last_position_command_[i] = joint_position_command_[i];
                        }
                        isPositionUpdated = true;
                    }
                    else
                    {
                        RCLCPP_DEBUG(node_->get_logger(),
                                     "Position command for joint %s unchanged: %.3f",
                                     info_.joints[i].name.c_str(), joint_position_command_[i]);
                    }
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Failed to write command to joint %s: %s",
                             info_.joints[i].name.c_str(), e.what());
                return return_type::ERROR;
            }
        }

        // Write command to the actuator.
        if (isPositionUpdated) {
            motor_driver_->writeCommand();
        }
        
        return return_type::OK;
    }

    void ArctosInterface::initializeMotors()
    {
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            const auto &joint = info_.joints[i];
            uint8_t motor_id = motor_ids_[i];

            std::string param_prefix = "motors." + joint.name + ".";

            // Get hardware_type parameter
            std::string hardware_type;
            if (!node_->get_parameter(param_prefix + "hardware_type", hardware_type))
            {
                RCLCPP_WARN(node_->get_logger(), "No hardware type specified for joint %s, using default MKS_42D",
                            joint.name.c_str());
                hardware_type = "MKS_42D";
            }

            double gear_ratio;
            if (!node_->get_parameter(param_prefix + "gear_ratio", gear_ratio))
            {
                RCLCPP_WARN(node_->get_logger(), "No gear ratio specified for joint %s, using 1:1",
                            joint.name.c_str());
                gear_ratio = 1.0;
            }

            bool inverted;
            if (!node_->get_parameter(param_prefix + "inverted", inverted))
            {
                RCLCPP_WARN(node_->get_logger(), "No inverted specified for joint %s, using false",
                            joint.name.c_str());
                inverted = false;
            }

            bool inverted_feedback;
            if (!node_->get_parameter(param_prefix + "inverted_feedback", inverted_feedback))
            {
                RCLCPP_WARN(node_->get_logger(), "No inverted_feedback specified for joint %s, using false",
                            joint.name.c_str());
                inverted_feedback = false;
            }

            // Add joint to motor driver with gear ratio
            motor_driver_->addJoint(joint.name, motor_id, hardware_type, gear_ratio, inverted, inverted_feedback);

            // Configure motor parameters
            if (!setupMotorParameters(joint, motor_id))
            {
                throw std::runtime_error("Failed to configure motor for joint " + joint.name);
            }

            RCLCPP_INFO(node_->get_logger(), "Initialized motor for joint %s with ID %d and gear ratio %.2f:1",
                        joint.name.c_str(), motor_id, gear_ratio);
        }
    }

    bool ArctosInterface::setupMotorParameters(
        const hardware_interface::ComponentInfo &joint_info, uint8_t motor_id)
    {
        try
        {
            double gear_ratio;
            double lower_limit;
            double upper_limit;
            double max_rpm = 3000.0;
            std::string param_prefix = "motors." + joint_info.name + ".";

            // Get gear ratio from parameters
            if (!node_->get_parameter(param_prefix + "gear_ratio", gear_ratio))
            {
                RCLCPP_WARN(node_->get_logger(), "No gear ratio specified for joint %s, using 1:1",
                            joint_info.name.c_str());
                gear_ratio = 1.0;
            }

            // Get home position from parameters

            node_->get_parameter(param_prefix + "lower_limit", lower_limit);
            node_->get_parameter(param_prefix + "upper_limit", upper_limit);

            // Calculate maximum velocity based on gear ratio
            double max_velocity = (max_rpm * M_PI / 30.0) / gear_ratio;

            // Set joint limits using home position and opposite limit
            motor_driver_->setJointLimits(joint_info.name, lower_limit, upper_limit, max_velocity, 255.0);

            RCLCPP_INFO(node_->get_logger(), "Set joint limits for joint %s of motor %d: pos=[%.2f, %.2f], vel=%.2f, acc=%.2f",
                        joint_info.name.c_str(), motor_id, lower_limit, upper_limit, max_velocity, 255.0);

            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to setup motor parameters for joint %s: %s",
                         joint_info.name.c_str(), e.what());
            return false;
        }
    }

} // namespace arctos_interface

PLUGINLIB_EXPORT_CLASS(
    arctos_interface::ArctosInterface,
    hardware_interface::SystemInterface)