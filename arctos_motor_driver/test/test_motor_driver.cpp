#include <gtest/gtest.h>
#include "arctos_motor_driver/motor_driver.hpp"
#include "arctos_motor_driver/motor_types.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace arctos_motor_driver;

class TestCANProtocol : public CANProtocol {
public:
    explicit TestCANProtocol(rclcpp::Node::SharedPtr node) : CANProtocol(node) {
        last_motor_id = 0;
        call_count = 0;
    }
    
    std::vector<std::vector<uint8_t>> sent_frames;
    
    void sendFrame(uint8_t motor_id, const std::vector<uint8_t>& data) override {
        last_motor_id = motor_id;
        last_data = data;
        sent_frames.push_back(data);
        call_count++;
    }
    
    uint8_t last_motor_id;
    std::vector<uint8_t> last_data;
    int call_count;
};

class MotorDriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_node");
        can_protocol_ = std::make_shared<TestCANProtocol>(node_);
        motor_driver_ = std::make_shared<MotorDriver>(node_);
        motor_driver_->setCAN(can_protocol_);
    }

    void TearDown() override {
        motor_driver_.reset();
        can_protocol_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<TestCANProtocol> can_protocol_;
    std::shared_ptr<MotorDriver> motor_driver_;
};

TEST_F(MotorDriverTest, TestEnableDisable) {
    motor_driver_->addJoint("joint1", 1);
    
    // Test enabling motor
    const std::vector<uint8_t> enable_cmd = {CANCommands::ENABLE_MOTOR, 0x01};
    motor_driver_->enableMotor("joint1");
    EXPECT_EQ(can_protocol_->last_motor_id, 1);
    EXPECT_EQ(can_protocol_->last_data, enable_cmd);
    EXPECT_TRUE(motor_driver_->getMotorStatus("joint1").is_enabled);

    // Test disabling motor
    const std::vector<uint8_t> disable_cmd = {CANCommands::ENABLE_MOTOR, 0x00};
    motor_driver_->disableMotor("joint1");
    EXPECT_EQ(can_protocol_->last_motor_id, 1);
    EXPECT_EQ(can_protocol_->last_data, disable_cmd);
    EXPECT_FALSE(motor_driver_->getMotorStatus("joint1").is_enabled);
}

TEST_F(MotorDriverTest, TestEmergencyStop) {
    const std::vector<uint8_t> stop_cmd = {CANCommands::EMERGENCY_STOP};
    
    motor_driver_->addJoint("joint1", 1);
    motor_driver_->addJoint("joint2", 2);
    int initial_count = can_protocol_->call_count;
    motor_driver_->stopAllMotors();
    
    // Verify two calls were made
    EXPECT_EQ(can_protocol_->call_count - initial_count, 2);
    // Verify last command was correct format
    EXPECT_EQ(can_protocol_->last_data, stop_cmd);
}

TEST_F(MotorDriverTest, TestCalibration) {
    const std::vector<uint8_t> cal_cmd = {CANCommands::CALIBRATE, 0x00};
    
    motor_driver_->addJoint("joint1", 1);
    motor_driver_->calibrateMotor("joint1");
    
    EXPECT_EQ(can_protocol_->last_motor_id, 1);
    EXPECT_EQ(can_protocol_->last_data, cal_cmd);
    EXPECT_FALSE(motor_driver_->getMotorStatus("joint1").is_calibrated);
}

TEST_F(MotorDriverTest, TestPositionControl) {
    motor_driver_->addJoint("joint1", 1);
    
    // Test absolute position command
    double target_pos = M_PI/2;  // 90 degrees
    
    // Print out debug information
    RCLCPP_INFO(node_->get_logger(), "Target position: %f radians", target_pos);
    
    // Convert position to encoder counts using the same method as in implementation
    int32_t encoder_counts = static_cast<int32_t>(
        (target_pos * MotorConstants::ENCODER_STEPS) / (2.0 * M_PI)
    );
    
    RCLCPP_INFO(node_->get_logger(), "Encoder counts: %d", encoder_counts);
    
    // Determine speed (matching implementation)
    uint16_t speed = 600;  // Default speed
    
    const std::vector<uint8_t> pos_cmd = {
        CANCommands::ABSOLUTE_POSITION,  // Command byte (0xF5)
        static_cast<uint8_t>(speed & 0xFF),           // Speed low byte
        static_cast<uint8_t>((speed >> 8) & 0x0F),    // Speed high byte
        0x02,  // Acceleration
        static_cast<uint8_t>(encoder_counts & 0xFF),           // Position lowest byte
        static_cast<uint8_t>((encoder_counts >> 8) & 0xFF),    // Position middle byte
        static_cast<uint8_t>((encoder_counts >> 16) & 0xFF)    // Position highest byte
    };
    
    // Print out expected command details
    RCLCPP_INFO(node_->get_logger(), "Expected command bytes:");
    for (size_t i = 0; i < pos_cmd.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "Byte %zu: 0x%02X", i, pos_cmd[i]);
    }
    
    motor_driver_->setJointPosition("joint1", target_pos);
    
    // Print out actual received data
    RCLCPP_INFO(node_->get_logger(), "Actual received bytes:");
    for (size_t i = 0; i < can_protocol_->last_data.size(); ++i) {
        RCLCPP_INFO(node_->get_logger(), "Byte %zu: 0x%02X", i, can_protocol_->last_data[i]);
    }
    
    ASSERT_FALSE(can_protocol_->sent_frames.empty());
    EXPECT_EQ(can_protocol_->sent_frames[0], pos_cmd) << "First command should be position control";
    EXPECT_EQ(can_protocol_->sent_frames[1], std::vector<uint8_t>{CANCommands::READ_ENCODER}) << "Second command should be READ_ENCODER";

}

TEST_F(MotorDriverTest, TestVelocityControl) {
    motor_driver_->addJoint("joint1", 1);
    
    // Test velocity command (2 rad/s ≈ 19.1 RPM)
    double target_vel = 2.0;
    double rpm = target_vel * MotorConstants::RADPS_TO_RPM;
    uint16_t speed = static_cast<uint16_t>(std::abs(rpm));
    
    const std::vector<uint8_t> vel_cmd = {
        CANCommands::SPEED_CONTROL,
        static_cast<uint8_t>(((speed >> 8) & 0x0F)),  // Positive direction
        static_cast<uint8_t>(speed & 0xFF),
        0x02  // Default acceleration
    };
    
    motor_driver_->setJointVelocity("joint1", target_vel);
    EXPECT_EQ(can_protocol_->last_motor_id, 1);
    EXPECT_EQ(can_protocol_->last_data, vel_cmd);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}