#include <gtest/gtest.h>
#include "arctos_motor_driver/motor_types.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace arctos_motor_driver;

class MotorTypesTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(MotorTypesTest, TestMotorStatusDefaultValues) {
    MotorStatus status;
    EXPECT_FALSE(status.is_enabled);
    EXPECT_FALSE(status.is_calibrated);
    EXPECT_FALSE(status.is_homed);
    EXPECT_FALSE(status.is_error);
    EXPECT_EQ(status.error_code, 0);
    EXPECT_TRUE(status.error_message.empty());
    EXPECT_FALSE(status.limit_switch_left);
    EXPECT_FALSE(status.limit_switch_right);
    EXPECT_FALSE(status.is_stalled);
}

TEST_F(MotorTypesTest, TestMotorParametersDefaultValues) {
    MotorParameters params;
    EXPECT_EQ(params.working_mode, 2);  // Default CR_vFOC
    EXPECT_EQ(params.working_current, 1600);  // Default mA
    EXPECT_EQ(params.holding_current_percentage, 50);
    EXPECT_EQ(params.subdivisions, 16);
    EXPECT_FALSE(params.encoder_direction);
    EXPECT_FALSE(params.protection_enabled);
    EXPECT_EQ(params.pulse_count, 0);
}

TEST_F(MotorTypesTest, TestJointConfigConstruction) {
    JointConfig config(1, "test_joint");
    EXPECT_EQ(config.motor_id, 1);
    EXPECT_EQ(config.joint_name, "test_joint");
    EXPECT_NEAR(config.position, 0.0, 1e-6);
    EXPECT_NEAR(config.velocity, 0.0, 1e-6);
    EXPECT_NEAR(config.command_position, 0.0, 1e-6);
    EXPECT_NEAR(config.command_velocity, 0.0, 1e-6);
    EXPECT_NEAR(config.position_error, 0.0, 1e-6);
}

TEST_F(MotorTypesTest, TestJointConfigLimits) {
    JointConfig config(1, "test_joint");
    EXPECT_NEAR(config.position_min, -M_PI, 1e-6);
    EXPECT_NEAR(config.position_max, M_PI, 1e-6);
    EXPECT_NEAR(config.velocity_max, 50.0, 1e-6);
    EXPECT_NEAR(config.acceleration_max, 100.0, 1e-6);
}

TEST_F(MotorTypesTest, TestMotorModeValues) {
    EXPECT_EQ(static_cast<uint8_t>(MotorMode::CR_OPEN), 0);
    EXPECT_EQ(static_cast<uint8_t>(MotorMode::CR_CLOSE), 1);
    EXPECT_EQ(static_cast<uint8_t>(MotorMode::CR_vFOC), 2);
    EXPECT_EQ(static_cast<uint8_t>(MotorMode::SR_OPEN), 3);
    EXPECT_EQ(static_cast<uint8_t>(MotorMode::SR_CLOSE), 4);
    EXPECT_EQ(static_cast<uint8_t>(MotorMode::SR_vFOC), 5);
}

TEST_F(MotorTypesTest, TestCANCommandValues) {
    EXPECT_EQ(CANCommands::READ_ENCODER, 0x31);
    EXPECT_EQ(CANCommands::READ_VELOCITY, 0x32);
    EXPECT_EQ(CANCommands::CALIBRATE, 0x80);
    EXPECT_EQ(CANCommands::SET_WORKING_MODE, 0x82);
    EXPECT_EQ(CANCommands::ENABLE_MOTOR, 0xF3);
    EXPECT_EQ(CANCommands::EMERGENCY_STOP, 0xF7);
}

TEST_F(MotorTypesTest, TestMotorConstants) {
    EXPECT_EQ(MotorConstants::ENCODER_STEPS, 0x4000);
    EXPECT_NEAR(MotorConstants::DEGREES_PER_REVOLUTION, 360.0, 1e-6);
    EXPECT_NEAR(MotorConstants::RADIANS_PER_REVOLUTION, 2.0 * M_PI, 1e-6);
    EXPECT_NEAR(MotorConstants::MAX_RPM_OPEN, 400.0, 1e-6);
    EXPECT_NEAR(MotorConstants::MAX_RPM_CLOSE, 1500.0, 1e-6);
    EXPECT_NEAR(MotorConstants::MAX_RPM_vFOC, 3000.0, 1e-6);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}