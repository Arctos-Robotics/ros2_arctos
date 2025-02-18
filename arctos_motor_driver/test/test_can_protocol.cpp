#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "arctos_motor_driver/can_protocol.hpp"
#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

using namespace arctos_motor_driver;

class CANProtocolTest : public testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_node");
        can_protocol_ = std::make_shared<CANProtocol>(node_);
    }

    void TearDown() override {
        can_protocol_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<CANProtocol> can_protocol_;
};

TEST_F(CANProtocolTest, TestDecodeInt48) {
    // Test one complete revolution
    // carry=1 (one full rotation), value=0 (no partial)
    std::vector<uint8_t> pos_data = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
    double angle = CANProtocol::decodeInt48(pos_data);
    EXPECT_NEAR(angle, 360.0, 1e-6) << "Expected 360 degrees for one revolution";

    // Test zero position
    std::vector<uint8_t> zero_data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    angle = CANProtocol::decodeInt48(zero_data);
    EXPECT_NEAR(angle, 0.0, 1e-6) << "Expected 0 degrees for zero position";

    // Test half revolution
    // carry=0, value=0x2000 (half of 0x4000)
    std::vector<uint8_t> half_data = {0x00, 0x00, 0x00, 0x00, 0x20, 0x00};
    angle = CANProtocol::decodeInt48(half_data);
    EXPECT_NEAR(angle, 180.0, 1e-6) << "Expected 180 degrees for half revolution";
}

TEST_F(CANProtocolTest, TestDecodeVelocityToRPM) {
    // Test positive velocity
    std::vector<uint8_t> pos_vel = {0xD0, 0x07};  // 2000 RPM
    double rpm = CANProtocol::decodeVelocityToRPM(pos_vel);
    EXPECT_NEAR(rpm, 2000.0, 1e-6);

    // Test negative velocity
    std::vector<uint8_t> neg_vel = {0x30, 0xF8};  // -2000 RPM
    rpm = CANProtocol::decodeVelocityToRPM(neg_vel);
    EXPECT_NEAR(rpm, -2000.0, 1e-6);
}

TEST_F(CANProtocolTest, TestUnitConversions) {
    // Test encoderToRadians
    EXPECT_NEAR(CANProtocol::encoderToRadians(180.0), M_PI, 1e-6);
    EXPECT_NEAR(CANProtocol::encoderToRadians(360.0), 2 * M_PI, 1e-6);
    EXPECT_NEAR(CANProtocol::encoderToRadians(0.0), 0.0, 1e-6);

    // Test radiansToEncoder
    EXPECT_NEAR(CANProtocol::radiansToEncoder(M_PI), 180.0, 1e-6);
    EXPECT_NEAR(CANProtocol::radiansToEncoder(2 * M_PI), 360.0, 1e-6);
    EXPECT_NEAR(CANProtocol::radiansToEncoder(0.0), 0.0, 1e-6);

    // Test RPM conversions
    EXPECT_NEAR(CANProtocol::rpmToRadPS(60.0), 2 * M_PI, 1e-6);  // 1 rotation per second
    EXPECT_NEAR(CANProtocol::radPSToRPM(2 * M_PI), 60.0, 1e-6);  // 1 rotation per second
}

// TEST_F(CANProtocolTest, TestCalculateCRC) {
//     uint8_t data[] = {0x01, 0x02, 0x03};
//     uint16_t crc = can_protocol_->calculateCRC(data, 3);
//     EXPECT_EQ(crc, 0x06);  // 0x01 + 0x02 + 0x03 = 0x06

//     uint8_t data2[] = {0xFF, 0xFF};
//     crc = can_protocol_->calculateCRC(data2, 2);
//     EXPECT_EQ(crc, 0xFE);  // (0xFF + 0xFF) & 0xFF = 0xFE
// }

TEST_F(CANProtocolTest, TestPublishing) {
    auto test_pub = node_->create_publisher<can_msgs::msg::Frame>(
        "/to_motor_can_bus", 10);

    // Create a subscription to verify the published message
    bool msg_received = false;
    can_msgs::msg::Frame::SharedPtr received_msg;
    
    auto sub = node_->create_subscription<can_msgs::msg::Frame>(
        "/to_motor_can_bus", 10,
        [&msg_received, &received_msg](const can_msgs::msg::Frame::SharedPtr msg) {
            received_msg = msg;
            msg_received = true;
        });

    // Send a test message through our CAN protocol
    std::vector<uint8_t> test_data = {0x01, 0x02, 0x03};
    can_protocol_->sendFrame(1, test_data);

    // Wait for message with timeout
    auto start = node_->now();
    while (rclcpp::ok() && !msg_received && 
           (node_->now() - start) < rclcpp::Duration::from_seconds(1.0)) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_TRUE(msg_received) << "Did not receive test message within timeout";
    ASSERT_NE(received_msg, nullptr) << "Received message pointer is null";
    EXPECT_EQ(received_msg->id, 1);
    EXPECT_EQ(received_msg->dlc, 3);
    EXPECT_EQ(received_msg->data[0], 0x01);
    EXPECT_EQ(received_msg->data[1], 0x02);
    EXPECT_EQ(received_msg->data[2], 0x03);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}