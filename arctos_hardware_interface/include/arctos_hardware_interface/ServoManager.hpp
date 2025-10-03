#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <cmath>

class ServoManager {
public:
    static constexpr double TWO_PI = 6.283185307179586;
    static constexpr int ENCODER_CPR = 16384; // counts per revolution

    struct CMD {
        struct HOME { static const uint8_t SET = 0x90, GO = 0x91, ZERO = 0x92; };
        struct READ {
            static const uint8_t ENCODER_CARRY = 0x30;
            static const uint8_t ENCODER_ABSOLUTE = 0x31;
            static const uint8_t SPEED = 0x32;
            static const uint8_t ANGLE = 0x39;
        };
        struct SET {
            static const uint8_t WORKING_CURRENT = 0x83;
            static const uint8_t HOLDING_CURRENT = 0x9B;
            static const uint8_t MODE = 0x82;
            static const uint8_t CAN_ID = 0x8B;
        };
        struct MOTOR {
            static const uint8_t ABSOLUTE = 0xFE;
            static const uint8_t RELATIVE_PULSE = 0xFD;
            static const uint8_t RELATIVE_AXIS = 0xF4;
            static const uint8_t ABSOLUTE_AXIS = 0xF5;
            static const uint8_t SPEED = 0xF6;
            static const uint8_t STOP = 0xF7;
        };
        struct CONTROL {
            static const uint8_t ENABLE = 0xF3;
        };
    };

    enum ResponseType { TYPE_NONE, TYPE_ENCODER, TYPE_SPEED, TYPE_MOTION, TYPE_ZERO, TYPE_IO };
    struct ResponseData {
        ResponseType type = TYPE_NONE;
        union {
            uint32_t value;
            int16_t speed;
        };
    };

    ServoManager(uint8_t id = 0x01, double ratio = 1.0);

    // Motion commands
    struct can_frame runMotorSpeedMode(uint8_t dir, uint16_t speed, uint8_t accel);
    struct can_frame enableServo(bool on);
    struct can_frame stopMotor();
    struct can_frame absoluteMotion(uint16_t speed, uint8_t accel, uint32_t position);
    struct can_frame absoluteMotionRad(double angleRad, uint16_t speed, uint8_t accel);

    // Read commands
    struct can_frame readEncoderAbsolute();
    struct can_frame readEncoderCarry();
    struct can_frame readSpeed();

    // Set commands
    struct can_frame setWorkingCurrent(uint16_t current);
    struct can_frame setHoldingCurrent(uint8_t percent);

    // Response parsing
    ResponseData parseResponse(const struct can_frame& frame);
    void updateSpeedFromCAN(int16_t rpm);

    // Encoder tracking
    void updateEncoder(uint32_t rawValue);

    // ROS2-friendly methods
    void setGearRatio(double ratio) { gearRatio = ratio; }
    double getAbsoluteAngle() const;             // unwrapped angle (can be > 2π)
    double getVelocityRadPerSec() const { return speedRadPerSec; }
    void setTargetAngle(double rad);            // set target in radians
    int64_t getFullTurns() const { return fullTurns; } // number of full rotations

private:
    uint8_t canId;
    uint32_t lastEncoder = 0;
    int64_t accumulatedCounts = 0;
    int64_t fullTurns = 0;
    double speedRadPerSec = 0.0;
    double accumulatedAngle = 0.0;
    double angleForROS = 0.0;   // wrapped 0..2π for ROS
    double gearRatio = 1.0;
    uint32_t targetEncoder = 0;

    std::chrono::steady_clock::time_point lastTime;

    void putWord(uint16_t v, uint8_t* result);
    void putPosition(uint32_t v, uint8_t* result);
    uint8_t checksum(const uint8_t* data, size_t len);
    uint32_t getValue(const uint8_t* data, uint8_t length);
    struct can_frame pack(const uint8_t* data, size_t len);
};
