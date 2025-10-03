#include "arctos_hardware_interface/ServoManager.hpp"
#include <cstring>
#include <chrono>
#include <cmath>

// Constructor
ServoManager::ServoManager(uint8_t id, double ratio)
    : canId(id), lastEncoder(0), accumulatedCounts(0), fullTurns(0),
      speedRadPerSec(0.0), accumulatedAngle(0.0), gearRatio(ratio), targetEncoder(0) {}

// --- Helpers ---
void ServoManager::putWord(uint16_t v, uint8_t* result) {
    result[0] = (v >> 8) & 0xFF;
    result[1] = v & 0xFF;
}

void ServoManager::putPosition(uint32_t v, uint8_t* result) {
    result[0] = (v >> 16) & 0xFF;
    result[1] = (v >> 8) & 0xFF;
    result[2] = v & 0xFF;
}

uint8_t ServoManager::checksum(const uint8_t* data, size_t len) {
    uint16_t sum = canId;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum & 0xFF;
}

uint32_t ServoManager::getValue(const uint8_t* data, uint8_t length) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < length; i++) value = (value << 8) | data[i];
    return value;
}

struct can_frame ServoManager::pack(const uint8_t* data, size_t len) {
    can_frame frame{};
    frame.can_id = canId;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);
    return frame;
}

// --- Motion commands ---
struct can_frame ServoManager::runMotorSpeedMode(uint8_t dir, uint16_t speed, uint8_t accel) {
    uint8_t speedByte = static_cast<uint8_t>(speed / 8);
    uint8_t byte2 = (dir ? 0x80 : 0x00) | 0x01;
    uint8_t data[5]{CMD::MOTOR::SPEED, byte2, speedByte, accel, 0};

    uint16_t sum = canId;
    for (int i = 0; i < 4; ++i) sum += data[i];
    data[4] = sum & 0xFF;

    return pack(data, 5);
}

struct can_frame ServoManager::absoluteMotion(uint16_t speed, uint8_t accel, uint32_t position) {
    uint8_t data[6]{
        CMD::MOTOR::ABSOLUTE_AXIS,
        accel,
        static_cast<uint8_t>(speed >> 8),
        static_cast<uint8_t>(speed & 0xFF),
        static_cast<uint8_t>(position >> 8),
        static_cast<uint8_t>(position & 0xFF)
    };
    return pack(data, 6);
}

struct can_frame ServoManager::absoluteMotionRad(double angleRad, uint16_t speed, uint8_t accel) {
    double motorRad = angleRad * gearRatio;
    uint32_t encoderValue = static_cast<uint32_t>((motorRad / TWO_PI) * ENCODER_CPR) % ENCODER_CPR;
    uint8_t data[6]{
        CMD::MOTOR::ABSOLUTE_AXIS,
        accel,
        static_cast<uint8_t>(speed >> 8),
        static_cast<uint8_t>(speed & 0xFF),
        static_cast<uint8_t>(encoderValue >> 8),
        static_cast<uint8_t>(encoderValue & 0xFF)
    };
    return pack(data, 6);
}

struct can_frame ServoManager::enableServo(bool on) {
    uint8_t data[3]{CMD::CONTROL::ENABLE, static_cast<uint8_t>(on ? 1 : 0), 0};
    data[2] = checksum(data, 2);
    return pack(data, 3);
}

struct can_frame ServoManager::stopMotor() {
    uint8_t val = CMD::MOTOR::STOP;
    uint8_t data[2]{val, checksum(&val, 1)};
    return pack(data, 2);
}

// --- Read commands ---
struct can_frame ServoManager::readEncoderAbsolute() {
    uint8_t val = CMD::READ::ENCODER_ABSOLUTE;
    uint8_t data[2]{val, checksum(&val, 1)};
    return pack(data, 2);
}

struct can_frame ServoManager::readEncoderCarry() {
    uint8_t val = CMD::READ::ENCODER_CARRY;
    uint8_t data[2]{val, checksum(&val, 1)};
    return pack(data, 2);
}

struct can_frame ServoManager::readSpeed() {
    uint8_t val = CMD::READ::SPEED;
    uint8_t data[2]{val, checksum(&val, 1)};
    return pack(data, 2);
}

// --- Set commands ---
struct can_frame ServoManager::setWorkingCurrent(uint16_t current) {
    uint8_t data[3]{CMD::SET::WORKING_CURRENT, static_cast<uint8_t>(current >> 8), static_cast<uint8_t>(current & 0xFF)};
    return pack(data, 3);
}

struct can_frame ServoManager::setHoldingCurrent(uint8_t percent) {
    uint8_t data[2]{CMD::SET::HOLDING_CURRENT, percent};
    return pack(data, 2);
}

// --- Response parsing ---
ServoManager::ResponseData ServoManager::parseResponse(const struct can_frame& frame) {
    ResponseData resp;

    if (frame.can_dlc < 4) return resp; // expect at least 4 bytes

    uint8_t cmd = frame.data[0];
    switch (cmd) {
        case CMD::READ::ENCODER_CARRY: {
            resp.type = TYPE_ENCODER;
            int32_t carry = static_cast<int8_t>(frame.data[1]);  // signed 8-bit carry
            uint16_t value = (frame.data[2] << 8) | frame.data[3];
            uint32_t combined = (static_cast<int32_t>(carry) << 14) | value;
            resp.value = combined;
            updateEncoder(combined);
            break;
        }
        case CMD::READ::ENCODER_ABSOLUTE: {
            resp.type = TYPE_ENCODER;
            uint16_t value = (frame.data[1] << 8) | frame.data[2];
            updateEncoder(value);
            break;
        }
        case CMD::READ::SPEED: {
            resp.type = TYPE_SPEED;
            int16_t rpm = static_cast<int16_t>((frame.data[1] << 8) | frame.data[2]);
            updateSpeedFromCAN(rpm);
            break;
        }
        default:
            resp.type = TYPE_NONE;
            break;
    }

    return resp;
}

// --- Encoder update ---
void ServoManager::updateEncoder(uint32_t rawValue) {
    int32_t delta = int32_t(rawValue) - int32_t(lastEncoder);
    lastEncoder = rawValue;

    accumulatedCounts += delta;
    accumulatedAngle = (double(accumulatedCounts) / ENCODER_CPR) * TWO_PI / gearRatio;
}

double ServoManager::getAbsoluteAngle() const {
    return accumulatedAngle;
}

void ServoManager::setTargetAngle(double rad) {
    targetEncoder = static_cast<uint32_t>((rad * gearRatio / TWO_PI) * ENCODER_CPR) % ENCODER_CPR;
}

void ServoManager::updateSpeedFromCAN(int16_t rpm) {
    speedRadPerSec = double(rpm) * TWO_PI / 60.0;
}
