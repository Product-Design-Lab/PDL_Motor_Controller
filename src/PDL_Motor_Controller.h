#pragma once

#include "PDL_Motor_Driver.h"
#include "RotaryEncoder.h"
#include "FreeRTOS.h"
#include "task.h"

class MotorController
{
private:
    enum ControlMode
    {
        CONTROL_PWM,
        CONTROL_POSITION
    };
    ControlMode control_mode = CONTROL_PWM;

    MotorDriver &motor;
    HwRotaryEncoder &encoder;
    TaskHandle_t motorTaskHandle = NULL;
    int32_t max_pos = 0;
    int32_t min_pos = 0;
    int32_t target_position = 0;
    int32_t current_position = 0;

    uint32_t position_tolerance = 100; // position within this range is considered reached
    bool target_reached = false;
    bool onTargetReachCalled = false;

    uint32_t stall_threshold_ms = 500; // in ms
    bool motor_stalled = false;
    bool onMotorStallCalled = false;

    float current_speed = 0;
    float Kp = 0, Ki = 0, Kd = 0;
    float error = 0, error_integral = 0, error_derivative = 0;
    float control_signal = 0;
    TickType_t xFrequency = 10; // loop delay in Ticks, note that 1 Tick is not always 1 ms, refer to FreeRTOSconfig.h
    uint8_t debug_option = 0;

    static void motorTaskWrapper(void *parameter);
    void motorTask();
    void checkTargetReach();
    void checkMotorStall();
    void pidPositionControl();

public:
    enum DebugStatus
    {
        DEBUG_OFF,
        DEBUG_CURRENT,
        DEBUG_CONTROL_LOOP,
        DEBUG_EVENT,
        DEBUG_MAX,
    };

    MotorController(MotorDriver &motor, HwRotaryEncoder &encoder);
    ~MotorController();

    void setPositionLimits(int32_t max_pos, int32_t min_pos); // encoder count value
    void setTargetPosition(int32_t target_position);
    void setPositionTolerance(uint32_t position_tolerance);

    void setStallThreshold(uint32_t stall_threshold_ms);

    int32_t getCurrentPosition() const;
    void setCurrentPosition(int32_t current_position);
    float getCurrentSpeed() const;
    bool isMotorStalled() const;
    bool isTargetReached() const;

    void setPwm(float control_signal);
    void setGain(float Kp, float Ki = 0.0, float Kd = 0.0);
    void setLoopDelay(uint32_t delay_ms);
    void setDebug(uint8_t debug);
    void printDebug() const;

    void start(uint8_t priority = 1);
    void pause();

    // Motor Event callbacks
    using MotorEventCallback = void (*)();
    MotorEventCallback onMotorStall = NULL;
    MotorEventCallback onTargetReach = NULL;

    void setOnMotorStall(MotorEventCallback callback);
    void setOnTargetReach(MotorEventCallback callback);
};
