#include "motor_controller.h"

MotorController::MotorController(MotorDriver &motor, HwRotaryEncoder &encoder)
    : motor(motor), encoder(encoder) {}

MotorController::~MotorController() {}

void MotorController::setPositionLimits(int32_t max_pos, int32_t min_pos)
{
    if (max_pos < min_pos)
    {
        this->max_pos = min_pos;
        this->min_pos = max_pos;
    }
    else
    {
        this->max_pos = max_pos;
        this->min_pos = min_pos;
    }
}

void MotorController::setTargetPosition(int32_t target_position)
{
    if (target_position > max_pos)
    {
        this->target_position = max_pos;
    }
    else if (target_position < min_pos)
    {
        this->target_position = min_pos;
    }
    else
    {
        this->target_position = target_position;
    }
    control_mode = CONTROL_POSITION;

    onTargetReachCalled = false;
    onMotorStallCalled = false;
}

void MotorController::setCurrentPosition(int32_t current_position)
{
    encoder.writeAbs(current_position);
    this->current_position = current_position;
}

void MotorController::setGain(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void MotorController::setPwm(float control_signal)
{
    this->control_signal = control_signal;
    control_mode = CONTROL_PWM;
    onTargetReachCalled = false;
    onMotorStallCalled = false;
}

void MotorController::printDebug() const
{
    switch (debug_option)
    {
    case DEBUG_OFF:
        return;
    case DEBUG_CONTROL_LOOP:
        if (control_mode == CONTROL_PWM)
        {
            Serial.printf("PWM mode, current_pos:%ld, current_speed:%6.3f, pwm:%3.3f", current_position, current_speed, control_signal);
        }
        else if (control_mode == CONTROL_POSITION)
        {
            Serial.printf("Position mode, target_pos:%ld, current_pos:%ld, error:%3.3f, integral:%3.3f, derivative:%3.3f, control_signal:%3.3f",
                          target_position, current_position, error, error_integral, error_derivative, control_signal);
            Serial.printf(", speed:%6.3f", current_speed);
        }
        break;
    case DEBUG_EVENT:
        if (motor_stalled)
        {
            Serial.print("Motor Stalled");
        }
        if (target_reached)
        {
            Serial.print("Target Reached");
        }
        break;
    case DEBUG_CURRENT:
        if (motor.hasCurrentPin())
        {
            Serial.printf("Current: %d (%.3fmA)", motor.getCurrent(), motor.getCurrent_mA());
        }
        break;
    default:
        break;
    }
    Serial.println();
}

void MotorController::motorTaskWrapper(void *parameter)
{
    static_cast<MotorController *>(parameter)->motorTask();
}

void MotorController::checkMotorStall()
{
    static uint32_t stall_start_tick = 0;

    if (target_reached || current_speed != 0 || control_signal == 0)
    {
        motor_stalled = false;
        stall_start_tick = 0;
        return;
    }

    if (stall_start_tick == 0)
    {
        stall_start_tick = millis();
    }

    if (millis() - stall_start_tick > stall_threshold_ms)
    {
        motor_stalled = true;
    }
}

void MotorController::checkTargetReach()
{
    target_reached = abs(target_position - current_position) < position_tolerance;
}

void MotorController::pidPositionControl()
{
    error = target_position - current_position;

    if (Ki != 0)
    {
        error_integral += error;
        error_integral = fmin(fmax(error_integral, -abs(1 / Ki)), abs(1 / Ki)); // anti-windup
    }

    static float prev_error = 0;
    error_derivative = error - prev_error;
    prev_error = error;

    control_signal = Kp * error + Ki * error_integral + Kd * error_derivative;
    control_signal = fmin(fmax(control_signal, -1), 1); // normalize control_signal to [-1,1]

    motor.runMotor(control_signal);
}

void MotorController::motorTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        current_speed = encoder.read(); // needs to run at a fixed interval
        current_position = encoder.readAbs();

        if (control_mode == CONTROL_PWM)
        {
            motor.runMotor(control_signal);
        }
        else if (control_mode == CONTROL_POSITION)
        {
            pidPositionControl();
        }

        checkTargetReach();
        checkMotorStall();

        if (target_reached && onTargetReach && !onTargetReachCalled)
        {
            onTargetReachCalled = true;
            onTargetReach();
        }

        if (motor_stalled && onMotorStall && !onMotorStallCalled)
        {
            onMotorStallCalled = true;
            onMotorStall();
        }

        printDebug();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void MotorController::start(uint8_t priority)
{
    xTaskCreate(motorTaskWrapper, "motorTask", 2048, this, priority, &motorTaskHandle);
}

void MotorController::pause()
{
    if (motorTaskHandle != NULL)
    {
        vTaskDelete(motorTaskHandle);
        motorTaskHandle = NULL;
    }
}

void MotorController::setOnMotorStall(MotorEventCallback callback)
{
    onMotorStall = callback;
}

void MotorController::setOnTargetReach(MotorEventCallback callback)
{
    onTargetReach = callback;
}
