#include "PDL_Motor_Controller.h"

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
}

void MotorController::setPositionTolerance(uint32_t position_tolerance)
{
    this->position_tolerance = position_tolerance;
}

void MotorController::setStallThreshold(uint32_t stall_threshold_ms, float stall_threshold_dutycycle)
{
    this->stall_threshold_ms = stall_threshold_ms;
    this->stall_threshold_dutycycle = stall_threshold_dutycycle;
}

int32_t MotorController::getCurrentPosition() const
{
    return current_position;
}

void MotorController::setCurrentPosition(int32_t current_position)
{
    encoder.writeAbs(current_position);
    this->current_position = current_position;
}

float MotorController::getCurrentSpeed() const
{
    return current_speed;
}

bool MotorController::isMotorStalled() const
{
    return motor_stalled;
}

bool MotorController::isTargetReached() const
{
    return target_reached;
}

void MotorController::setGain(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void MotorController::setLoopDelay(uint32_t loop_delay_ms)
{
    xFrequency = pdMS_TO_TICKS(loop_delay_ms);
}

void MotorController::setDebug(uint8_t debug)
{
    debug_option = debug;
}

void MotorController::setPwm(float control_signal)
{
    this->control_signal = control_signal;
    control_mode = CONTROL_PWM;
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
    static bool stall_flag = false; // flag to prevent multiple stall events
    // Serial.printf("current_speed: %f, control_signal: %f, stall_threshold_dutycycle: %f\n", current_speed, fabs(control_signal), stall_threshold_dutycycle);
    if ((current_speed == 0) && (fabs(control_signal) > stall_threshold_dutycycle))
    {
        // Serial.println("stall");
        if (stall_start_tick == 0)
        {
            stall_start_tick = millis();
            // Serial.println("stall detected");
        }
        else if (millis() - stall_start_tick > stall_threshold_ms)
        {
            motor_stalled = true;
            // Serial.println("stall registered");
            if (!stall_flag && onMotorStall)
            {
                // Serial.println("stall event");
                onMotorStall();
                stall_flag = true;
            }
        }
    }
    else
    {
        motor_stalled = false;
        stall_start_tick = 0;
        stall_flag = false;
        // Serial.println("no stall");
    }
}

void MotorController::checkTargetReach()
{
    static bool target_reached_flag = false;
    if (abs(target_position - current_position) < position_tolerance)
    {
        target_reached = true;
        if (!target_reached_flag && onTargetReach != NULL)
        {
            onTargetReach();
            target_reached_flag = true;
        }
    }
    else
    {
        target_reached = false;
        target_reached_flag = false;
    }
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

    static float prev_control_signal = 0;

    // Calculate the change in control signal
    float delta_control_signal = control_signal - prev_control_signal;
    constexpr float RATE_LIM = 0.1;

    // Check if the control signal is accelerating (increasing in magnitude)
    if (control_signal >= 0.9 && delta_control_signal > RATE_LIM)
    {
        control_signal = prev_control_signal + RATE_LIM;
    }
    else if (control_signal <= -0.9 && delta_control_signal < -RATE_LIM)
    {
        control_signal = prev_control_signal - RATE_LIM;
    }

    // Update the previous control signal
    prev_control_signal = control_signal;

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
            checkTargetReach();
        }

        checkMotorStall();

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
