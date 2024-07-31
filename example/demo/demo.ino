#include "PDL_Motor_Controller.h"
#include <Arduino.h>

MotorDriver mp6550;
HwRotaryEncoder encoder;
MotorController motor_controller(mp6550, encoder); // Pass references here

void posReachedCallback()
{
    Serial.println("MAIN: Position reached");
}

void motorStalledCallback()
{
    Serial.println("MAIN: Motor stalled");
}

void setup()
{
    // Initialize Serial Communication
    Serial.begin(115200);
    while (!Serial)
        ;

    // Initialize Motor Driver
    mp6550.setPwmPin(D8, D9);
    mp6550.setDirNoPin();
    mp6550.setEnableNoPin();
    mp6550.setVisenPin(A1);
    mp6550.setVisenSensitivity(5.0); // Breakout board gives 200 mV/A, so 5 A/V
    mp6550.setMaxPwm(255);
    mp6550.setDebug(false);

    // Initialize Rotary Encoder
    encoder.begin(D2, D3);
    encoder.start();

    // Configure Motor Controller
    motor_controller.setPositionLimits(20000, -20000);
    motor_controller.setGain(0.005);
    motor_controller.setDebug(MotorController::DEBUG_OFF);
    motor_controller.setLoopDelay(20);
    motor_controller.setOnTargetReach(posReachedCallback);
    motor_controller.setOnMotorStall(motorStalledCallback);
    motor_controller.setStallThreshold(100, 0.5);
    motor_controller.start();
}

void loop()
{
    // Check for serial input and set the target position
    if (Serial.available())
    {
        char command = Serial.read();
        int target = Serial.parseInt();
        int max_speed = Serial.parseInt();
        while (Serial.available())
            Serial.read(); // Clear the buffer

        if (max_speed <= 0)
        {
            Serial.printf("Invalid max_speed: %d, setting to 100\n", max_speed);
            max_speed = 100;
        }

        Serial.printf("Command: %c, Target: %d, max_speed: %d\n", command, target, max_speed);
        if (command == 'P' || command == 'p')
        {
            motor_controller.setMaxSpeed(max_speed);
            motor_controller.setTargetPosition(target);
            Serial.printf("Setting target position to %d\n", target);
        }
        else if (command == 'T' || command == 't')
        {
            float target_pwm = 0.001 * target;
            motor_controller.setPwm(target_pwm);
            Serial.printf("Setting PWM to %f\n", target_pwm);
        }
        else if (command == 's')
        {
            motor_controller.start();
        }
        else if (command == 'a')
        {
            motor_controller.pause();
        }
        else
        {
            Serial.printf("Invalid command: %c\n", command);
        }
    }

    // Adding a small delay to avoid flooding the serial communication
    delay(100);
}
