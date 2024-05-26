#include <Arduino.h>
#include "PDL_Motor_Controller.h"

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
    motor_controller.setLoopDelay(50);
    motor_controller.setOnTargetReach(posReachedCallback);
    motor_controller.setOnMotorStall(motorStalledCallback);
    motor_controller.start();
}

void loop()
{
    // Check for serial input and set the target position
    if (Serial.available())
    {
        int targetPosition = Serial.parseInt();
        if (Serial.read() == '\n')
        { // Ensure the command is complete
            motor_controller.setTargetPosition(targetPosition);
            Serial.print("New target position set: ");
            Serial.println(targetPosition);
        }
        else
        {
            Serial.println("Invalid input. Please enter a valid integer.");
            while (Serial.available())
                Serial.read(); // Clear the buffer
        }
    }

    // Adding a small delay to avoid flooding the serial communication
    delay(100);
}
