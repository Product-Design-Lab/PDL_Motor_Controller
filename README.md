 # MotorController Library
 
 This library provides advanced functionality to control DC motors using PWM, position control, and stall detection. It depends on the MotorDriver library and the hardware encoder from the SEEED Xiao BLE Arduino package. This module is tested on the SEEED Xiao BLE board with the FreeRTOS framework.
 
 ## Installation
 
 1. Download the library from the GitHub repository: [PDL_Motor_Controller](https://github.com/Product-Design-Lab/PDL_Motor_Controller).
 2. Unzip the library and place the `MotorController` folder in your Arduino `libraries` directory.
 3. Restart the Arduino IDE.
 
 ### Installing Dependencies
 
 This library depends on the following:
 
 1. [MotorDriver](https://github.com/Product-Design-Lab/PDL_Motor_Driver) library. Please ensure it is installed in your Arduino libraries directory.
 2. Hardware encoder provided by the SEEED Xiao BLE Arduino package.
 
 #### Installing MotorDriver Library
 
 1. Download the MotorDriver library from [here](https://github.com/Product-Design-Lab/PDL_Motor_Driver).
 2. Unzip the library and place the `MotorDriver` folder in your Arduino `libraries` directory.
 3. Restart the Arduino IDE.
 
 #### Installing SEEED Xiao BLE Arduino Package
 
 1. Open the Arduino IDE.
 2. Go to `File` > `Preferences`.
 3. In the "Additional Boards Manager URLs" field, add the following URL: `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`.
 4. Go to `Tools` > `Board` > `Boards Manager`.
 5. Search for `Seeed nRF52 Boards` and install the package.
 6. Restart the Arduino IDE.
 
 For more information, refer to the official SEEED Xiao BLE documentation: [Seeed Studio XIAO BLE](https://wiki.seeedstudio.com/XIAO_BLE/).
 
 ## Usage
 
 ### Including the Library
 
 ```cpp
 #include <PDL_Motor_Controller.h>
 ```
 
 ### Creating an Instance
 
 ```cpp
 MotorDriver motor;
 HwRotaryEncoder encoder;
 MotorController motorController(motor, encoder);
 ```
 
 ### Setting Pins
 
 Different motor drivers have different pin definitions. Ensure you set the appropriate pins for the motor driver you are using. Refer to the example sketches in the `examples` folder for specific pin setups.
 
 ### Running the Motor
 
 ```cpp
 motorController.setPwm(control_signal); // control_signal value between -1 and 1
 ```
 
 ## API Reference
 
 - `MotorController(MotorDriver &motor, HwRotaryEncoder &encoder)`: Constructor for the MotorController class.
 - `~MotorController()`: Destructor for the MotorController class.
 - `void setPositionLimits(int32_t max_pos, int32_t min_pos)`: Set the position limits using encoder count values.
 - `void setTargetPosition(int32_t target_position)`: Set the target position.
 - `void setPositionTolerance(uint32_t position_tolerance)`: Set the position tolerance.
 - `void setStallThreshold(uint32_t stall_threshold_ms)`: Set the stall threshold in milliseconds.
 - `int32_t getCurrentPosition() const`: Get the current position.
 - `void setCurrentPosition(int32_t current_position)`: Set the current position.
 - `int32_t getCurrentSpeed() const`: Get the current speed.
 - `bool isMotorStalled() const`: Check if the motor is stalled.
 - `bool isTargetReached() const`: Check if the target position is reached.
 - `void setPwm(float control_signal)`: Set the PWM control signal.
 - `void setGain(float Kp, float Ki = 0.0, float Kd = 0.0)`: Set the PID gains.
 - `void setLoopDelay(uint32_t delay_ms)`: Set the loop delay in milliseconds.
 - `void setDebug(uint8_t debug)`: Enable or disable debug mode.
 - `void printDebug() const`: Print debug information.
 - `void start(uint8_t priority = 1)`: Start the motor controller task with a given priority.
 - `void pause()`: Pause the motor controller task.
 - `void setOnMotorStall(MotorEventCallback callback)`: Set the callback for motor stall events.
 - `void setOnTargetReach(MotorEventCallback callback)`: Set the callback for target reach events.
 
 ## License
 
 This library is licensed under the MIT License. See the `LICENSE` file for more details.
