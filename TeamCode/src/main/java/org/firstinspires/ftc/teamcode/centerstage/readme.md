# Shmulik.java
**Robot Control**

The class enables control of the robot's movement using a gamepad. It supports the following actions:
- Forward and backward movement.
- Strafing left and right.
- Rotating or turning.

**Orientation Tracking**

The robot's orientation can be tracked using onboard sensors. The class provides the ability to reset the robot's orientation to a specific angle.

**AprilTag Integration**

This class integrates with an AprilTag detection system, allowing the robot to detect and track AprilTags in its environment, and to align toward them. It provides telemetry data related to the detected AprilTag's:
- X, Y, and Z coordinates.
- Roll, pitch, and yaw angles.

## Usage

To use the `Shmulik` class, follow these steps:

1. **Hardware Setup**: Ensure that your robot is correctly configured with the required hardware, including motor controllers, servos, and a webcam named "cam."

2. **Initialization**: In the `runOpMode()` method, the robot's hardware components and vision system are initialized using the hardware map. Ensure that the hardware map correctly references your robot's components.

3. **TeleOperation**: The primary teleOperation loop is where you control the robot using the gamepad. Key control features include:
    - Left stick for forward/backward and left/right motion.
    - Right stick for turning.
    - D-pad for fine adjustments in orientation.

4. **Orientation Reset**: Pressing the "X" button on the gamepad resets the robot's orientation to 0 degrees, which can be useful for aligning the robot's reference heading.

5. **AprilTag Integration**: If the robot encounters an AprilTag, telemetry data related to the detected tag's position and orientation will be displayed. This information can aid in navigation and object tracking.

# **DriveClass**

## **Purpose**
The primary purpose of this class is to control the movement and positioning of a robot on the field. It manages the robot's motors, sensors, and integration with an Inertial Measurement Unit (IMU) to perform precise movements and navigate the game field.

## **Class Members**

### **Constants**
- `tile`: A constant representing the distance of one tile on the field (0.6 units).

### **Flags**
- `USE_ENCODERS`, `USE_BRAKE`, `USE_DASHBOARD_FIELD`: Constants used as flags for configuring the behavior of the robot, like using encoders, enabling braking, or displaying the robot's position on a dashboard.

### **Member Variables**
- Various private member variables to store information about motors, IMU, robot type, starting position, movement parameters, and more.

### **Enums**
- `ROBOT`: An enum representing different robot types (e.g., SCORPION, COBALT).
- `DriveMode`: An enum representing different drive modes (LEFT and RIGHT) with associated origin and direction vectors.

### **Constructor**
- The class has a constructor that initializes various parameters, including the robot type, starting position, and drive mode.

### **Methods**

- `init(HardwareMap hw)`: Initializes motors, sets their directions, and configures their modes and behaviors based on the specified parameters.

- `initIMU(HardwareMap hw)`: Initializes the IMU (Inertial Measurement Unit) sensor, calibrates it, and configures its integration with the robot's movement.

- `setPower(double forward, double turn, double strafe)`: Sets the power levels for the robot's motors to control forward, turn, and strafe movements.

- `setPowerOriented(double y, double x, double turn, boolean fieldOriented)`: Sets the power levels for the robot's motors while considering field orientation if enabled.

- `stopPower()`: Stops all robot motors by setting their power levels to zero.

- `resetOrientation(double angle)`: Resets the robot's orientation to a specified angle.

- `getHeading()`: Retrieves the current heading (orientation) of the robot.

- `getDeltaHeading(double target)`: Calculates the difference in heading between the robot's current orientation and a target orientation.

- `getForwardDistance()`: Calculates and returns the distance the robot has traveled in the forward direction using encoder ticks.

- `getPosX()` and `getPosY()`: Retrieve the current X and Y positions of the robot on the field.

- `goToLocation(Location location, double power, double targetHeading, double tolerance, double timeout)`: Moves the robot to a specified location on the field using PID control.

- `drive(double forward, double sideward, double targetPower, double targetAngle, boolean fieldOriented, double tolerance)`: Controls the robot's movement using PID control, taking into account forward, sideward, target power, target angle, field orientation, and tolerance.

- `hoverBoardMode()`: Implements a "hoverboard" mode to correct the robot's orientation when it deviates from a target angle.

The `DriveClass` is a crucial component of a robotics program, providing control and navigation capabilities for a robot on the game field.
# **Input**

## **Class Structure**

- `Input` class contains the following fields:
    - `private static Gamepad gamepad1, gamepad2`: These are instances of the `Gamepad` class, representing two game controllers. These objects will store the input from the physical game controllers.
    - `private static Toggle[] toggles`: An array of `Toggle` objects that represent toggle switches for various buttons on the game controllers. This array is initially set to `null`.

## **Methods**

- `public static void updateControls(Gamepad gamepad1_, Gamepad gamepad2_)`: This method updates the references to the `gamepad1` and `gamepad2` objects with new instances passed as arguments.

- `private static Toggle[] getToggles()`: This method returns the `toggles` array. If it's `null`, it initializes the array with `Toggle` objects representing various button mappings and then returns it.

- `public static boolean GetKeyPressed(KeyCode key)`: This method checks if a specific `KeyCode` is currently pressed and returns a boolean value.

- `public static boolean GetKeyClicked(KeyCode key)`: This method checks if a specific `KeyCode` has been clicked (pressed and released) and returns a boolean value.

- `public static boolean GetKeyReleased(KeyCode key)`: This method checks if a specific `KeyCode` has been released and returns a boolean value.

## **Inner Class `KeyCode`**

The `KeyCode` class is an inner class of `Input` and is used to represent individual buttons on the game controllers.

- It has a private field `returnFunc` of type `Func<Boolean>`, which is a functional interface used to retrieve the button value.

- The constructor of `KeyCode` takes a `Func<Boolean>` as an argument to initialize `returnFunc`.

- The `KeyCode` class defines constants for various buttons on both `gamepad1` and `gamepad2`, using lambdas to map the button value to the corresponding `KeyCode` constant.

## **Example Usage**

- You can use this `Input` class to check the value of gamepad buttons and toggle switches to control your robot in a FIRST Tech Challenge competition.
```java
  if (Input.GetKeyPressed(Input.KeyCode.Gamepad1A)){
      // Do Something
  }
```
# **Toggle**

## **Constructor**

- `Toggle()`: Default constructor with no initial value.
- `Toggle(boolean initialState)`: Constructor with an initial value parameter.
- `Toggle(Func<Boolean>)`: Constructor with a func arg to tell it what logic will mean it's pressed.
- `Toggle(KeyCode)`: Constructor with a KeyCode value to tell it what gamepad button it's mapped to.
## **Methods**

- `update(boolean input)`: This method updates the toggle based on the given input.
- `update()`: This method updates the toggle based on the KeyCode mapping or the Func it was given in the constructor.
- `set(boolean input)`: Sets the toggle value. It can be used to directly set the value without relying on input changes.

- `toggle()`: Toggles the current value of the toggle and returns the new value.

- `isPressed()`: Checks if the toggle is currently pressed.

- `isChanged()`: Checks if the toggle value has changed since the last update.

- `getState()`: Returns the current value of the toggle.

- `isClicked()`: Checks if the toggle has changed and is currently pressed (clicked).

- `isReleased()`: Checks if the toggle has changed and is not currently pressed (released).

## **Location**

## **Constructors**

- `Location(double x, double y)`: Initializes a location with the specified x and y coordinates.

- `Location(double x, double y, double angle)`: Initializes a location with the specified x and y coordinates and an angle.

- `Location(Location location)`: Creates a copy of an existing location.

- `Location(Location location, double xOffset, double yOffset, double angleOffset)`: Creates a new location based on an existing location with added offsets.

## **Methods**

## **Coordinate Flipping**

- `flipX()`: Inverts the x-coordinate.

- `flipY()`: Inverts the y-coordinate.

- `flipAngle()`: Inverts the angle.

## **Offset Manipulation**

- `offsetY(double offset)`: Returns a new location with an offset added to the y-coordinate.

- `offsetX(double offset)`: Returns a new location with an offset added to the x-coordinate.

- `stayOnX(DriveClass drive)`: Returns a new location with the x-coordinate set to the current x-position from a `DriveClass` instance.

- `stayOnY(DriveClass drive)`: Returns a new location with the y-coordinate set to the current y-position from a `DriveClass` instance.

## **Arithmetic Operations**

- `add(Location location)`: Adds the x and y coordinates of another location to the current location.

- `addX(double x)`: Adds a specified value to the x-coordinate.

- `addY(double y)`: Adds a specified value to the y-coordinate.

- `subtract(Location location)`: Subtracts the x and y coordinates of another location from the current location.

- `subtractX(double x)`: Subtracts a specified value from the x-coordinate.

- `subtractY(double y)`: Subtracts a specified value from the y-coordinate.

- `multiply(float num)`: Multiplies both the x and y coordinates by a specified value.

- `multiplyX(double x)`: Multiplies the x-coordinate by a specified value.

- `multiplyY(double y)`: Multiplies the y-coordinate by a specified value.

