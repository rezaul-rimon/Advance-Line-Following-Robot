# Advanced Line Following Robot

## Description
This project showcases an advanced line following robot designed for competition purposes. The robot is capable of detecting black lines on a white surface and vice versa. It can navigate through a maze, memorize the path, and find the shortest path back. Additionally, the robot includes features such as finding broken lines, obstacle avoidance, and the ability to navigate along a wall or in a cave when the line is broken. The project utilizes an Arduino Nano, TCRT5000 IR sensors, ultrasonic sensor, L298N motor driver, 3S LiPo battery, XL6009 boost converter, LM2596 buck converter, and other components.

## Features
- Line Following: The robot can detect and follow black lines on a white surface or vice versa.
- Maze Solving: It can navigate through a maze, memorize the path, and find the shortest path back to the starting point.
- Broken Line Detection: The robot can identify and navigate around broken or discontinuous lines.
- Obstacle Avoidance: Utilizing the ultrasonic sensor, the robot can detect obstacles and avoid collisions.
- Wall and Cave Following: When the line is broken, the robot can move along a wall or in a cave until it finds the line again.

## Hardware Requirements
- Arduino Nano or compatible board
- TCRT5000 IR sensors for line detection
- Ultrasonic sensor (e.g., HC-SR04) for obstacle avoidance
- L298N motor driver for controlling motors
- 12V Metal Geer Motors with Wheel
- 360* Custer Wheel
- 3S LiPo battery for power supply
- XL6009 boost converter (if necessary) for voltage regulation
- LM2596 buck converter (if necessary) for voltage regulation
- Motor chassis or wheels
- Connecting wires
- Other mechanical components as required for your specific robot design

## Software Requirements
- Arduino IDE (Integrated Development Environment)
- Libraries for controlling the motor driver, line detection sensors, and ultrasonic sensor (if applicable)

## Installation and Setup
1. Clone the repository to your local machine using the following command: git clone https://github.com/rezaul-rimon/Advance-Line-Following-Robot.git

2. Connect the required components, including TCRT5000 IR sensors, ultrasonic sensor, L298N motor driver, Arduino Nano, and power supply components, based on your robot's schematic or the provided connections in the code.

3. Open the Arduino IDE.

4. Open the Arduino sketch (.ino file) for the advanced line following robot.

5. Install any required libraries for controlling the motor driver, line detection sensors, and ultrasonic sensor.

6. Upload the sketch to the Arduino Nano.

7. Power on the robot and ensure all connections are properly made.

8. Calibrate and adjust the sensor readings and motor control settings as necessary, based on your robot's behavior and the specific environment it operates in.

## Usage
1. Power on the robot and place it on the line to be followed.

2. The robot will detect the line using the TCRT5000 IR sensors and begin following it.

3. To solve a maze, position the robot at the maze entrance, and it will navigate through the maze, memorize the path, and find the shortest path back to the starting point.

4. If the line is broken or discontinuous, the robot will use its broken line detection feature to navigate around the obstacle until the line is found again.

5. Utilizing the ultrasonic sensor, the robot will detect obstacles in its path and adjust its movement to avoid collisions.

6. If a wall or cave is encountered and the line is broken, the robot will continue moving along the wall or in the cave until the line is detected.

7. Observe the robot's behavior and make adjustments to the code or hardware as needed for optimal performance.

## Contributing
Contributions to this project are welcome! If you have any suggestions, improvements, or encounter any issues, feel free to open an issue or submit a pull request.

## License
[MIT License](LICENSE)
