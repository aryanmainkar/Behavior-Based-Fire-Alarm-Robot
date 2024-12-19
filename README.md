# Behavior-Based-Fire-Alarm-Robot

Robot Navigation and Calibration
================================

Overview
--------

This project involves the development of a compact robot with advanced fire-suppression mechanisms, navigation strategies, and calibration methods to ensure precise operations. The repository includes the code and documentation to replicate the design and functionality of the robot.

* * * * *

1\. Robot Design
----------------

### Design Choices

-   **Base Configuration:** The robot's base remains similar to the previous version but includes significant enhancements.
-   **Sensors and Components:**
    -   Added a **color sensor** at the rear, with the wheel repositioned in front of it.
    -   Integrated an **ultrasonic sensor** on the left to measure wall distances.
    -   Equipped the right side with a custom-made robotic arm, the **Fire Sweeper**, for fire suppression tasks.
    -   A **front switch** to detect frontal collisions.
    -   A **counterweight** on the left to balance the Fire Sweeper arm.

> **TODO:** Add a **picture of the robot** to visualize the design.

* * * * *

2\. Navigation Strategy
-----------------------

### Wall-Following and Escape Mechanisms

-   **Initial Behavior:** The robot begins with a wall-following strategy.
-   **Escape Scenarios:**
    1.  **Frequent Collisions:** If the bumper is hit 20 times, the robot switches to a Roomba-like random movement strategy.
    2.  **Timeout:** If no collision occurs for 1 minute, the robot switches to random movement.

These mechanisms prevent the robot from getting stuck or trapped in infinite loops.

* * * * *

3\. Calibration Strategy
------------------------

### Travel Distance Calibration

-   **Wheel Diameter:** The wheels have a diameter of 2.2 inches.
-   **Distance Formula:**

    ```
    TraveledDistance = Diameter * pi * (MotorAngle / 360)

    ```

-   Using the encoder data, the robot accurately calculates travel distance.

### Kp Controller for Movement

-   Proportional controller (`Kp`) for linear movement:

    ```
    Kp * (TargetDistance - TraveledDistance)

    ```

### Turning with IMU Sensor

-   Uses the hub's IMU sensor and includes an integral term to handle low-friction surfaces:

    ```
    Kp * (TargetRotation - TraveledRotation) + Ki * Σ(TargetRotation - TraveledRotation)

    ```

### Noise Filtering for Color Detection

-   Samples the color sensor at 20Hz, maintaining a queue of 10 readings.
-   Detects target color only if all 10 samples match.

* * * * *

4\. Code Overview
-----------------

### Main Robot Code

```
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait
from queue import FixedLengthQueue
import urandom

hub = PrimeHub()
ultra = UltrasonicSensor(Port.D)
LeftMotor = Motor(Port.A, Direction.CLOCKWISE)
RightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
SweepMotor = Motor(Port.F, Direction.CLOCKWISE)
color_sense = ColorSensor(Port.C)
button = ForceSensor(Port.E)

# Add additional functions here for movement, turning, and fire suppression

```

### Fixed-Length Queue for Color Detection

```
class FixedLengthQueue:
    def __init__(self, max_size):
        self.queue = []
        self.max_size = max_size

    def enqueue(self, item):
        if len(self.queue) >= self.max_size:
            self.queue.pop(0)
        self.queue.append(item)

    def all_true(self):
        return len(self.queue) == self.max_size and all(self.queue)

    def true_percentage(self):
        true_count = sum(1 for item in self.queue if item)
        return (true_count / len(self.queue)) * 100 if self.queue else 0

```

* * * * *

5\. Fire Suppression Mechanism
------------------------------

### Fire Detection

-   Detects fire using the color sensor by identifying red objects.
-   Queues consecutive detections to confirm the presence of fire.

### Extinguishing Fire

-   A motorized fan is deployed using a winch mechanism.
-   End stops are defined with LEGO Technic pins to control the fan's position.
-   The extinguishing sequence includes spinning the fan and moving it in a sweeping motion.

* * * * *

6\. Navigation and Random Movement
----------------------------------

### Wall Following

-   Maintains a specific distance from the wall using the ultrasonic sensor.

### Random Movement

-   Activated based on collision frequency or timeout.
-   Utilizes random angles for turning to escape confined spaces.

* * * * *

7\. How to Run
--------------

1.  Clone this repository.
2.  Install dependencies.
3.  Connect the robot hardware.
4.  Run the main Python script:

    ```
    python3 main_robot.py

    ```

* * * * *

8\. Future Enhancements
-----------------------

-   **Dynamic Mapping:** Real-time mapping of obstacles.
-   **Enhanced Algorithms:** Experiment with alternative pathfinding techniques.
-   **Improved Sensors:** Add more sensors for better environmental awareness.

* * * * *

9\. License
-----------

This project is licensed under the MIT License.

* * * * *

10\. Contact
------------

For questions or contributions, open an issue or email [<your_email@example.com>].
