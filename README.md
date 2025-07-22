# Small Robot Arm

This project implements a 6-DOF (Degrees of Freedom) robotic arm with advanced control capabilities, including forward/inverse kinematics, trajectory planning, and velocity-based control.

## Features

- **Forward and Inverse Kinematics**: Based on Denavit-Hartenberg parameters
- **Trajectory Planning**: Linear functions with parabolic blends
- **Velocity-Based Control**: Jacobian-based velocity control for smooth motion
- **Multiple Motion Patterns**:
  - Circular motion
  - Figure-eight patterns
  - Square paths with rounded corners
  - Spiral trajectories
  - Zigzag patterns
- **3D Camera Integration**: Object detection and pick-and-place operations
- **Graceful Shutdown**: Signal handling for clean program termination
- **Real-time Visualization**: 3D plotting of robot movement

## Installation

### Prerequisites

- Python 3.6 or higher
- Arduino IDE (for firmware upload)
- USB connection to Arduino

### System Dependencies (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install python3-venv python3-dev
```

### Setup Virtual Environment

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/smallRobotArm.git
   cd smallRobotArm
   ```

2. Create a virtual environment:
   ```bash
   python3 -m venv myenv
   source myenv/bin/activate  # Linux/macOS
   ```

3. Install Python dependencies:
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

### Required Libraries

The project depends on the following Python packages:

- **numpy** (>=1.17.3, <2.0.0): For numerical computations
- **scipy** (>=1.6): For scientific computing
- **pyserial** (>=3.5): For serial communication with Arduino
- **spatialmath-python** (>=1.1.14): For spatial mathematics and transformations
- **pandas** (>=1.2.4): For data manipulation
- **matplotlib** (>=3.3.4): For plotting and visualization
- **roboticstoolbox-python** (==1.1.1): For robotics algorithms

### Arduino Setup

1. Connect the Arduino to your computer via USB
2. Upload the firmware from the `firmware` directory using Arduino IDE
3. Note the serial port (e.g., `/dev/ttyUSB0` or `COM3`)

### Verify Installation

Test the installation by running:
```bash
python -c "import roboticstoolbox as rtb; print(rtb.__version__)"
```

## Usage

### Basic Control

Run the main control script:
```bash
python small_robot_arm.py
```

For UI control:
```bash
python ui_for_smallrobotarm1.py
```

### Velocity Control

Run the Jacobian-based velocity control demo:
```bash
python jacobian_vel_ctrl.py
```

This demonstrates various motion patterns including:
- Constant velocity motion
- Circular paths
- Figure-eight patterns
- Square paths with rounded corners

### Pick and Place

For object detection and manipulation:
```bash
python pick_n_place.py
```

## Project Structure

- `robot_tools/`: Core functionality modules
  - `kinematics/`: Forward and inverse kinematics
  - `controller/`: Robot motion control
  - `serial/`: Communication with Arduino
  - `trajectory/`: Path planning and interpolation
  - `misc/`: Helper functions and utilities
- `examples/`: Example scripts demonstrating specific features
- `test/`: Test scripts and utilities
- `firmware/`: Arduino firmware for motor control

## Implementation Details

1. **Kinematics**: Uses standard DH parameters for a 6-DOF arm
2. **Control System**: Implements both joint-space and task-space control
3. **Communication**: Serial protocol for Arduino communication with acknowledgment system
4. **Motion Planning**: Linear functions with parabolic blends for smooth acceleration/deceleration

## Advanced Features

### Velocity Control

The system implements Jacobian-based velocity control:

```python
controller.velocity_control(
    robot=robot_model,
    q_init=initial_joint_angles,
    x_dot_func=lambda t: velocity_function(t),
    dt=0.05,
    duration=10.0
)
```

### Approach Pose Calculation

For object manipulation, the system can calculate approach poses:

```python
T_approach = controller.compute_approach_pose(
    T_object,
    approach_vector,
    offset=50  # mm
)
```

## References

- [Original YouTube Tutorial](https://www.youtube.com/watch?v=oFCUw1pXlnA)
- [Robotics Toolbox Documentation](https://petercorke.github.io/robotics-toolbox-python/)
- [Coursera Robotics Course](https://www.coursera.org/learn/robotics1/lecture/EddnO/7-3-gui-ji-gui-hua-shi-li-fang-fa-er)

## Future Development

- G-code interpreter for standard CNC-style commands
- Improved trajectory planning with obstacle avoidance
- Enhanced 3D vision system for object recognition
- Web-based control interface