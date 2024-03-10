# AMR Final Project

This repository contains the source code to the final project at the AMR.

## Usage

### Prerequisites

In order to use this project, make sure the following prerequisites are met:

1. OS: Ubuntu 22.04 LTS
2. Python (tested on Python 3.10.12)
3. ROS2 (use [these](https://gist.github.com/Elektra-V/74e241c97843efe6a5a0cc8e60067bca) Bash scripts)
4. VSCode, alongside [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python) and [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) extensions

Next, create a `.settings.json` file in `.vscode` folder:

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages",
        "/opt/ros/humble/local/lib/python3.10/dist-packages"
    ],
    "ros.distro": "humble"
}
```

This will ensure access to autocompletion and source code analysis, and it will configure the ROS extension. If needed, reload the window.

Next, setup a virtual environment and install NumPy:
```bash
python3 -m venv venv
source venv/bin/activate

# Install NumPy
pip install numpy
```

To use the project, source ROS and run the `main.py`:

```bash
source /opt/ros/humble/setup.bash
python3 main.py
```

### Running Robile in simulation

In order to run the Robile robot in a simulation, enter the following commands in a new terminal window:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
```

In order to control the Robile with your keyboard, in another terminal window, enter the following:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Testing

In order to run tests (they should all be in the `test` folder), you will need to set the `PYTHONPATH` variable in order for Python to be able to correctly resolve module imports.

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="${pwd}:$PYTHONPATH"
```

Do not use the `unittest` library module, instead, run test files as normal Python programs.
