# CSSR: Car Simulation System by ROS
**January 31, 2025 (Version 1.0)**
## Introduction
CSSR provides a simulation platform for testing the decision-making and planning system of autonomous vehicles.

## Software Architecture
The code is organized as follows:
- `src/CSSR/scripts/simmain.py` is used to run CSSR.
- `src/CSSR/scripts/simtest.py` is used to run the test program to provide control messages to CSSR.
- `src/CSSR/msg/Control.msg` defines the control message used in ROS communication.
- `src/CSSR/msg/Obstacle.msg` defines one obstacle in the field.
- `src/CSSR/msg/ObstacleVec.msg` defines the obstacle vector that combines all obstacle information in the field.
- `src/CSSR/msg/VehicleState.msg` defines the vehicle state used in ROS communication.

## Software Interfaces
The ROS message subscribed by CSSR is listed below:
- `/Control` is a `Control` message. CSSR uses this message to control the vehicle state in the simulation.  

The ROS messages published by CSSR are listed below:
- `/Obstacle` is an `ObstacleVec` message and combines all obstacle information in the field.
- `/VehicleState` is a `VehicleState` message and contains the state of the vehicle in the simulation.

The detailed definitions of the messages above can be found in the `src/CSSR/msg` folder.

**Therefore, the user needs to develop a program that can subscribe to the `/Obstacle` and `/VehicleState` messages published by CSSR, and publish `/Control` messages to control the car in the simulation.**

`src/CSSR/scripts/simtest.py` can publish `/Control` messages to test the efficiency of CSSR, but cannot receive the messages published by CSSR.

## Environment Setup
You can unzip the provided `CSSR.zip`, or download the code from the GitHub link below:
<br>
[https://github.com/myTristan/CSSR.git](https://github.com/myTristan/CSSR.git)
<br>
Then you can set up a virtual Python environment, and substitute `env` with your preferred environment name if you like.
```
cd CSSR
python -m venv env
source env/bin/activate
```
Then you can install CSSR dependencies:
```
pip install -r requirements.txt
```
Press `Ctrl + Alt + T` to open a new command window, and compile the program:
```
cd CSSR
catkin_make
```

## Run CSSR
Open a new command window in the `CSSR` folder, and run the roscore:
```
roscore
```
Open another command window in the `CSSR` folder and run the test program to provide control messages to CSSR:
```
source env/bin/activate
source devel/setup.bash
rosrun cssr simtest.py
```
When your own decision-making and planning system is finished, you no longer need to run the code above.  
Open another command window in the `CSSR` folder and run the software:
```
source env/bin/activate
source devel/setup.bash
rosrun cssr simmain.py
```
Then you can see the software interface of CSSR.

## Use CSSR
### Set Parameters
`simmain.py` defines some simulation parameters.
``` python
# Parameters
# Square boundary (1 corresponds to 0.1 m)
boundary = 600
# Time interval
T = 1000 # 1000 ms
```
If you want to change the boundary of the figure, you can change the value of `boundary`. If you want to change the time interval between each simulation step, you can change the value of `T`.

### Generate Obstacles
If you want to generate fixed obstacles, you can check the `Fixed Obs` option in the combo box. Then the obstacles will be displayed in the figure.
  ![Generate Fixed Obstacles](demos/1.jpg)
If you want to generate random obstacles, you can check the `Random Obs` option in the combo box, and change the number of obstacles in the spin box. Then the obstacles will be displayed in the figure. If you are not satisfied with the random obstacles, you can click the spin box and press `Enter` to change the random obstacles.
  ![Generate Random Obstacles](demos/2.jpg)

### Run Simulation
Click the `Run` button to run the simulation. Click the `Stop` button to stop the simulation. 
![Run the simulation](demos/3.jpg)
Click the `Clear` button to clear the obstacles and trajectories in the figure.
![Clear the figure](demos/4.jpg)

### Save Figure
Click the `Save` button to save the figure. Then you can choose the filename and the path to save the figure in the pop-up window.
![Save Figure](demos/5.jpg)
The saved figure is shown below:
![Saved figure](demos/demo.png)

### Exit
To exit the program, please click the `×` in the top right corner of the window.

## History Versions
- Version 1.0 (January 31, 2025)
  - The design objectives have been initially achieved.

## Future Developments
Although the design objectives have been initially achieved, there may still be some aspects to improve depending on the requirements.
- Make the software interface more suitable, elegant, and user-friendly
- Complete the assignment of the message
- Generate more diverse obstacles
- Change the car model from dot to rectagle
- Improve the control logic of the car
- Add the function of detecting collisions between the car and the obstacles
- Solve the potential bugs in the software

## Acknowledgments
I am grateful to Professor Xu and my senior colleagues for their meticulous guidance and assistance throughout the  development process.

## Contact
Developer<br>
[Zhmy](https://myTristan.github.io/)
<br>
If there are any shortcomings in CSSR, please feel free to email me. I welcome your corrections.
<!-- If there are any shortcomings in my answers, please feel free to email me (zhmy22@mails.tsinghua.edu.cn). I welcome your corrections. -->
