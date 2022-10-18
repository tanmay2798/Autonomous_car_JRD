# Autonomous Car JRD

I have designed a lane keep assist system, initially understanding and working on raw color and image detection edge detection algorithms for images and integrated and tested on ROS (Robot Operating System). Lane detection algorithms were made robust by employing various strategies like Kalman filter. I tested these on data from Zed Stereo camera: mounted on the windshield pointing towards the front of the car. I tested it on all the roads inside Indian Institute of Technology Delhi. Following this I planned the lane following motion of the car on straight, curved and discontinuous lane roads. A simulation environment on CARLA along with its proper control inputs was designed for the project. All the functionalities and features which were being tested in the real car were successfully incorporated in the simulator. The ROS-bridge was used to control the vehicle and get the sensor data from CARLA. Then the work was directed towards further improving control by applying, tuning, and optimising the PID control in the CARLA simulator, assuming parameters for the real world can be found close to them.

## This folder consists of the codes of implementation of the algorithm used:

1. Simulation of the autonomous movement of car in CARLA simulator.

2. Computer vision algorithms to detect the lanes on both sides of the roads.

3. Kalman Filter to predict the lanes using the error values and new lane values.

4. Testing of the algorithm in the Real world car.

5. Implementation of ROS functionality for the control of the car.

6. Algorithms for improving the control of the car to make it move on curved and discontinuous roads.

7. PID control implementation and parameters tuning for faster corrections.
[![Watch the video](https://i.imgur.com/vKb2F1B.png)](https://www.youtube.com/watch?v=el0VRzcktlQ)

Final presentation - [click here](https://docs.google.com/presentation/d/1D70iyy52JKXWYugCRAvB1wr3chXNPb1twON8NtktRFM/edit?usp=sharing)
