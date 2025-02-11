# robust inverse dynamics control

From: [Continuous State Feedback Guaranteeing Uniform Ultimate Boundedness for Uncertain Dynamic Systems](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1102785&tag=1)

required ROS1 and [rosdyn](https://github.com/CNR-STIIMA-IRAS/rosdyn)

required parameters:
``` yaml
natural_frequency: 20   # natural frequency of the closed loop
damping: 1              # relative damping of the closed loop  
robustness_gain: 0.01   # robustness gain
```
