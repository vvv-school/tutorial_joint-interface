Tutorial on Joint Interface
===========================

This tutorial will guide you to the use of **Motor Interfaces** to control the
iCub movements in the **joint space** relying on the following modalities:

- [**Position Control**](http://www.yarp.it/classyarp_1_1dev_1_1IPositionControl2.html)
- [**Velocity Control**](http://www.yarp.it/classyarp_1_1dev_1_1IVelocityControl2.html)
- [**Position-Direct Control**](http://www.yarp.it/classyarp_1_1dev_1_1IPositionDirect.html)

# Tutorial
We will show you how to control the elbow joint of the iCub arm in three different
contexts:

1. With a **single call** delivering the **final position set-point** to the low level layer.
1. With a **controller thread** that continuously computes the **velocity commands** to reach for a target.
1. With a **planner thread** streaming suitable **position references** used by the low level layer.

In the end, we will obtain the joint moving as below.

![output](/misc/output.gif)

## Notes

You might want to play with our [**yarpmotorgui**](http://www.yarp.it/yarpmotorgui.html), which lets you interact with the joint control of the robot.

# Diagrams of low-level control modalities

### Position Control
The high-level user code provides a **one-shot command** in terms of **final position set-point** to achieve along with the **time required** to attain it (or, equivalently, the corresponding speed). Then, the low-level control is responsible for generating a suitable trajectory (typically **minimum-jerk** profiles) connecting the current joint angle with the desired set-point as well as for controlling the motor to let the joint reach for the target. Thus, the **control takes place exclusively at the low-level**, whereas the high-level only sends the command to initiate the movement.

![position](/misc/position.png)

### Velocity Control
The high-level user code is responsible for **continuosly providing joint velocity** to be used to solve a given task defined upstream (e.g. reach a position). The low-level layer is only dealing with the actual control of the motor, **integrating instantly the received velocity** into a position set-point. Therefore, part of the control **complexity is shifted to higher level**.

![velocity](/misc/velocity.png)

### Position Direct Control
The high-level user code is responsible for **continuosly providing position references** to be used to solve a given task defined upstream. The low-level layer is only dealing with the actual control of the motor. This modality allows for doing **position planning** at higher level, since we do not need to read back the encoders to perform control upstream as in the case of velocity control. As a result, we can achieve **faster movements with the freedom to plan generic joint trajectories**.

![position-direct](/misc/position-direct.png)

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
