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

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
