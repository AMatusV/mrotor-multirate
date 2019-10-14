Multirate Integration Methods for Multicopters

Overview

This repository contains multirate integration methods for multirotors. Typically, mathematical models of multicopters are integrated by well known standard numerical methods, whose computational efficiency could not be suitable for scenarios where time is bounded. However, these models are described by slow and fast dynamics that could be decoupled and resolved by numerical methods tailored to such dynamics and with a specific integration step. In this sense, we present a set of special numerical methods that take advantage of the fact that the set of differential equations corresponding to the slow dynamics could be integrated with a larger integration step than that used for the fast dynamics. This proposed approach saves computational effort, thus enabling performance within time limitations. Furthermore, we deploy this multirate approach, which parallelizes the numerical integration of the model, in a control application in which a quadrotor navigates an unknown environment with onboard sensors only. We demonstrate how the multirate integration method can be used to increase the performance of a position controller consuming estimations from an accurate but slow metric monocular SLAM system. Our control application can be evaluated in simulation (Gazebo) or in real-world experiments.

We provide a folder with a Visual Studio solution (qrIntegMethods) containing three numerical integration methods: four-order Runge-Kutta (baseline), multirate with heuristic partition, and multirate with Selective Model Analysis partition. For each multirate partition, we can select three inference strategies for the off-mesh points of the slow subsystem: interpolation, forward information, and backward information. Also, we provide a ROS package (beb_mr_ctrl) with the control application for a Parrot Bebop 2 quadcopter, which can be simulated in Gazebo as a AR.Drone quadcopter. Natively, the code was written for Kinetic Kame. Natively, the code was written for Kinetic Kame. The package has the following dependencies:


-angles

-control_toolbox

-bebop_msgs
