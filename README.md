# Ultra-Wideband Localization Error Correction using Gaussian Process for Teach-and-Repeat UAV Flight

This GitLab repository contains the code for the "Ultra-Wideband Localization Error Correction using Gaussian Process for Teach-and-Repeat UAV Flight" project. It was developed in the context of a master thesis at ETH Zurich.

## Context

In recent years, unmanned aerial vehicles (UAV) have gained more interests in the research community. Due to their improved videography capabilities, they have the potential to bridge the gap between precise and flexible filming. However, their current localization methods are
not reliable enough for tasks such as the following of a predefined trajectory.

Motivated by this gap, the work in the context of this thesis is to establish an accurate localization system for UAVs based on commercially available ultra-wideband (UWB) technology. It describes the implementation of a system based on filtered multilateration using distance measurements between the drone-mounted tag and several environmentfixed anchors. Here, methods are presented to estimate the anchor placements as well as to correct for the systematic biases of the range readings. The former is solved by means of Gauss-Newton optimization using inter-anchor distance estimations. For the latter, in order to correct for the systematic distance measurement errors, a teach-andrepeat framework is proposed, where Gaussian Process models are derived from training data given by a motion capture system and incorporated in the corresponding equation of the state estimation algorithm.

## Installation

This installation was tested for Ubuntu 16.04 LTS. For other operating systems, changes or additional packages might be required. The following software and packages were used:

* [Matlab 9.6](https://www.mathworks.com/products/matlab.html)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [picocom](https://www.github.com/npat-efault/picocom) serial communication program
* [bebop autonomy ROS driver](https://www.bebop-autonomy.readthedocs.io/en/latest/) for the Bebop drone
* [vicon bridge ROS driver](http://wiki.ros.org/vicon_bridge) for the Vicon motion capture system
* [spacenav node ROS driver](http://wiki.ros.org/spacenav_node) for the Spacemouse
* [GPy](https://www.github.com/SheffieldML/GPy) Gaussian Process Python framework (optional)

Additional libraries might be required, install on request.

## Hardware and Platforms

The following hardware and platforms were used:

* [Loco Positioning Node](https://www.bitcraze.io/loco-pos-node/)
* [Parrot Bebop 2](https://www.parrot.com/us/drones/parrot-bebop-2)
* [Vicon Motion Capture System](https://www.vicon.com/)

## Matlab Code Structure

All the source code for the flight logic are stored in the matlab folder and commented extensively. The following lists the most important classes and functions:

* **BebopControl.m:** Interface to the bebop autonomy library.
* **getGroundTruth.m:** Interface for the Vicon measurements.
* **Controller.m:**  Implementation of the position and attitude controller.
* **TrajectoryGenerator.m:** Construction of the specific trajectory.
* **ConstantVelocityEKF.m:**  EKF implementation using Vicon data.
* **ConstantVelocityUWB.m:** EKF implementation using UWB data.
* **ConstantVelocityGP.m:**  Gaussian Process EKF implementation using UWB data.
* **DataPrep.m:** Data preprocessing for Gaussian Process.
* **RBFKernel.m:** Calculation of the radial basis kernel matrix.
* **getLogLikelihood.m:** Returning the marginal log likelihood.
* **GaussianModel.m:**  Returning all the terms for Gaussian Process regression.
* **GaussianPrediction.m:** Returning the distribution for a specific input.


The [main](https://gitlab.com/jdiep/semester-thesis/tree/master/neural_mesh_renderer/OBMBACPE) folder contains the executables of the classes mentioned above which can be run individually depending on the desired output:

* **pose estimation.py:**<br/>
This is the main executable file of this project. It finds the pose of the blurred input image via an optimization process.
* **3d representation.py:**<br/> 
This program produces a 3D pointcloud and polygon-mesh representation of the environment in the intial frame. The generated txt- and obj-file can be observed in Meshlab.
* **image generator.py:**<br/> 
This program generates sharp and blurry images at arbitrary poses.

## Example: Optimization Process

Artificial blurred images generated during optimization.


## Version

* 7. January 2020: Version 1.0, [Documentation](https://gitlab.com/jdiep/semester-thesis/tree/master/documentation)

## Authors

* Johann Diep (MSc Student Mechanical Engineering, ETH Zurich, jdiep@student.ethz.ch)