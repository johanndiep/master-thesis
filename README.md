# Ultra-Wideband Localization Error Correction using Gaussian Process for UAV Flight

This GitLab repository contains the code for the "Ultra-Wideband Localization Error Correction using Gaussian Process for Teach-and-Repeat UAV Flight" project. It was developed in the context of a master thesis at ETH Zurich.


## Context

In recent years, unmanned aerial vehicles (UAV) have gained more interests in the research community. Due to their improved videography capabilities, they have the potential to bridge the gap between precise and flexible filming. However, their current localization methods are
not reliable enough for tasks such as the following of a predefined trajectory.

Motivated by this gap, the work in the context of this thesis is to establish an accurate localization system for UAVs based on commercially available ultra-wideband (UWB) technology. It describes the implementation of a system based on filtered multilateration using distance measurements between the drone-mounted tag and several environmentfixed anchors. Here, methods are presented to estimate the anchor placements as well as to correct for the systematic biases of the range readings. The former is solved by means of Gauss-Newton optimization using inter-anchor distance estimations. For the latter, in order to correct for the systematic distance measurement errors, a teach-and-repeat framework is proposed, where Gaussian Process models are derived from training data given by a motion capture system and incorporated in the corresponding equation of the state estimation algorithm.

## Installation

This installation was tested for Ubuntu 16.04 LTS. For other operating systems, changes or additional packages might be required. The following software and packages were used:

* [Matlab 9.6](https://www.mathworks.com/products/matlab.html)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [picocom](https://www.github.com/npat-efault/picocom) serial communication program
* [bebop autonomy ROS driver](https://bebop-autonomy.readthedocs.io/en/latest/) for the Bebop drone
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


The following lists the corresponding executable scripts:

* **AnchorCalibMain.m:**  Self-calibration of the anchor setup.
* **CircOffsetDataMain.m:** Collection of data along the circular trajectory.
* **SplineOffsetDataMain.m:** Collection of data along the spline trajectory.
* **CovEvalMain.m:** Implementation of the Gaussian Process hyperparameter learning.
* **CircleControl.m:** Circular flight using Vicon data.
* **UWBCircleControl.m:**  Circular flight using UWB data and uncorrected measurement model.
* **GPSCircleControl.m:** Circular flight using UWB data and corrected measurement model.
* **SplineControl.m:** Spline flight using Vicon data.
* **UWBSplineControl.m:**  Spline flight using UWB data and uncorrected measurement model.
* **GPSplineControl.m:** Spline flight using UWB data and corrected measurement model.

## Starting Instructions

This subsection provide the basic instructions in order to run the executables. The following commands should be executed in individual terminal tabs and in order.

**Enabling Serial Communication with Matlab**

Use this command after connecting the modified sniffer to the ground station computer. Since the exact USB port enumeration can differ, it is recommended to apply command-line completion here.

```console
$ sudo chmod 666 /dev/ttyACM0
```

**Starting Spacemouse ROS Driver**

```console
$ roslaunch spacenav_node classic.launch
```

**Starting Vicon ROS Driver**

```console
$ cd vicon_ws
$ source devel/setup.bash
$ cd src/vicon_bridge/launch
$ roslaunch vicon.launch
```

**Starting Bebop ROS Driver**

The following instructions starts the driver after connecting to the specific Wi-Fi of the drone.

```console
$ cd bebop_ws
$ source devel/setup.bash
$ cd src/bebop_autonomy/bebop_driver/launch
$ roslaunch bebop_node.launch
```

**Bebop Forced Landing**

For yet inexplicable reasons, the drone occasionally does not react to the landing command given using the Spacemouse. In those cases, the drone can be forced to land by using the following command:

```console
$ rostopic pub /bebop/land std_msgs/Empty "{}"
```

## Example: Circular Flight

Video comparison between corrected and uncorrected measurement model on flight accuracy.

[![Corrected Model](https://i.imgur.com/kWOU5PV.png)](https://www.youtube.com/watch?v=YJo8tMqly8E)
[![Uncorrected Model](https://i.imgur.com/gY58LXf.png)](https://www.youtube.com/watch?v=BqnEMfDYWC4)

## Version

* 7. January 2020: Version 1.0, [Documentation](https://gitlab.com/jdiep/master-thesis/tree/master/documentation)

## Credential

The core of this work is based on the publication ["Ultra-Wideband Range Measurement Model with Gaussian Processes"](https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/People/antonl/cctaconf17.pdf) by Anton Ledergerber and Rafaello D'Andrea.

## Authors

* Johann Diep (MSc Student Mechanical Engineering, ETH Zurich, jdiep@student.ethz.ch)
* In cooperation with [Tinamu Labs AG](http://tinamu-labs.com/wp/).